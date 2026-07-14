#include <algorithm>
#include <cmath>
#include <map>
#include <regex>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

// 圆周率常量
constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 计算二维向量 (x,y) 的欧几里得范数
 */
double norm2d(double x, double y) {
    return std::sqrt(x * x + y * y);
}

/**
 * @brief 单个目标的状态结构体
 *
 * 存储目标的估计里程计信息、时间戳和有效性标志
 */
struct TargetState {
    nav_msgs::Odometry estimate;   // 目标估计（位置+速度）
    ros::Time stamp;               // 最后一次收到估计的时间戳
    bool has_estimate = false;     // 是否已收到有效估计
};

}  // namespace

/**
 * @brief 协同制导节点类
 *
 * 订阅分配结果话题（/assignment/result），解析每个目标分配的拦截器列表，
 * 并根据目标估计状态为每个拦截器生成捕获目标（capture_goal）。
 * 捕获目标采用“包围圈”模式：多个拦截器均匀分布在以目标预测位置为中心的圆周上，
 * 并考虑前馈时间（lead_time）进行提前量补偿。
 * 同时发布可视化标记（球体、连线、文本）。
 */
class CooperativeGuidanceNode {
public:
    CooperativeGuidanceNode() : nh_(), pnh_("~") {
        loadParams();                      // 加载参数

        targets_.resize(target_count_);    // 初始化目标状态数组
        assignments_.resize(interceptor_count_, -1); // 初始化分配数组（-1表示未分配）

        createRosInterfaces();             // 创建订阅器和发布器

        // 创建定时器，按指定频率执行制导计算
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(update_rate_, 1e-3)),
            &CooperativeGuidanceNode::timerCallback,
            this);

        ROS_INFO("cooperative_guidance started with %d targets and %d interceptors",
                 target_count_,
                 interceptor_count_);
    }

private:
    /**
     * @brief 从参数服务器加载配置参数
     */
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<int>("interceptor_count", interceptor_count_, 6);
        pnh_.param<double>("update_rate", update_rate_, 10.0);                 // 制导更新频率（Hz）
        pnh_.param<std::string>("assignment_topic", assignment_topic_, "/assignment/result"); // 分配结果话题
        pnh_.param<std::string>("goal_prefix", goal_prefix_, "/interceptor_"); // 拦截器捕获目标话题前缀
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");  // 目标估计话题前缀
        pnh_.param<double>("stale_timeout", stale_timeout_, 2.0);              // 目标估计超时时间（秒）
        pnh_.param<double>("capture_radius", capture_radius_, 8.0);            // 捕获包围圈半径（米）
        pnh_.param<double>("vertical_offset", vertical_offset_, 0.0);          // 垂直偏移量（米）
        pnh_.param<double>("lead_time", lead_time_, 1.0);                      // 前馈时间（秒），用于预测目标未来位置
        pnh_.param<double>("min_goal_altitude", min_goal_altitude_, 2.0);      // 捕获目标最低高度（米）
        pnh_.param<double>("marker_scale", marker_scale_, 0.8);                // 可视化球体大小
        pnh_.param<double>("line_width", line_width_, 0.12);                   // 可视化连线宽度
    }

    /**
     * @brief 创建所有ROS订阅器和发布器
     *
     * 订阅：
     * - 分配结果话题（/assignment/result）
     * - 每个目标的估计话题（/target_{i}/estimate）
     *
     * 发布：
     * - 每个拦截器的捕获目标话题（/interceptor_{i}/capture_goal）
     * - 可视化标记数组（/cooperative_guidance/markers）
     */
    void createRosInterfaces() {
        // 订阅分配结果
        assignment_sub_ = nh_.subscribe(assignment_topic_,
                                        10,
                                        &CooperativeGuidanceNode::assignmentCallback,
                                        this);

        // 订阅每个目标的估计
        for (int i = 0; i < target_count_; ++i) {
            const std::string estimate_topic =
                target_prefix_ + std::to_string(i) + "/estimate";

            target_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    estimate_topic,
                    10,
                    [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                        targetEstimateCallback(msg, i);
                    }));
        }

        // 为每个拦截器创建捕获目标发布器
        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string capture_goal_topic =
                goal_prefix_ + std::to_string(i) + "/capture_goal";

            capture_goal_pubs_.push_back(
                nh_.advertise<geometry_msgs::PoseStamped>(capture_goal_topic, 1, true));
        }

        // 创建可视化标记发布器
        marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/cooperative_guidance/markers", 1);
    }

    /**
     * @brief 目标估计回调：更新对应目标的估计状态和时间戳
     * @param msg 里程计估计消息
     * @param target_index 目标索引
     */
    void targetEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= target_count_) {
            return;
        }

        targets_[target_index].estimate = *msg;
        targets_[target_index].stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        targets_[target_index].has_estimate = true;
    }

    /**
     * @brief 分配结果回调：解析字符串格式的分配结果，更新内部分配映射
     * @param msg 分配结果字符串消息
     *
     * 解析格式示例：target_0(0.800) <- interceptor_2,interceptor_5
     * 使用正则表达式提取目标ID和拦截器ID列表
     */
    void assignmentCallback(const std_msgs::String::ConstPtr& msg) {
        // 重置所有分配状态
        std::fill(assignments_.begin(), assignments_.end(), -1);
        grouped_assignments_.clear();

        // 正则表达式：匹配 "target_N(...) <- interceptor_M,..."
        const std::regex block_regex("target_([0-9]+)\\([^)]*\\)\\s*<-\\s*([^|]+)");
        // 正则表达式：匹配 "interceptor_M"
        const std::regex interceptor_regex("interceptor_([0-9]+)");

        auto block_begin =
            std::sregex_iterator(msg->data.begin(), msg->data.end(), block_regex);
        const auto block_end = std::sregex_iterator();

        // 遍历每个目标块
        for (auto block_it = block_begin; block_it != block_end; ++block_it) {
            const int target_id = std::stoi((*block_it)[1].str());
            const std::string interceptor_list = (*block_it)[2].str();

            // 在该块中查找所有拦截器ID
            auto interceptor_begin =
                std::sregex_iterator(interceptor_list.begin(),
                                     interceptor_list.end(),
                                     interceptor_regex);
            const auto interceptor_end = std::sregex_iterator();

            for (auto interceptor_it = interceptor_begin;
                 interceptor_it != interceptor_end;
                 ++interceptor_it) {
                const int interceptor_id = std::stoi((*interceptor_it)[1].str());

                // 有效性检查
                if (target_id < 0 || target_id >= target_count_ ||
                    interceptor_id < 0 || interceptor_id >= interceptor_count_) {
                    continue;
                }

                // 记录分配关系
                assignments_[interceptor_id] = target_id;
                grouped_assignments_[target_id].push_back(interceptor_id);
            }
        }
    }

    /**
     * @brief 定时器回调：为每个有分配关系的目标-拦截器组生成捕获目标并发布
     *
     * 遍历分组分配表，为每个目标下的所有拦截器生成捕获目标，
     * 同时生成对应的可视化标记。
     */
    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();

        visualization_msgs::MarkerArray markers;
        int marker_id = 0;

        // 遍历每个目标的分组
        for (const auto& item : grouped_assignments_) {
            const int target_id = item.first;
            const std::vector<int>& interceptors = item.second;

            // 跳过无效或过期的目标
            if (!isTargetFresh(target_id, now) || interceptors.empty()) {
                continue;
            }

            // 为该目标的所有拦截器生成捕获目标
            for (size_t slot = 0; slot < interceptors.size(); ++slot) {
                const int interceptor_id = interceptors[slot];

                // 生成捕获目标（考虑包围圈位置和前馈）
                geometry_msgs::PoseStamped capture_goal =
                    makeCaptureGoal(target_id,
                                    static_cast<int>(slot),
                                    static_cast<int>(interceptors.size()),
                                    now);

                // 发布到对应拦截器的捕获目标话题
                capture_goal_pubs_[interceptor_id].publish(capture_goal);

                // 添加可视化标记
                addGoalMarkers(markers,
                               marker_id,
                               interceptor_id,
                               target_id,
                               capture_goal);
            }
        }

        // 发布所有标记
        marker_pub_.publish(markers);
    }

    /**
     * @brief 检查目标估计是否在有效期内
     * @param target_id 目标索引
     * @param now 当前时间
     * @return true 表示目标有效且新鲜
     */
    bool isTargetFresh(int target_id, const ros::Time& now) const {
        if (target_id < 0 || target_id >= target_count_) {
            return false;
        }

        const TargetState& target = targets_[target_id];

        if (!target.has_estimate) {
            return false;
        }

        return (now - target.stamp).toSec() <= stale_timeout_;
    }

    /**
     * @brief 为指定目标下的某个拦截器生成捕获目标位姿
     * @param target_id 目标索引
     * @param slot 该拦截器在分组中的槽位（0-based）
     * @param slot_count 该目标的总拦截器数量
     * @param stamp 当前时间戳
     * @return 捕获目标位姿（PoseStamped）
     *
     * 捕获目标位置计算：
     *   基准点 = 目标当前位置 + 目标速度 * lead_time（前馈）
     *   最终位置 = 基准点 + 极坐标偏移（半径为capture_radius，角度均匀分布）
     *   高度 = max(min_goal_altitude, 目标高度 + vertical_offset)
     */
    geometry_msgs::PoseStamped makeCaptureGoal(int target_id,
                                               int slot,
                                               int slot_count,
                                               const ros::Time& stamp) const {
        const nav_msgs::Odometry& estimate = targets_[target_id].estimate;
        const geometry_msgs::Point& target_pos = estimate.pose.pose.position;
        const geometry_msgs::Vector3& target_vel = estimate.twist.twist.linear;

        // 计算目标速度方向角（航向角）
        double heading = std::atan2(target_vel.y, target_vel.x);
        if (norm2d(target_vel.x, target_vel.y) < 1e-3) {
            heading = 0.0;  // 速度为零时默认朝X轴正方向
        }

        // 该拦截器在包围圈上的角度偏移（均匀分布，并以目标航向为基准）
        const double angle =
            heading +
            2.0 * kPi * static_cast<double>(slot) / std::max(1, slot_count);

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = stamp;
        goal.header.frame_id = world_frame_;

        // 计算捕获目标位置
        goal.pose.position.x =
            target_pos.x + lead_time_ * target_vel.x + capture_radius_ * std::cos(angle);
        goal.pose.position.y =
            target_pos.y + lead_time_ * target_vel.y + capture_radius_ * std::sin(angle);
        goal.pose.position.z =
            std::max(min_goal_altitude_, target_pos.z + vertical_offset_);

        // 无旋转
        goal.pose.orientation.w = 1.0;
        return goal;
    }

    /**
     * @brief 为单个拦截器的捕获目标添加可视化标记
     * @param markers 标记数组引用
     * @param marker_id 当前标记ID（递增）
     * @param interceptor_id 拦截器ID
     * @param target_id 目标ID
     * @param goal 捕获目标位姿
     *
     * 添加三种标记：
     * - 球体：表示捕获目标位置（青色）
     * - 连线：从目标当前位置到捕获目标位置
     * - 文本：显示 "Ix -> Ty"
     */
    void addGoalMarkers(visualization_msgs::MarkerArray& markers,
                        int& marker_id,
                        int interceptor_id,
                        int target_id,
                        const geometry_msgs::PoseStamped& goal) const {
        // 捕获目标球体
        visualization_msgs::Marker sphere;
        sphere.header = goal.header;
        sphere.ns = "interceptor_capture_goals";
        sphere.id = marker_id++;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose = goal.pose;
        sphere.scale.x = marker_scale_;
        sphere.scale.y = marker_scale_;
        sphere.scale.z = marker_scale_;
        sphere.color.r = 0.1;
        sphere.color.g = 0.9;
        sphere.color.b = 1.0;
        sphere.color.a = 0.9;
        markers.markers.push_back(sphere);

        // 从目标当前位置到捕获目标的连线
        visualization_msgs::Marker line;
        line.header = goal.header;
        line.ns = "guidance_target_lines";
        line.id = marker_id++;
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = line_width_;
        line.color.r = 0.2;
        line.color.g = 0.8;
        line.color.b = 1.0;
        line.color.a = 0.7;
        line.points.push_back(goal.pose.position);
        line.points.push_back(targets_[target_id].estimate.pose.pose.position);
        markers.markers.push_back(line);

        // 文本标签（显示拦截器→目标关系）
        visualization_msgs::Marker text;
        text.header = goal.header;
        text.ns = "guidance_goal_text";
        text.id = marker_id++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose = goal.pose;
        text.pose.position.z += 1.0;  // 在球体上方显示
        text.scale.z = 0.9;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 0.95;
        text.text = "I" + std::to_string(interceptor_id) +
                    " -> T" + std::to_string(target_id);
        markers.markers.push_back(text);
    }

    // ROS句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    // 参数
    std::string world_frame_ = "world";           // 世界坐标系
    int target_count_ = 3;                        // 目标数量
    int interceptor_count_ = 6;                   // 拦截器数量
    double update_rate_ = 10.0;                   // 制导更新频率（Hz）

    std::string assignment_topic_ = "/assignment/result";   // 分配结果话题
    std::string goal_prefix_ = "/interceptor_";             // 拦截器捕获目标话题前缀
    std::string target_prefix_ = "/target_";                // 目标估计话题前缀

    double stale_timeout_ = 2.0;                  // 目标估计超时时间（秒）
    double capture_radius_ = 8.0;                 // 捕获包围圈半径（米）
    double vertical_offset_ = 0.0;                // 垂直偏移量（米）
    double lead_time_ = 1.0;                      // 前馈时间（秒）
    double min_goal_altitude_ = 2.0;              // 捕获目标最低高度（米）
    double marker_scale_ = 0.8;                   // 可视化球体大小
    double line_width_ = 0.12;                    // 可视化连线宽度

    // 数据容器
    std::vector<TargetState> targets_;                      // 所有目标的状态
    std::vector<int> assignments_;                          // 每个拦截器分配的目标ID（-1表示未分配）
    std::map<int, std::vector<int>> grouped_assignments_;   // 按目标分组的拦截器ID列表

    // ROS通信接口
    ros::Subscriber assignment_sub_;                        // 分配结果订阅器
    std::vector<ros::Subscriber> target_subs_;              // 目标估计订阅器列表
    std::vector<ros::Publisher> capture_goal_pubs_;         // 捕获目标发布器列表

    ros::Publisher marker_pub_;                             // 可视化标记发布器
};

/**
 * @brief 主函数：初始化节点，创建协同制导节点实例，进入ROS事件循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "cooperative_guidance_node");
    CooperativeGuidanceNode node;
    ros::spin();
    return 0;
}