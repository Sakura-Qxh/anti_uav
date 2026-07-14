#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

/**
 * @brief 计算两个三维点之间的欧几里得距离
 */
double distance3(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 将数值钳制到 [0, 1] 区间
 */
double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 单个目标的状态结构体
 *
 * 存储目标的估计、威胁值、分配到的拦截器列表等信息
 */
struct TargetState {
    nav_msgs::Odometry estimate;           // 目标当前估计（位置+速度）
    ros::Time last_estimate_time;          // 最后一次收到估计的时间
    ros::Time last_threat_time;            // 最后一次收到威胁值的时间
    bool has_estimate = false;             // 是否已收到估计
    bool has_threat = false;               // 是否已收到威胁值
    double threat = 0.0;                   // 威胁分数 [0,1]
    std::vector<int> assigned_interceptors; // 已分配到此目标的拦截器ID列表
};

/**
 * @brief 单个拦截器的状态结构体
 *
 * 存储拦截器的当前位置、归位位置、分配情况等
 */
struct InterceptorState {
    int id = 0;                              // 拦截器编号
    geometry_msgs::Point home_position;      // 归位位置（地面站/发射点）
    geometry_msgs::Point position;           // 当前位置（来自里程计）
    ros::Time last_odom_time;                // 最后一次收到里程计的时间
    bool has_odom = false;                   // 是否已收到里程计
    bool assigned = false;                   // 当前是否已被分配
    int assigned_target = -1;                // 分配到的目标ID（-1表示未分配）
};

/**
 * @brief 目标排序结构体（用于按威胁降序排列）
 */
struct TargetOrder {
    int id = 0;           // 目标编号
    double threat = 0.0;  // 威胁分数
};

/**
 * @brief 候选分配结构体（用于贪心选择代价最小的拦截器）
 */
struct Candidate {
    int interceptor_id = -1;                                        // 拦截器ID
    double cost = std::numeric_limits<double>::infinity();          // 分配代价
};

/**
 * @brief 协同分配节点类
 *
 * 订阅每个目标的估计和威胁值，以及每个拦截器的里程计信息。
 * 根据威胁等级确定每个目标所需的拦截器数量，然后基于代价函数
 * （距离、时间、威胁加权）贪心地分配空闲拦截器。
 * 发布分配结果文本和RViz可视化标记。
 */
class CooperativeAssignmentNode {
public:
    CooperativeAssignmentNode() : nh_(), pnh_("~") {
        // 加载参数
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<int>("interceptor_count", interceptor_count_, 6);
        pnh_.param<double>("update_rate", update_rate_, 5.0);                     // 分配更新频率（Hz）
        pnh_.param<int>("max_interceptors_per_target", max_interceptors_per_target_, 3); // 每个目标最多分配的拦截器数
        pnh_.param<double>("high_threat_threshold", high_threat_threshold_, 0.70); // 高威胁阈值
        pnh_.param<double>("medium_threat_threshold", medium_threat_threshold_, 0.40); // 中威胁阈值
        pnh_.param<int>("high_threat_interceptors", high_threat_interceptors_, 3); // 高威胁目标分配数
        pnh_.param<int>("medium_threat_interceptors", medium_threat_interceptors_, 2); // 中威胁目标分配数
        pnh_.param<int>("low_threat_interceptors", low_threat_interceptors_, 1);   // 低威胁目标分配数
        pnh_.param<double>("interceptor_speed", interceptor_speed_, 30.0);         // 拦截器速度参考值（米/秒）
        pnh_.param<double>("max_assignment_distance", max_assignment_distance_, 300.0); // 最大分配距离（米）
        pnh_.param<double>("stale_timeout", stale_timeout_, 2.0);                  // 目标估计/威胁超时时间（秒）
        pnh_.param<double>("interceptor_odom_timeout", interceptor_odom_timeout_, 2.0); // 拦截器里程计超时时间
        pnh_.param<std::string>("interceptor_prefix", interceptor_prefix_, "/interceptor_"); // 拦截器话题前缀
        pnh_.param<double>("cost_distance_weight", cost_distance_weight_, 0.55);   // 代价函数中距离权重
        pnh_.param<double>("cost_time_weight", cost_time_weight_, 0.35);           // 代价函数中时间权重
        pnh_.param<double>("cost_threat_weight", cost_threat_weight_, 0.45);       // 代价函数中威胁权重（负项）
        pnh_.param<double>("interceptor_marker_scale", interceptor_marker_scale_, 1.0); // 拦截器标记大小
        pnh_.param<double>("line_width", line_width_, 0.18);                       // 连线宽度
        pnh_.param<double>("text_height", text_height_, 1.2);                      // 文本标签高度
        pnh_.param<double>("ground_station_marker_altitude", ground_station_marker_altitude_, 0.2); // 地面站标记高度

        // 初始化目标状态容器
        targets_.resize(target_count_);
        // 加载拦截器配置（归位位置）
        loadInterceptors();
        // 创建ROS通信接口
        createRosInterfaces();

        // 创建定时器，按指定频率执行分配
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_),
                                &CooperativeAssignmentNode::timerCallback,
                                this);

        ROS_INFO("cooperative_assignment started with %d targets and %d interceptors",
                 target_count_, interceptor_count_);
    }

private:
    /**
     * @brief 加载拦截器的归位位置（home_position）
     *
     * 从参数服务器读取 interceptors/{i} 数组，若不存在则使用默认位置
     */
    void loadInterceptors() {
        interceptors_.clear();
        for (int i = 0; i < interceptor_count_; ++i) {
            InterceptorState interceptor;
            interceptor.id = i;

            const std::string key = "interceptors/" + std::to_string(i);
            std::vector<double> pos;
            if (pnh_.getParam(key, pos) && pos.size() >= 3) {
                interceptor.home_position.x = pos[0];
                interceptor.home_position.y = pos[1];
                interceptor.home_position.z = pos[2];
            } else {
                // 默认排布：沿Y轴均匀分布
                interceptor.home_position.x = -40.0;
                interceptor.home_position.y = -30.0 + 12.0 * i;
                interceptor.home_position.z = 2.0;
            }
            interceptor.position = interceptor.home_position;  // 初始位置等于归位位置

            interceptors_.push_back(interceptor);
        }
    }

    /**
     * @brief 创建所有ROS订阅器和发布器
     *
     * 订阅：
     * - 每个目标的估计话题：/target_{i}/estimate
     * - 每个目标的威胁话题：/threat/target_{i}
     * - 威胁排名话题：/threat/ranking（仅用于日志）
     * - 每个拦截器的里程计话题：/interceptor_{i}/odom
     *
     * 发布：
     * - 分配结果字符串：/assignment/result
     * - 可视化标记：/assignment/markers
     */
    void createRosInterfaces() {
        estimate_subs_.clear();
        threat_subs_.clear();
        interceptor_odom_subs_.clear();

        // 订阅每个目标的估计和威胁
        for (int i = 0; i < target_count_; ++i) {
            const std::string estimate_topic = "/target_" + std::to_string(i) + "/estimate";
            const std::string threat_topic = "/threat/target_" + std::to_string(i);

            estimate_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                estimate_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    estimateCallback(msg, i);
                }));

            threat_subs_.push_back(nh_.subscribe<std_msgs::Float64>(
                threat_topic, 10, [this, i](const std_msgs::Float64::ConstPtr& msg) {
                    threatCallback(msg, i);
                }));
        }

        // 订阅威胁排名（仅用于输出日志）
        ranking_sub_ = nh_.subscribe<std_msgs::String>("/threat/ranking", 10,
                                                       &CooperativeAssignmentNode::rankingCallback,
                                                       this);

        // 订阅每个拦截器的里程计
        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string odom_topic =
                interceptor_prefix_ + std::to_string(i) + "/odom";
            interceptor_odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                odom_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    interceptorOdomCallback(msg, i);
                }));
        }

        // 创建发布器
        result_pub_ = nh_.advertise<std_msgs::String>("/assignment/result", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/assignment/markers", 1);
    }

    /**
     * @brief 目标估计回调：更新目标的估计状态和时间戳
     */
    void estimateCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= target_count_) {
            return;
        }
        targets_[target_index].estimate = *msg;
        targets_[target_index].last_estimate_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        targets_[target_index].has_estimate = true;
    }

    /**
     * @brief 目标威胁回调：更新目标的威胁值和时间戳
     */
    void threatCallback(const std_msgs::Float64::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= target_count_) {
            return;
        }
        targets_[target_index].threat = clamp01(msg->data);
        targets_[target_index].last_threat_time = ros::Time::now();
        targets_[target_index].has_threat = true;
    }

    /**
     * @brief 威胁排名回调：保存最新的排名文本（仅用于输出日志）
     */
    void rankingCallback(const std_msgs::String::ConstPtr& msg) {
        latest_ranking_text_ = msg->data;
    }

    /**
     * @brief 拦截器里程计回调：更新拦截器的当前位置和时间戳
     */
    void interceptorOdomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                                 int interceptor_index) {
        if (interceptor_index < 0 || interceptor_index >= interceptor_count_) {
            return;
        }
        interceptors_[interceptor_index].position = msg->pose.pose.position;
        interceptors_[interceptor_index].last_odom_time =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        interceptors_[interceptor_index].has_odom = true;
    }

    /**
     * @brief 定时器回调：执行分配流程
     *
     * 1. 清空所有分配
     * 2. 获取按威胁降序排列的有效目标列表
     * 3. 依次为每个目标分配拦截器（贪心策略）
     * 4. 发布分配结果和可视化标记
     */
    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        clearAssignments();

        std::vector<TargetOrder> ordered_targets = makeOrderedTargets(now);
        for (const auto& target_order : ordered_targets) {
            assignInterceptorsToTarget(target_order.id);
        }

        publishResult(now, ordered_targets);
        publishMarkers(now);
    }

    /**
     * @brief 清空所有目标和拦截器的分配状态
     */
    void clearAssignments() {
        for (auto& target : targets_) {
            target.assigned_interceptors.clear();
        }
        for (auto& interceptor : interceptors_) {
            interceptor.assigned = false;
            interceptor.assigned_target = -1;
        }
    }

    /**
     * @brief 检查目标是否新鲜（估计和威胁都未超时）
     * @param target 目标状态
     * @param now 当前时间
     * @return true 表示有效
     */
    bool isTargetFresh(const TargetState& target, const ros::Time& now) const {
        if (!target.has_estimate || !target.has_threat) {
            return false;
        }
        const double estimate_age = (now - target.last_estimate_time).toSec();
        const double threat_age = (now - target.last_threat_time).toSec();
        return estimate_age <= stale_timeout_ && threat_age <= stale_timeout_;
    }

    /**
     * @brief 生成按威胁降序排列的有效目标列表
     * @param now 当前时间
     * @return 排序后的目标顺序
     *
     * 只包含新鲜且威胁大于1e-3的目标
     */
    std::vector<TargetOrder> makeOrderedTargets(const ros::Time& now) const {
        std::vector<TargetOrder> ordered;
        for (int i = 0; i < target_count_; ++i) {
            if (isTargetFresh(targets_[i], now) && targets_[i].threat > 1e-3) {
                ordered.push_back({i, targets_[i].threat});
            }
        }

        std::sort(ordered.begin(), ordered.end(), [](const TargetOrder& a, const TargetOrder& b) {
            return a.threat > b.threat;
        });
        return ordered;
    }

    /**
     * @brief 根据威胁等级确定目标所需的拦截器数量
     * @param threat 威胁分数
     * @return 期望分配的拦截器数量
     */
    int desiredInterceptorCount(double threat) const {
        if (threat >= high_threat_threshold_) {
            return high_threat_interceptors_;
        }
        if (threat >= medium_threat_threshold_) {
            return medium_threat_interceptors_;
        }
        return low_threat_interceptors_;
    }

    /**
     * @brief 为指定目标贪心地分配拦截器
     * @param target_id 目标编号
     *
     * 从空闲且新鲜的拦截器中选出代价最小的若干，分配给该目标。
     * 代价函数综合考虑距离、飞行时间和威胁（威胁越高代价越低）。
     */
    void assignInterceptorsToTarget(int target_id) {
        if (target_id < 0 || target_id >= target_count_) {
            return;
        }

        TargetState& target = targets_[target_id];
        const int desired_count = std::min(max_interceptors_per_target_,
                                           desiredInterceptorCount(target.threat));

        // 收集所有符合条件的空闲拦截器及其代价
        std::vector<Candidate> candidates;
        for (const auto& interceptor : interceptors_) {
            if (interceptor.assigned) {
                continue;
            }
            if (!isInterceptorFresh(interceptor, ros::Time::now())) {
                continue;
            }

            const double cost = assignmentCost(interceptor, target);
            if (std::isfinite(cost)) {
                candidates.push_back({interceptor.id, cost});
            }
        }

        // 按代价升序排列
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.cost < b.cost;
        });

        // 选取代价最小的若干个拦截器分配给该目标
        for (const auto& candidate : candidates) {
            if (static_cast<int>(target.assigned_interceptors.size()) >= desired_count) {
                break;
            }
            if (candidate.interceptor_id < 0 || candidate.interceptor_id >= interceptor_count_) {
                continue;
            }

            InterceptorState& interceptor = interceptors_[candidate.interceptor_id];
            if (interceptor.assigned) {
                continue;
            }

            interceptor.assigned = true;
            interceptor.assigned_target = target_id;
            target.assigned_interceptors.push_back(interceptor.id);
        }
    }

    /**
     * @brief 检查拦截器是否新鲜（里程计未超时或无里程计时视为新鲜）
     * @param interceptor 拦截器状态
     * @param now 当前时间
     * @return true 表示可用
     */
    bool isInterceptorFresh(const InterceptorState& interceptor,
                            const ros::Time& now) const {
        if (!interceptor.has_odom) {
            return true;  // 从未收到里程计，视为可用
        }
        return (now - interceptor.last_odom_time).toSec() <= interceptor_odom_timeout_;
    }

    /**
     * @brief 计算将某个拦截器分配给某个目标的代价
     * @param interceptor 拦截器状态
     * @param target 目标状态
     * @return 代价（越小越好），若超出最大距离则返回无穷大
     *
     * 代价公式：
     *   cost = w_dist * (dist/maxDist) + w_time * (time/maxTime) - w_threat * threat
     */
    double assignmentCost(const InterceptorState& interceptor, const TargetState& target) const {
        geometry_msgs::Point target_position = target.estimate.pose.pose.position;
        const double distance = distance3(interceptor.position, target_position);
        if (distance > max_assignment_distance_) {
            return std::numeric_limits<double>::infinity();
        }

        const double time_to_go = distance / std::max(1.0, interceptor_speed_);
        const double distance_score = distance / std::max(1.0, max_assignment_distance_);
        const double time_score = time_to_go / std::max(1.0, max_assignment_distance_ / std::max(1.0, interceptor_speed_));

        return cost_distance_weight_ * distance_score +
               cost_time_weight_ * time_score -
               cost_threat_weight_ * target.threat;
    }

    /**
     * @brief 发布分配结果字符串
     * @param stamp 时间戳
     * @param ordered_targets 按威胁排序的目标列表
     *
     * 格式示例：
     *   stamp=123456.789 | threat_ranking=[...] | target_0(0.800) <- interceptor_2,interceptor_5
     */
    void publishResult(const ros::Time& stamp, const std::vector<TargetOrder>& ordered_targets) {
        std_msgs::String msg;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "stamp=" << stamp.toSec();

        if (!latest_ranking_text_.empty()) {
            oss << " | threat_ranking=[" << latest_ranking_text_ << "]";
        }

        for (const auto& item : ordered_targets) {
            const TargetState& target = targets_[item.id];
            if (target.assigned_interceptors.empty()) {
                continue;
            }

            oss << " | target_" << item.id << "(" << target.threat << ") <- ";
            for (size_t k = 0; k < target.assigned_interceptors.size(); ++k) {
                if (k > 0) {
                    oss << ",";
                }
                oss << "interceptor_" << target.assigned_interceptors[k];
            }
        }

        msg.data = oss.str();
        result_pub_.publish(msg);
    }

    /**
     * @brief 发布RViz可视化标记
     *
     * 包括：
     * - 每个拦截器的地面站位置（立方体，颜色区分是否已分配）
     * - 拦截器编号标签（显示分配关系）
     * - 分配连线（从地面站到目标位置）
     * - 连线上的文字标签
     */
    void publishMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        // 绘制每个拦截器的地面站标记和文本
        for (const auto& interceptor : interceptors_) {
            geometry_msgs::Point station_position = interceptor.home_position;
            station_position.z = ground_station_marker_altitude_;

            // 地面站立方体
            visualization_msgs::Marker cube;
            cube.header.stamp = stamp;
            cube.header.frame_id = world_frame_;
            cube.ns = "assignment_interceptors";
            cube.id = interceptor.id;
            cube.type = visualization_msgs::Marker::CUBE;
            cube.action = visualization_msgs::Marker::ADD;
            cube.pose.position = station_position;
            cube.pose.orientation.w = 1.0;
            cube.scale.x = interceptor_marker_scale_;
            cube.scale.y = interceptor_marker_scale_;
            cube.scale.z = 0.5 * interceptor_marker_scale_;
            cube.color.a = 1.0;
            cube.color.r = 0.1;
            cube.color.g = interceptor.assigned ? 0.7 : 0.3;  // 已分配更亮
            cube.color.b = 1.0;
            markers.markers.push_back(cube);

            // 拦截器编号标签
            visualization_msgs::Marker text;
            text.header.stamp = stamp;
            text.header.frame_id = world_frame_;
            text.ns = "assignment_interceptor_text";
            text.id = interceptor.id;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position = station_position;
            text.pose.position.z += text_height_;
            text.pose.orientation.w = 1.0;
            text.scale.z = text_height_;
            text.color.a = 1.0;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;

            std::ostringstream label;
            label << "I" << interceptor.id;
            if (interceptor.assigned_target >= 0) {
                label << "->T" << interceptor.assigned_target;
            }
            text.text = label.str();
            markers.markers.push_back(text);
        }

        // 绘制分配连线和连线标签
        int line_id = 0;
        int label_id = 0;
        for (const auto& interceptor : interceptors_) {
            if (!interceptor.assigned || interceptor.assigned_target < 0 ||
                interceptor.assigned_target >= target_count_ ||
                !targets_[interceptor.assigned_target].has_estimate) {
                continue;
            }

            const geometry_msgs::Point target_position =
                targets_[interceptor.assigned_target].estimate.pose.pose.position;
            geometry_msgs::Point station_position = interceptor.home_position;
            station_position.z = ground_station_marker_altitude_;

            // 连线（从地面站到目标位置）
            visualization_msgs::Marker line;
            line.header.stamp = stamp;
            line.header.frame_id = world_frame_;
            line.ns = "assignment_lines";
            line.id = line_id++;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.scale.x = line_width_;
            line.color.a = 0.95;
            line.color.r = 0.2;
            line.color.g = 0.8;
            line.color.b = 1.0;
            line.points.push_back(station_position);
            line.points.push_back(target_position);
            markers.markers.push_back(line);

            // 连线中点处的文字标签
            visualization_msgs::Marker label;
            label.header.stamp = stamp;
            label.header.frame_id = world_frame_;
            label.ns = "assignment_line_text";
            label.id = label_id++;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position.x = 0.5 * (station_position.x + target_position.x);
            label.pose.position.y = 0.5 * (station_position.y + target_position.y);
            label.pose.position.z = 0.5 * (station_position.z + target_position.z) + text_height_;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.8 * text_height_;
            label.color.a = 1.0;
            label.color.r = 1.0;
            label.color.g = 0.9;
            label.color.b = 0.2;

            std::ostringstream text;
            text << "I" << interceptor.id << " -> T" << interceptor.assigned_target;
            label.text = text.str();
            markers.markers.push_back(label);
        }

        marker_pub_.publish(markers);
    }

    // ROS句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    // 参数
    std::string world_frame_;                          // 世界坐标系
    int target_count_ = 3;                             // 目标数量
    int interceptor_count_ = 6;                        // 拦截器数量
    double update_rate_ = 5.0;                         // 分配更新频率（Hz）
    int max_interceptors_per_target_ = 3;              // 每个目标最多分配的拦截器数
    double high_threat_threshold_ = 0.70;              // 高威胁阈值
    double medium_threat_threshold_ = 0.40;            // 中威胁阈值
    int high_threat_interceptors_ = 3;                 // 高威胁目标分配数
    int medium_threat_interceptors_ = 2;               // 中威胁目标分配数
    int low_threat_interceptors_ = 1;                  // 低威胁目标分配数
    double interceptor_speed_ = 30.0;                  // 拦截器速度参考值（米/秒）
    double max_assignment_distance_ = 300.0;           // 最大分配距离（米）
    double stale_timeout_ = 2.0;                       // 目标估计/威胁超时时间（秒）
    double interceptor_odom_timeout_ = 2.0;            // 拦截器里程计超时时间（秒）
    std::string interceptor_prefix_ = "/interceptor_"; // 拦截器话题前缀
    double cost_distance_weight_ = 0.55;               // 代价函数中距离权重
    double cost_time_weight_ = 0.35;                   // 代价函数中时间权重
    double cost_threat_weight_ = 0.45;                 // 代价函数中威胁权重（负项）
    double interceptor_marker_scale_ = 1.0;            // 拦截器标记大小
    double line_width_ = 0.18;                         // 连线宽度
    double text_height_ = 1.2;                         // 文本标签高度
    double ground_station_marker_altitude_ = 0.2;      // 地面站标记高度

    // 数据容器
    std::vector<TargetState> targets_;                  // 所有目标的状态
    std::vector<InterceptorState> interceptors_;        // 所有拦截器的状态

    // ROS通信接口
    std::vector<ros::Subscriber> estimate_subs_;        // 目标估计订阅器列表
    std::vector<ros::Subscriber> threat_subs_;          // 目标威胁订阅器列表
    std::vector<ros::Subscriber> interceptor_odom_subs_; // 拦截器里程计订阅器列表
    ros::Subscriber ranking_sub_;                       // 威胁排名订阅器
    ros::Publisher result_pub_;                         // 分配结果发布器
    ros::Publisher marker_pub_;                         // 可视化标记发布器
    std::string latest_ranking_text_;                   // 最近一次收到的威胁排名文本
};

}  // namespace

/**
 * @brief 主函数：初始化节点，创建协同分配节点实例，进入ROS事件循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "cooperative_assignment_node");
    CooperativeAssignmentNode node;
    ros::spin();
    return 0;
}