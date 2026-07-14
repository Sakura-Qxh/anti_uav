#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

/**
 * @brief 将数值钳制到 [0, 1] 区间
 */
double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

/**
 * @brief 计算三维向量的欧几里得范数
 */
double norm3(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
}

/**
 * @brief 单个目标的威胁状态结构体
 *
 * 存储目标的估计状态、预测轨迹、时间戳以及各项威胁子分数
 */
struct TargetThreatState {
    nav_msgs::Odometry estimate;           // 目标当前估计（位置+速度）
    nav_msgs::Path prediction;             // 目标预测轨迹
    ros::Time last_estimate_time;          // 最后一次收到估计的时间
    bool has_estimate = false;             // 是否已收到估计
    bool has_prediction = false;           // 是否已收到预测
    double threat = 0.0;                   // 综合威胁分数 [0,1]
    double distance_score = 0.0;           // 距离子分数
    double speed_score = 0.0;              // 速度子分数
    double approach_score = 0.0;           // 接近速率子分数
    double altitude_score = 0.0;           // 高度子分数
    double time_to_go_score = 0.0;         // 预计到达时间子分数
};

/**
 * @brief 排序后的目标排名结构体
 */
struct RankedTarget {
    int id = 0;           // 目标编号
    double threat = 0.0;  // 威胁分数
};

/**
 * @brief 威胁评估节点类
 *
 * 订阅多个目标的估计和预测话题，综合多项指标计算每个目标的威胁分数，
 * 并按威胁降序排列，发布威胁值、排名文本以及可视化标记。
 */
class ThreatAssessmentNode {
public:
    ThreatAssessmentNode() : nh_(), pnh_("~") {
        // 加载参数
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<double>("update_rate", update_rate_, 10.0);                 // 评估更新频率（Hz）
        pnh_.param<double>("protected_radius", protected_radius_, 5.0);        // 受保护区域半径（米）
        pnh_.param<double>("max_assessment_distance", max_assessment_distance_, 120.0); // 最大评估距离
        pnh_.param<double>("max_target_speed", max_target_speed_, 45.0);       // 目标最大速度（归一化参考）
        pnh_.param<double>("max_approach_speed", max_approach_speed_, 45.0);   // 最大接近速率
        pnh_.param<double>("safe_altitude", safe_altitude_, 25.0);             // 安全高度（米）
        pnh_.param<double>("critical_altitude", critical_altitude_, 3.0);      // 临界高度（米）
        pnh_.param<double>("max_time_to_go", max_time_to_go_, 20.0);           // 最大预计到达时间（秒）
        pnh_.param<double>("weight_distance", weight_distance_, 0.25);         // 距离权重
        pnh_.param<double>("weight_speed", weight_speed_, 0.20);               // 速度权重
        pnh_.param<double>("weight_approach", weight_approach_, 0.25);         // 接近速率权重
        pnh_.param<double>("weight_altitude", weight_altitude_, 0.15);         // 高度权重
        pnh_.param<double>("weight_time_to_go", weight_time_to_go_, 0.15);     // 预计到达时间权重
        pnh_.param<double>("stale_timeout", stale_timeout_, 2.0);              // 估计超时时间（秒）
        pnh_.param<double>("text_height", text_height_, 1.2);                  // 标签文本高度
        pnh_.param<double>("marker_scale", marker_scale_, 0.8);                // 威胁球体基准尺寸

        // 读取受保护区域中心点（三维坐标），默认为原点
        std::vector<double> protected_center;
        if (pnh_.getParam("protected_center", protected_center) && protected_center.size() >= 3) {
            protected_center_.x = protected_center[0];
            protected_center_.y = protected_center[1];
            protected_center_.z = protected_center[2];
        } else {
            protected_center_.x = 0.0;
            protected_center_.y = 0.0;
            protected_center_.z = 0.0;
        }

        // 归一化权重（确保总和为1）
        normalizeWeights();

        // 初始化目标状态容器
        targets_.resize(target_count_);
        // 创建ROS通信接口
        createRosInterfaces();

        // 创建定时器，按指定频率执行威胁评估
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_),
                                &ThreatAssessmentNode::timerCallback,
                                this);

        ROS_INFO("threat_assessment started for %d targets", target_count_);
    }

private:
    /**
     * @brief 归一化权重，使五个权重之和为1
     */
    void normalizeWeights() {
        const double sum = weight_distance_ + weight_speed_ + weight_approach_ +
                           weight_altitude_ + weight_time_to_go_;
        if (sum <= 1e-6) {  // 防止除零，恢复默认值
            weight_distance_ = 0.25;
            weight_speed_ = 0.20;
            weight_approach_ = 0.25;
            weight_altitude_ = 0.15;
            weight_time_to_go_ = 0.15;
            return;
        }

        weight_distance_ /= sum;
        weight_speed_ /= sum;
        weight_approach_ /= sum;
        weight_altitude_ /= sum;
        weight_time_to_go_ /= sum;
    }

    /**
     * @brief 为每个目标创建估计订阅器、预测订阅器和威胁发布器
     *
     * 话题约定：
     * - 估计输入：/target_{i}/estimate
     * - 预测输入：/target_{i}/prediction
     * - 威胁输出：/threat/target_{i}
     */
    void createRosInterfaces() {
        estimate_subs_.clear();
        prediction_subs_.clear();
        threat_pubs_.clear();

        for (int i = 0; i < target_count_; ++i) {
            const std::string estimate_topic = "/target_" + std::to_string(i) + "/estimate";
            const std::string prediction_topic = "/target_" + std::to_string(i) + "/prediction";
            const std::string threat_topic = "/threat/target_" + std::to_string(i);

            // 订阅估计话题，回调绑定目标索引
            estimate_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                estimate_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    estimateCallback(msg, i);
                }));

            // 订阅预测话题，回调绑定目标索引
            prediction_subs_.push_back(nh_.subscribe<nav_msgs::Path>(
                prediction_topic, 10, [this, i](const nav_msgs::Path::ConstPtr& msg) {
                    predictionCallback(msg, i);
                }));

            // 创建威胁值发布器
            threat_pubs_.push_back(nh_.advertise<std_msgs::Float64>(threat_topic, 10));
        }

        // 创建威胁排名文本发布器
        ranking_pub_ = nh_.advertise<std_msgs::String>("/threat/ranking", 10);
        // 创建可视化标记发布器
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/threat/markers", 1);
    }

    /**
     * @brief 估计回调：更新目标的估计状态和时间戳
     * @param msg 里程计估计消息
     * @param target_index 目标索引
     */
    void estimateCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= static_cast<int>(targets_.size())) {
            return;
        }

        targets_[target_index].estimate = *msg;
        targets_[target_index].last_estimate_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        targets_[target_index].has_estimate = true;
    }

    /**
     * @brief 预测回调：更新目标的预测轨迹
     * @param msg 路径消息（预测轨迹）
     * @param target_index 目标索引
     */
    void predictionCallback(const nav_msgs::Path::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= static_cast<int>(targets_.size())) {
            return;
        }

        targets_[target_index].prediction = *msg;
        targets_[target_index].has_prediction = true;
    }

    /**
     * @brief 定时器回调：对所有目标执行威胁评估、排序、发布
     */
    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        std::vector<RankedTarget> ranking;

        for (int i = 0; i < target_count_; ++i) {
            TargetThreatState& target = targets_[i];
            // 检查目标估计是否超时，若超时则威胁置零
            if (!isTargetFresh(target, now)) {
                target.threat = 0.0;
                publishThreat(i, target.threat);
                continue;
            }

            // 计算综合威胁分数
            computeThreat(target);
            publishThreat(i, target.threat);
            ranking.push_back({i, target.threat});
        }

        // 按威胁分数降序排列
        std::sort(ranking.begin(), ranking.end(), [](const RankedTarget& a, const RankedTarget& b) {
            return a.threat > b.threat;
        });

        // 发布排名文本和可视化标记
        publishRanking(ranking);
        publishMarkers(now, ranking);
    }

    /**
     * @brief 检查目标估计是否在有效时间内
     * @param target 目标状态
     * @param now 当前时间
     * @return true 表示有效，false 表示已过期
     */
    bool isTargetFresh(const TargetThreatState& target, const ros::Time& now) const {
        if (!target.has_estimate) {
            return false;
        }
        return (now - target.last_estimate_time).toSec() <= stale_timeout_;
    }

    /**
     * @brief 计算单个目标的综合威胁分数（加权融合五项子分数）
     * @param target 目标状态（输出：各项子分数和总威胁）
     *
     * 子分数说明：
     * - distance_score：距保护区距离评分（越近越高）
     * - speed_score：速度评分（越快越高）
     * - approach_score：接近速率评分（径向速度朝向保护区越高）
     * - altitude_score：高度评分（越低或下降越快越高）
     * - time_to_go_score：预计到达时间评分（越短越高）
     */
    void computeThreat(TargetThreatState& target) {
        const auto& pos = target.estimate.pose.pose.position;
        const auto& vel = target.estimate.twist.twist.linear;

        // 目标相对于保护区中心的矢量
        const double rx = pos.x - protected_center_.x;
        const double ry = pos.y - protected_center_.y;
        const double rz = pos.z - protected_center_.z;
        const double distance_to_center = norm3(rx, ry, rz);
        const double distance_to_zone = std::max(0.0, distance_to_center - protected_radius_); // 到保护区边界的距离

        // 目标合速度
        const double speed = norm3(vel.x, vel.y, vel.z);

        // 径向接近速率（负值表示靠近）
        double approach_speed = 0.0;
        if (distance_to_center > 1e-6) {
            const double radial_velocity = (rx * vel.x + ry * vel.y + rz * vel.z) / distance_to_center;
            approach_speed = std::max(0.0, -radial_velocity);  // 取正值表示接近速度
        }

        // 预计到达保护区边界的时间
        const double time_to_go = approach_speed > 1e-3
                                      ? distance_to_zone / approach_speed
                                      : max_time_to_go_;

        // 当前高度和下降速率
        const double altitude = pos.z;
        const double descending_speed = std::max(0.0, -vel.z);

        // ---- 计算各子分数 ----
        // 距离分数：距离保护区边界越近，分数越高
        target.distance_score = clamp01(1.0 - distance_to_zone / max_assessment_distance_);
        // 速度分数：速度越大，分数越高
        target.speed_score = clamp01(speed / max_target_speed_);
        // 接近速率分数：径向接近速度越大，分数越高
        target.approach_score = clamp01(approach_speed / max_approach_speed_);

        // 高度分数：由低高度得分和下降速率得分组合（75%低高度 + 25%下降速率）
        const double low_altitude_score = clamp01((safe_altitude_ - altitude) /
                                                  std::max(1e-6, safe_altitude_ - critical_altitude_));
        const double descending_score = clamp01(descending_speed / std::max(1.0, max_target_speed_));
        target.altitude_score = clamp01(0.75 * low_altitude_score + 0.25 * descending_score);

        // 预计到达时间分数：时间越短，分数越高
        target.time_to_go_score = clamp01(1.0 - time_to_go / max_time_to_go_);

        // 综合威胁分数 = 加权和，并钳制到 [0,1]
        target.threat = clamp01(weight_distance_ * target.distance_score +
                                weight_speed_ * target.speed_score +
                                weight_approach_ * target.approach_score +
                                weight_altitude_ * target.altitude_score +
                                weight_time_to_go_ * target.time_to_go_score);
    }

    /**
     * @brief 发布单个目标的威胁值
     * @param target_index 目标索引
     * @param threat 威胁分数
     */
    void publishThreat(int target_index, double threat) {
        std_msgs::Float64 msg;
        msg.data = threat;
        threat_pubs_[target_index].publish(msg);
    }

    /**
     * @brief 发布威胁排名文本（格式：target_i(threat) > target_j(threat) > ...）
     * @param ranking 已排序的排名列表
     */
    void publishRanking(const std::vector<RankedTarget>& ranking) {
        std_msgs::String msg;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        for (size_t i = 0; i < ranking.size(); ++i) {
            if (i > 0) {
                oss << " > ";
            }
            oss << "target_" << ranking[i].id << "(" << ranking[i].threat << ")";
        }
        msg.data = oss.str();
        ranking_pub_.publish(msg);
    }

    /**
     * @brief 发布RViz可视化标记
     *
     * 包括：
     * - 威胁球体（颜色红绿渐变，大小随威胁变化）
     * - 目标标签（显示威胁值）
     * - 排名标签（显示第几名）
     * @param stamp 时间戳
     * @param ranking 已排序的排名列表
     */
    void publishMarkers(const ros::Time& stamp, const std::vector<RankedTarget>& ranking) {
        visualization_msgs::MarkerArray markers;

        // 为每个目标绘制威胁球体和文本标签
        for (int i = 0; i < target_count_; ++i) {
            const TargetThreatState& target = targets_[i];
            if (!target.has_estimate) {
                continue;
            }

            const auto& pos = target.estimate.pose.pose.position;
            const double threat = target.threat;

            // 威胁球体（大小和颜色反映威胁程度）
            visualization_msgs::Marker sphere;
            sphere.header.stamp = stamp;
            sphere.header.frame_id = world_frame_;
            sphere.ns = "threat_score_spheres";
            sphere.id = i;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position = pos;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = marker_scale_ * (0.7 + threat);  // 威胁越大球越大
            sphere.scale.y = marker_scale_ * (0.7 + threat);
            sphere.scale.z = marker_scale_ * (0.7 + threat);
            sphere.color.a = 0.85;
            sphere.color.r = threat;          // 红色分量随威胁增加
            sphere.color.g = 1.0 - threat;    // 绿色分量随威胁减少
            sphere.color.b = 0.15;
            markers.markers.push_back(sphere);

            // 目标编号和威胁值文本（始终面向相机）
            visualization_msgs::Marker text;
            text.header.stamp = stamp;
            text.header.frame_id = world_frame_;
            text.ns = "threat_score_text";
            text.id = i;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position.x = pos.x;
            text.pose.position.y = pos.y;
            text.pose.position.z = pos.z + text_height_;
            text.pose.orientation.w = 1.0;
            text.scale.z = text_height_;
            text.color.a = 1.0;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;

            std::ostringstream label;
            label << std::fixed << std::setprecision(2)
                  << "T" << i << " threat=" << threat;
            text.text = label.str();
            markers.markers.push_back(text);
        }

        // 为排名靠前的目标添加排名标签（显示 "rank 1", "rank 2" 等）
        for (size_t rank = 0; rank < ranking.size(); ++rank) {
            const int target_id = ranking[rank].id;
            if (target_id < 0 || target_id >= target_count_ || !targets_[target_id].has_estimate) {
                continue;
            }

            const auto& pos = targets_[target_id].estimate.pose.pose.position;
            visualization_msgs::Marker rank_text;
            rank_text.header.stamp = stamp;
            rank_text.header.frame_id = world_frame_;
            rank_text.ns = "threat_rank_text";
            rank_text.id = static_cast<int>(rank);
            rank_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            rank_text.action = visualization_msgs::Marker::ADD;
            rank_text.pose.position.x = pos.x;
            rank_text.pose.position.y = pos.y;
            rank_text.pose.position.z = pos.z + 2.0 * text_height_;  // 放在威胁标签上方
            rank_text.pose.orientation.w = 1.0;
            rank_text.scale.z = 0.9 * text_height_;
            rank_text.color.a = 1.0;
            rank_text.color.r = 1.0;
            rank_text.color.g = 0.8;
            rank_text.color.b = 0.1;

            std::ostringstream label;
            label << "rank " << (rank + 1);
            rank_text.text = label.str();
            markers.markers.push_back(rank_text);
        }

        marker_pub_.publish(markers);
    }

    // ROS句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    // 参数
    std::string world_frame_;                    // 世界坐标系
    int target_count_ = 3;                       // 目标数量
    double update_rate_ = 10.0;                  // 评估更新频率（Hz）
    geometry_msgs::Point protected_center_;      // 受保护区域中心点
    double protected_radius_ = 5.0;              // 受保护区域半径
    double max_assessment_distance_ = 120.0;     // 最大评估距离（米）
    double max_target_speed_ = 45.0;             // 目标最大速度参考值（米/秒）
    double max_approach_speed_ = 45.0;           // 最大接近速率参考值（米/秒）
    double safe_altitude_ = 25.0;                // 安全高度（米）
    double critical_altitude_ = 3.0;             // 临界高度（米）
    double max_time_to_go_ = 20.0;               // 最大预计到达时间（秒）
    double weight_distance_ = 0.25;              // 距离权重
    double weight_speed_ = 0.20;                 // 速度权重
    double weight_approach_ = 0.25;              // 接近速率权重
    double weight_altitude_ = 0.15;              // 高度权重
    double weight_time_to_go_ = 0.15;            // 预计到达时间权重
    double stale_timeout_ = 2.0;                 // 估计超时时间（秒）
    double text_height_ = 1.2;                   // 文本标签高度（米）
    double marker_scale_ = 0.8;                  // 威胁球体基准尺寸

    // 目标状态数组
    std::vector<TargetThreatState> targets_;

    // ROS通信接口
    std::vector<ros::Subscriber> estimate_subs_;      // 估计订阅器列表
    std::vector<ros::Subscriber> prediction_subs_;    // 预测订阅器列表
    std::vector<ros::Publisher> threat_pubs_;          // 威胁值发布器列表
    ros::Publisher ranking_pub_;                       // 排名文本发布器
    ros::Publisher marker_pub_;                        // 可视化标记发布器
};

}  // namespace

/**
 * @brief 主函数：初始化节点，创建威胁评估节点实例，进入ROS事件循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "threat_assessment_node");
    ThreatAssessmentNode node;
    ros::spin();
    return 0;
}