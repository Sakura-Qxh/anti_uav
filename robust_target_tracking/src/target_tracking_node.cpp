#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

/**
 * @brief 恒速模型卡尔曼滤波器类
 *
 * 状态向量：[x, y, z, vx, vy, vz]
 * 预测采用恒速运动模型，更新直接使用全状态观测（位置+速度）
 */
class ConstantVelocityKalmanFilter {
public:
    /**
     * @brief 配置滤波器参数
     * @param process_noise_position     位置过程噪声方差系数
     * @param process_noise_velocity     速度过程噪声方差系数
     * @param measurement_noise_position 位置测量噪声标准差
     * @param measurement_noise_velocity 速度测量噪声标准差
     * @param initial_position_variance  初始位置协方差
     * @param initial_velocity_variance  初始速度协方差
     */
    void configure(double process_noise_position,
                   double process_noise_velocity,
                   double measurement_noise_position,
                   double measurement_noise_velocity,
                   double initial_position_variance,
                   double initial_velocity_variance) {
        q_pos_ = process_noise_position;
        q_vel_ = process_noise_velocity;
        r_pos_ = measurement_noise_position;
        r_vel_ = measurement_noise_velocity;
        p_pos0_ = initial_position_variance;
        p_vel0_ = initial_velocity_variance;
    }

    /**
     * @brief 使用首次测量初始化滤波器状态和协方差
     * @param measurement 首次里程计测量
     */
    void initialize(const nav_msgs::Odometry& measurement) {
        x_.setZero();
        x_(0) = measurement.pose.pose.position.x;
        x_(1) = measurement.pose.pose.position.y;
        x_(2) = measurement.pose.pose.position.z;
        x_(3) = measurement.twist.twist.linear.x;
        x_(4) = measurement.twist.twist.linear.y;
        x_(5) = measurement.twist.twist.linear.z;

        P_.setZero();
        P_(0, 0) = p_pos0_;
        P_(1, 1) = p_pos0_;
        P_(2, 2) = p_pos0_;
        P_(3, 3) = p_vel0_;
        P_(4, 4) = p_vel0_;
        P_(5, 5) = p_vel0_;

        initialized_ = true;
    }

    /** @brief 检查滤波器是否已初始化 */
    bool initialized() const {
        return initialized_;
    }

    /**
     * @brief 卡尔曼预测步骤（恒速模型）
     * @param dt 时间间隔（秒），限制在[1ms, 200ms]之间以保证数值稳定性
     */
    void predict(double dt) {
        if (!initialized_) {
            return;
        }

        dt = std::max(1e-3, std::min(dt, 0.2));  // 限幅

        // 状态转移矩阵 F：恒速模型，位置 = 位置 + 速度 * dt
        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;

        // 过程噪声协方差矩阵 Q（近似离散化）
        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        Q(0, 0) = q_pos_ * dt * dt;
        Q(1, 1) = q_pos_ * dt * dt;
        Q(2, 2) = q_pos_ * dt * dt;
        Q(3, 3) = q_vel_ * dt;
        Q(4, 4) = q_vel_ * dt;
        Q(5, 5) = q_vel_ * dt;

        // 预测状态和协方差
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    /**
     * @brief 卡尔曼更新步骤（使用全状态观测）
     * @param measurement 里程计测量（位置+速度）
     *
     * 如果滤波器未初始化，则调用 initialize() 进行初始化
     */
    void update(const nav_msgs::Odometry& measurement) {
        if (!initialized_) {
            initialize(measurement);
            return;
        }

        // 构造观测向量 z
        Eigen::Matrix<double, 6, 1> z;
        z << measurement.pose.pose.position.x,
             measurement.pose.pose.position.y,
             measurement.pose.pose.position.z,
             measurement.twist.twist.linear.x,
             measurement.twist.twist.linear.y,
             measurement.twist.twist.linear.z;

        // 观测矩阵 H（单位阵，因为状态直接可测）
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
        // 测量噪声协方差矩阵 R（对角阵，标准差平方）
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
        R(0, 0) = r_pos_ * r_pos_;
        R(1, 1) = r_pos_ * r_pos_;
        R(2, 2) = r_pos_ * r_pos_;
        R(3, 3) = r_vel_ * r_vel_;
        R(4, 4) = r_vel_ * r_vel_;
        R(5, 5) = r_vel_ * r_vel_;

        // 创新（残差）
        const Eigen::Matrix<double, 6, 1> y = z - H * x_;
        // 创新协方差
        const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
        // 卡尔曼增益
        const Eigen::Matrix<double, 6, 6> K = P_ * H.transpose() * S.inverse();

        // 更新状态和协方差
        x_ = x_ + K * y;
        const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        P_ = (I - K * H) * P_;
    }

    /**
     * @brief 计算当前状态估计位置与给定测量位置之间的欧氏距离
     * @param measurement 里程计测量
     * @return 距离（米），若未初始化则返回0
     */
    double positionDistanceTo(const nav_msgs::Odometry& measurement) const {
        if (!initialized_) {
            return 0.0;
        }
        const double dx = measurement.pose.pose.position.x - x_(0);
        const double dy = measurement.pose.pose.position.y - x_(1);
        const double dz = measurement.pose.pose.position.z - x_(2);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    /** @brief 获取当前状态向量（常引用） */
    const Eigen::Matrix<double, 6, 1>& state() const {
        return x_;
    }

private:
    bool initialized_ = false;                      // 是否已初始化
    Eigen::Matrix<double, 6, 1> x_ = Eigen::Matrix<double, 6, 1>::Zero();   // 状态向量
    Eigen::Matrix<double, 6, 6> P_ = Eigen::Matrix<double, 6, 6>::Identity(); // 协方差矩阵
    double q_pos_ = 0.05;   // 位置过程噪声系数
    double q_vel_ = 0.5;    // 速度过程噪声系数
    double r_pos_ = 0.5;    // 位置测量噪声标准差
    double r_vel_ = 0.2;    // 速度测量噪声标准差
    double p_pos0_ = 2.0;   // 初始位置协方差
    double p_vel0_ = 5.0;   // 初始速度协方差
};

/**
 * @brief 单个目标的跟踪数据结构
 *
 * 包含卡尔曼滤波器、时间戳、路径记录等
 */
struct TargetTrack {
    ConstantVelocityKalmanFilter filter;  // 卡尔曼滤波器实例
    ros::Time last_update_time;           // 上一次预测/更新的时间
    ros::Time last_measurement_time;      // 上一次收到有效测量的时间
    bool has_measurement = false;         // 是否至少收到过一次测量
    bool last_update_rejected = false;    // 最近一次更新是否因门控被拒绝
    nav_msgs::Path estimate_path;         // 历史估计轨迹（用于可视化）
};

/**
 * @brief 鲁棒多目标跟踪节点类
 *
 * 订阅每个目标的雷达测量话题，使用恒速卡尔曼滤波器进行跟踪，
 * 并利用马氏距离门控剔除异常值。定期发布估计状态、预测轨迹和历史路径。
 */
class TargetTrackingNode {
public:
    TargetTrackingNode() : nh_(), pnh_("~") {
        // 加载参数
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<double>("update_rate", update_rate_, 50.0);                     // 定时器更新频率
        pnh_.param<double>("process_noise_position", process_noise_position_, 0.05);
        pnh_.param<double>("process_noise_velocity", process_noise_velocity_, 0.5);
        pnh_.param<double>("measurement_noise_position", measurement_noise_position_, 0.5);
        pnh_.param<double>("measurement_noise_velocity", measurement_noise_velocity_, 0.2);
        pnh_.param<double>("initial_position_variance", initial_position_variance_, 2.0);
        pnh_.param<double>("initial_velocity_variance", initial_velocity_variance_, 5.0);
        pnh_.param<double>("gating_distance", gating_distance_, 8.0);              // 门控阈值（米）
        pnh_.param<double>("max_prediction_gap", max_prediction_gap_, 2.0);        // 最大允许无测量时间（秒）
        pnh_.param<double>("prediction_horizon", prediction_horizon_, 2.0);        // 预测时长（秒）
        pnh_.param<double>("prediction_dt", prediction_dt_, 0.2);                  // 预测步长（秒）
        pnh_.param<int>("path_max_points", path_max_points_, 300);                 // 轨迹最大点数
        pnh_.param<double>("estimate_marker_scale", estimate_marker_scale_, 0.55); // 估计位置标记大小
        pnh_.param<double>("prediction_marker_scale", prediction_marker_scale_, 0.08); // 预测线宽

        // 初始化每个目标的滤波器参数
        tracks_.resize(target_count_);
        for (auto& track : tracks_) {
            track.filter.configure(process_noise_position_,
                                   process_noise_velocity_,
                                   measurement_noise_position_,
                                   measurement_noise_velocity_,
                                   initial_position_variance_,
                                   initial_velocity_variance_);
        }

        // 创建ROS接口（订阅器、发布器、定时器）
        createRosInterfaces();
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_),
                                &TargetTrackingNode::timerCallback,
                                this);

        ROS_INFO("robust_target_tracking started for %d targets", target_count_);
    }

private:
    /**
     * @brief 为每个目标创建测量订阅器、估计/预测/路径发布器
     *
     * 话题约定：
     * - 测量输入：/radar/target_{i}/measurement
     * - 估计输出：/target_{i}/estimate
     * - 预测输出：/target_{i}/prediction
     * - 轨迹输出：/target_{i}/estimate_path
     */
    void createRosInterfaces() {
        measurement_subs_.clear();
        estimate_pubs_.clear();
        prediction_pubs_.clear();
        estimate_path_pubs_.clear();

        for (int i = 0; i < target_count_; ++i) {
            const std::string measurement_topic = "/radar/target_" + std::to_string(i) + "/measurement";
            const std::string estimate_topic = "/target_" + std::to_string(i) + "/estimate";
            const std::string prediction_topic = "/target_" + std::to_string(i) + "/prediction";
            const std::string estimate_path_topic = "/target_" + std::to_string(i) + "/estimate_path";

            // 订阅测量话题，回调绑定目标索引
            measurement_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                measurement_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    measurementCallback(msg, i);
                }));

            // 发布估计、预测、路径
            estimate_pubs_.push_back(nh_.advertise<nav_msgs::Odometry>(estimate_topic, 10));
            prediction_pubs_.push_back(nh_.advertise<nav_msgs::Path>(prediction_topic, 1));
            estimate_path_pubs_.push_back(nh_.advertise<nav_msgs::Path>(estimate_path_topic, 1));
        }

        // 发布可视化标记（估计位置方块和预测线）
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/target_tracking/markers", 1);
    }

    /**
     * @brief 测量回调：对每个目标执行卡尔曼预测+更新，并使用门控剔除异常值
     * @param msg 里程计测量消息
     * @param target_index 目标索引
     */
    void measurementCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= static_cast<int>(tracks_.size())) {
            return;
        }

        TargetTrack& track = tracks_[target_index];
        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

        // 首次测量：直接初始化滤波器
        if (!track.filter.initialized()) {
            track.filter.initialize(*msg);
            track.last_update_time = stamp;
            track.last_measurement_time = stamp;
            track.has_measurement = true;
            track.last_update_rejected = false;
            return;
        }

        // 非首次：先根据时间差做预测
        const double dt = (stamp - track.last_update_time).toSec();
        if (dt > 0.0) {
            track.filter.predict(dt);
            track.last_update_time = stamp;
        }

        // 计算创新距离（预测位置与测量位置的欧氏距离），若超过门控则拒绝该测量
        const double innovation_distance = track.filter.positionDistanceTo(*msg);
        if (innovation_distance > gating_distance_) {
            track.last_update_rejected = true;
            return;  // 丢弃该测量
        }

        // 通过门控，执行更新
        track.filter.update(*msg);
        track.last_update_time = stamp;
        track.last_measurement_time = stamp;
        track.has_measurement = true;
        track.last_update_rejected = false;
    }

    /**
     * @brief 定时器回调：对所有已初始化的目标执行预测，并发布估计、预测和标记
     *
     * 如果距离上次测量时间超过 max_prediction_gap_，则停止发布（认为跟踪丢失）
     */
    void timerCallback(const ros::TimerEvent& event) {
        const ros::Time stamp = ros::Time::now();

        for (int i = 0; i < target_count_; ++i) {
            TargetTrack& track = tracks_[i];
            if (!track.filter.initialized()) {
                continue;
            }

            // 预测到当前时刻
            const double dt = track.last_update_time.isZero()
                                  ? 1.0 / update_rate_
                                  : (stamp - track.last_update_time).toSec();
            if (dt > 0.0) {
                track.filter.predict(dt);
                track.last_update_time = stamp;
            }

            // 检查无测量时间是否超限
            const double gap = track.last_measurement_time.isZero()
                                   ? 0.0
                                   : (stamp - track.last_measurement_time).toSec();
            if (gap <= max_prediction_gap_) {
                publishEstimate(i, stamp);
                publishPrediction(i, stamp);
            }
            // 若超限则不发布任何内容（相当于跟踪丢失）
        }

        publishMarkers(stamp);
    }

    /**
     * @brief 发布单个目标的估计状态（里程计消息）
     * @param target_index 目标索引
     * @param stamp 时间戳
     */
    void publishEstimate(int target_index, const ros::Time& stamp) {
        const auto& state = tracks_[target_index].filter.state();

        nav_msgs::Odometry estimate;
        estimate.header.stamp = stamp;
        estimate.header.frame_id = world_frame_;
        estimate.child_frame_id = "target_" + std::to_string(target_index) + "_estimate";
        estimate.pose.pose.position.x = state(0);
        estimate.pose.pose.position.y = state(1);
        estimate.pose.pose.position.z = state(2);
        estimate.pose.pose.orientation.w = 1.0;
        estimate.twist.twist.linear.x = state(3);
        estimate.twist.twist.linear.y = state(4);
        estimate.twist.twist.linear.z = state(5);

        estimate_pubs_[target_index].publish(estimate);
        appendEstimatePath(target_index, estimate);  // 添加到轨迹
    }

    /**
     * @brief 将估计位姿追加到轨迹路径中，并发布（限制最大点数）
     */
    void appendEstimatePath(int target_index, const nav_msgs::Odometry& estimate) {
        geometry_msgs::PoseStamped pose;
        pose.header = estimate.header;
        pose.pose = estimate.pose.pose;

        nav_msgs::Path& path = tracks_[target_index].estimate_path;
        path.header.stamp = estimate.header.stamp;
        path.header.frame_id = world_frame_;
        path.poses.push_back(pose);

        // 超出最大点数时移除最早的点
        if (static_cast<int>(path.poses.size()) > path_max_points_) {
            path.poses.erase(path.poses.begin());
        }

        estimate_path_pubs_[target_index].publish(path);
    }

    /**
     * @brief 发布单个目标的预测轨迹（基于恒速模型外推）
     * @param target_index 目标索引
     * @param stamp 当前时间戳
     */
    void publishPrediction(int target_index, const ros::Time& stamp) {
        const auto& state = tracks_[target_index].filter.state();

        nav_msgs::Path prediction;
        prediction.header.stamp = stamp;
        prediction.header.frame_id = world_frame_;

        // 从 prediction_dt_ 到 prediction_horizon_ 等间隔采样
        for (double t = prediction_dt_; t <= prediction_horizon_ + 1e-6; t += prediction_dt_) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = stamp + ros::Duration(t);
            pose.header.frame_id = world_frame_;
            pose.pose.position.x = state(0) + state(3) * t;
            pose.pose.position.y = state(1) + state(4) * t;
            pose.pose.position.z = state(2) + state(5) * t;
            pose.pose.orientation.w = 1.0;
            prediction.poses.push_back(pose);
        }

        prediction_pubs_[target_index].publish(prediction);
    }

    /**
     * @brief 发布所有目标的RViz可视化标记
     *
     * - 估计位置：立方体（颜色根据最近一次更新是否被拒绝而变化）
     * - 预测轨迹：线条（青色）
     */
    void publishMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            if (!tracks_[i].filter.initialized()) {
                continue;
            }

            const auto& state = tracks_[i].filter.state();

            // 估计位置标记（立方体）
            visualization_msgs::Marker estimate_marker;
            estimate_marker.header.stamp = stamp;
            estimate_marker.header.frame_id = world_frame_;
            estimate_marker.ns = "target_estimates";
            estimate_marker.id = i;
            estimate_marker.type = visualization_msgs::Marker::CUBE;
            estimate_marker.action = visualization_msgs::Marker::ADD;
            estimate_marker.pose.position.x = state(0);
            estimate_marker.pose.position.y = state(1);
            estimate_marker.pose.position.z = state(2);
            estimate_marker.pose.orientation.w = 1.0;
            estimate_marker.scale.x = estimate_marker_scale_;
            estimate_marker.scale.y = estimate_marker_scale_;
            estimate_marker.scale.z = estimate_marker_scale_;
            estimate_marker.color.a = 1.0;
            // 若最近一次更新被拒绝，标记为红色；否则为青色
            estimate_marker.color.r = tracks_[i].last_update_rejected ? 1.0 : 0.1;
            estimate_marker.color.g = tracks_[i].last_update_rejected ? 0.2 : 0.8;
            estimate_marker.color.b = 1.0;
            markers.markers.push_back(estimate_marker);

            // 预测轨迹标记（线条）
            visualization_msgs::Marker prediction_marker;
            prediction_marker.header.stamp = stamp;
            prediction_marker.header.frame_id = world_frame_;
            prediction_marker.ns = "target_predictions";
            prediction_marker.id = i;
            prediction_marker.type = visualization_msgs::Marker::LINE_STRIP;
            prediction_marker.action = visualization_msgs::Marker::ADD;
            prediction_marker.scale.x = prediction_marker_scale_;  // 线宽
            prediction_marker.color.a = 0.9;
            prediction_marker.color.r = 0.2;
            prediction_marker.color.g = 0.9;
            prediction_marker.color.b = 1.0;

            // 填充预测线上的点
            for (double t = 0.0; t <= prediction_horizon_ + 1e-6; t += prediction_dt_) {
                geometry_msgs::Point point;
                point.x = state(0) + state(3) * t;
                point.y = state(1) + state(4) * t;
                point.z = state(2) + state(5) * t;
                prediction_marker.points.push_back(point);
            }
            markers.markers.push_back(prediction_marker);
        }

        marker_pub_.publish(markers);
    }

    // ROS句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    // 参数
    std::string world_frame_;              // 世界坐标系
    int target_count_ = 3;                 // 目标数量
    double update_rate_ = 50.0;            // 定时器更新频率（Hz）
    double process_noise_position_ = 0.05; // 位置过程噪声系数
    double process_noise_velocity_ = 0.5;  // 速度过程噪声系数
    double measurement_noise_position_ = 0.5;  // 位置测量噪声标准差
    double measurement_noise_velocity_ = 0.2;  // 速度测量噪声标准差
    double initial_position_variance_ = 2.0;   // 初始位置协方差
    double initial_velocity_variance_ = 5.0;   // 初始速度协方差
    double gating_distance_ = 8.0;         // 门控阈值（米）
    double max_prediction_gap_ = 2.0;      // 最大允许无测量时间（秒）
    double prediction_horizon_ = 2.0;      // 预测时长（秒）
    double prediction_dt_ = 0.2;           // 预测步长（秒）
    int path_max_points_ = 300;            // 轨迹最大点数
    double estimate_marker_scale_ = 0.55;  // 估计位置标记大小
    double prediction_marker_scale_ = 0.08; // 预测线宽

    // 跟踪数据
    std::vector<TargetTrack> tracks_;

    // ROS通信接口
    std::vector<ros::Subscriber> measurement_subs_;      // 测量订阅器
    std::vector<ros::Publisher> estimate_pubs_;           // 估计发布器
    std::vector<ros::Publisher> prediction_pubs_;         // 预测发布器
    std::vector<ros::Publisher> estimate_path_pubs_;      // 估计轨迹发布器
    ros::Publisher marker_pub_;                           // 可视化标记发布器
};

}  // namespace

/**
 * @brief 主函数：初始化节点，创建目标跟踪节点实例，进入ROS事件循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "target_tracking_node");
    TargetTrackingNode node;
    ros::spin();
    return 0;
}