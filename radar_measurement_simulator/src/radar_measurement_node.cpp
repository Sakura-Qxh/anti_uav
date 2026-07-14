#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// 匿名命名空间：内部辅助类型和实现细节，对外不可见
namespace {

/**
 * @brief 单个目标的测量状态结构体
 *
 * 存储目标的最新真值、是否有效、是否处于突发丢失状态及结束时间
 */
struct TargetMeasurementState {
    nav_msgs::Odometry latest_truth;  // 最近一次接收到的真值里程计消息
    bool has_truth = false;           // 是否已经接收到真值
    bool in_burst_loss = false;       // 当前是否处于突发丢失状态
    ros::Time burst_loss_end;         // 突发丢失结束的时间点
};

/**
 * @brief 雷达测量模拟节点类
 *
 * 订阅每个目标的真值话题，模拟雷达测量过程：
 * - 添加高斯位置/速度噪声
 * - 随机丢包（一定概率不发布测量）
 * - 异常值（大幅跳跃）
 * - 突发丢失（连续一段时间内不发布任何测量）
 * 最后发布模拟的测量结果和可视化标记
 */
class RadarMeasurementNode {
public:
    /**
     * @brief 构造函数：加载参数，初始化随机数生成器，创建订阅器和发布器
     */
    RadarMeasurementNode() : nh_(), pnh_("~"), rng_(std::random_device{}()) {
        // 从参数服务器加载配置参数（带默认值）
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<double>("position_noise_std", position_noise_std_, 0.5);       // 位置噪声标准差（米）
        pnh_.param<double>("velocity_noise_std", velocity_noise_std_, 0.2);       // 速度噪声标准差（米/秒）
        pnh_.param<double>("random_drop_prob", random_drop_prob_, 0.1);           // 随机丢包概率
        pnh_.param<double>("outlier_prob", outlier_prob_, 0.03);                  // 异常值出现概率
        pnh_.param<double>("outlier_position_jump", outlier_position_jump_, 8.0); // 异常值跳跃幅度（米）
        pnh_.param<bool>("enable_burst_loss", enable_burst_loss_, true);          // 是否启用突发丢失
        pnh_.param<double>("burst_loss_prob", burst_loss_prob_, 0.02);            // 每次真值到达时触发突发丢失的概率
        pnh_.param<double>("burst_loss_duration", burst_loss_duration_, 1.0);     // 突发丢失持续时间（秒）
        pnh_.param<double>("marker_scale", marker_scale_, 0.35);                  // 可视化标记球体大小

        // 根据目标数量初始化状态容器
        targets_.resize(target_count_);
        latest_measurements_.resize(target_count_);
        latest_valid_.assign(target_count_, false);

        // 创建每个目标的订阅器和发布器
        createSubscribersAndPublishers();

        // 创建可视化标记发布器（用于在RViz中显示测量点）
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar/measurements/markers", 1);

        ROS_INFO("radar_measurement_simulator started for %d targets", target_count_);
    }

private:
    /**
     * @brief 为每个目标创建真值订阅器和测量发布器
     *
     * 话题格式：
     * - 真值: /target_{i}/truth
     * - 测量: /radar/target_{i}/measurement
     */
    void createSubscribersAndPublishers() {
        truth_subs_.clear();
        measurement_pubs_.clear();

        for (int i = 0; i < target_count_; ++i) {
            const std::string truth_topic = "/target_" + std::to_string(i) + "/truth";
            const std::string measurement_topic = "/radar/target_" + std::to_string(i) + "/measurement";

            // 订阅真值，回调中绑定目标索引
            truth_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                truth_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    truthCallback(msg, i);
                }));

            // 创建测量发布器
            measurement_pubs_.push_back(nh_.advertise<nav_msgs::Odometry>(measurement_topic, 10));
        }
    }

    /**
     * @brief 真值回调函数：处理每个目标的真值数据，决定是否发布模拟测量
     * @param msg 接收到的真值里程计消息
     * @param target_index 目标索引（0 ~ target_count_-1）
     */
    void truthCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        // 索引越界检查
        if (target_index < 0 || target_index >= static_cast<int>(targets_.size())) {
            return;
        }

        // 更新目标状态中的真值
        auto& target = targets_[target_index];
        target.latest_truth = *msg;
        target.has_truth = true;

        const ros::Time now = ros::Time::now();

        // 判断本次真值是否应该被丢弃（不发布测量）
        if (shouldDropMeasurement(target, now)) {
            // 即使丢弃，也要更新标记（可能删除旧标记）
            publishMarkers(now);
            return;
        }

        // 生成模拟测量（添加噪声和可能的异常值）
        nav_msgs::Odometry measurement = makeMeasurement(*msg, target_index);
        measurement_pubs_[target_index].publish(measurement);  // 发布测量

        // 保存最新有效测量和有效性标志
        latest_measurements_[target_index] = measurement;
        latest_valid_[target_index] = true;

        // 更新可视化标记
        publishMarkers(now);
    }

    /**
     * @brief 判断当前真值是否应被丢弃（不产生测量输出）
     * @param target 目标状态引用
     * @param now 当前时间
     * @return true 表示丢弃，false 表示正常生成测量
     *
     * 丢弃原因：
     * - 正在突发丢失期间
     * - 随机丢包概率命中
     * - 突发丢失触发概率命中（会开启一段突发丢失期）
     */
    bool shouldDropMeasurement(TargetMeasurementState& target, const ros::Time& now) {
        // 如果正处于突发丢失状态，且尚未结束，则丢弃
        if (target.in_burst_loss) {
            if (now < target.burst_loss_end) {
                return true;
            }
            // 突发丢失结束，清除状态
            target.in_burst_loss = false;
        }

        // 检查是否触发新的突发丢失
        if (enable_burst_loss_ && uniform01_() < burst_loss_prob_) {
            target.in_burst_loss = true;
            target.burst_loss_end = now + ros::Duration(burst_loss_duration_);
            return true;  // 本次丢弃，并开始一段持续丢失
        }

        // 随机丢包
        return uniform01_() < random_drop_prob_;
    }

    /**
     * @brief 根据真值生成模拟测量值（添加噪声和异常值）
     * @param truth 真值里程计消息
     * @param target_index 目标索引（用于设置child_frame_id）
     * @return 模拟测量消息
     */
    nav_msgs::Odometry makeMeasurement(const nav_msgs::Odometry& truth, int target_index) {
        nav_msgs::Odometry measurement = truth;  // 复制真值作为基础

        // 更新时间戳和坐标系
        measurement.header.stamp = ros::Time::now();
        measurement.header.frame_id = world_frame_;
        measurement.child_frame_id = "radar_target_" + std::to_string(target_index);

        // 添加位置高斯噪声
        measurement.pose.pose.position.x += normal_(position_noise_std_);
        measurement.pose.pose.position.y += normal_(position_noise_std_);
        measurement.pose.pose.position.z += normal_(position_noise_std_);

        // 添加速度高斯噪声
        measurement.twist.twist.linear.x += normal_(velocity_noise_std_);
        measurement.twist.twist.linear.y += normal_(velocity_noise_std_);
        measurement.twist.twist.linear.z += normal_(velocity_noise_std_);

        // 以一定概率添加异常值（大幅度位置跳跃）
        if (uniform01_() < outlier_prob_) {
            measurement.pose.pose.position.x += normal_(outlier_position_jump_);
            measurement.pose.pose.position.y += normal_(outlier_position_jump_);
            measurement.pose.pose.position.z += normal_(0.5 * outlier_position_jump_); // z轴跳跃幅度减半
        }

        return measurement;
    }

    /**
     * @brief 生成 [0,1) 均匀分布随机数
     */
    double uniform01_() {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(rng_);
    }

    /**
     * @brief 生成均值为0、标准差为stddev的高斯随机数
     * @param stddev 标准差，若<=0则返回0
     */
    double normal_(double stddev) {
        if (stddev <= 0.0) {
            return 0.0;
        }
        std::normal_distribution<double> dist(0.0, stddev);
        return dist(rng_);
    }

    /**
     * @brief 发布所有目标当前测量的可视化标记（球体）
     * @param stamp 时间戳
     *
     * 对于有效目标显示黄色球体，无效目标发送DELETE标记将其移除
     */
    void publishMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            visualization_msgs::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = world_frame_;
            marker.ns = "radar_measurements";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            // 有效则ADD，无效则DELETE
            marker.action = latest_valid_[i] ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

            if (latest_valid_[i]) {
                marker.pose = latest_measurements_[i].pose.pose;
                marker.scale.x = marker_scale_;
                marker.scale.y = marker_scale_;
                marker.scale.z = marker_scale_;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.1;   // 黄色
            }

            markers.markers.push_back(marker);
        }

        marker_pub_.publish(markers);
    }

    // ROS句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 参数
    std::string world_frame_;              // 世界坐标系名称
    int target_count_ = 3;                 // 目标数量
    double position_noise_std_ = 0.5;      // 位置噪声标准差
    double velocity_noise_std_ = 0.2;      // 速度噪声标准差
    double random_drop_prob_ = 0.1;        // 随机丢包概率
    double outlier_prob_ = 0.03;           // 异常值概率
    double outlier_position_jump_ = 8.0;   // 异常值跳跃幅度
    bool enable_burst_loss_ = true;        // 是否启用突发丢失
    double burst_loss_prob_ = 0.02;        // 突发丢失触发概率
    double burst_loss_duration_ = 1.0;     // 突发丢失持续时间
    double marker_scale_ = 0.35;           // 可视化标记大小

    // 随机数引擎
    std::mt19937 rng_;

    // 每个目标的状态
    std::vector<TargetMeasurementState> targets_;

    // 订阅器列表（每个目标一个）
    std::vector<ros::Subscriber> truth_subs_;

    // 发布器列表（每个目标一个）
    std::vector<ros::Publisher> measurement_pubs_;

    // 最近一次有效测量（用于标记发布）
    std::vector<nav_msgs::Odometry> latest_measurements_;

    // 每个目标是否有有效测量
    std::vector<bool> latest_valid_;

    // 可视化标记发布器
    ros::Publisher marker_pub_;
};

}  // namespace

/**
 * @brief 主函数：初始化节点，创建雷达测量模拟器实例，进入ROS事件循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_measurement_node");
    RadarMeasurementNode node;
    ros::spin();
    return 0;
}