#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

// 目标状态结构体：存储里程计数据、时间戳和有效性标志
struct TargetState {
    nav_msgs::Odometry odom;
    ros::Time stamp;
    bool valid = false;
};

// 计算二维向量的模长
double norm2d(double x, double y) {
    return std::sqrt(x * x + y * y);
}

}  // namespace

// 任务管理器节点：负责接收攻击目标，生成编队航点并转发给目标和拦截器
class MissionManagerNode {
public:
    MissionManagerNode() : nh_(), pnh_("~") {
        loadParams();
        target_states_.resize(target_count_);
        last_target_goals_.resize(target_count_);
        last_target_goal_times_.resize(target_count_);
        has_published_target_goal_.assign(target_count_, false);
        createTargetInterfaces();
        createInterceptorInterfaces();

        // 订阅外部攻击目标话题
        attack_goal_sub_ = nh_.subscribe(
            attack_goal_topic_, 1, &MissionManagerNode::attackGoalCallback, this);

        // 可视化标记发布器（用于在Rviz中显示编队目标点）
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/mission_manager/target_goal_markers", 1, true);

        // 定时器：定期重新发布编队航点
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
            &MissionManagerNode::timerCallback,
            this);

        // 设置默认攻击目标（坐标(24,0,2)），并标记有目标可用
        latest_attack_goal_.header.frame_id = world_frame_;
        latest_attack_goal_.pose.position.x = 24.0;
        latest_attack_goal_.pose.position.y = 0.0;
        latest_attack_goal_.pose.position.z = 2.0;
        latest_attack_goal_.pose.orientation.w = 1.0;
        has_goal_ = true;

        ROS_INFO("mission_manager started: targets=%d interceptors=%d",
                 target_count_, interceptor_count_);
    }

private:
    // 从参数服务器加载所有配置参数
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<int>("interceptor_count", interceptor_count_, 4);
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);
        pnh_.param<double>("formation_radius", formation_radius_, 3.0);
        pnh_.param<double>("formation_lookahead", formation_lookahead_, 8.0);
        pnh_.param<double>("formation_heading_lock_distance",
                           formation_heading_lock_distance_, 5.0);
        pnh_.param<double>("odom_timeout", odom_timeout_, 1.0);
        pnh_.param<double>("goal_publish_rate", goal_publish_rate_, 2.0);
        pnh_.param<double>("goal_position_threshold", goal_position_threshold_, 0.3);
        pnh_.param<bool>("spread_target_goals", spread_target_goals_, true);
        pnh_.param<bool>("latch_waypoints", latch_waypoints_, true);
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_, "/attack_goal");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");
        pnh_.param<std::string>("interceptor_prefix", interceptor_prefix_, "/interceptor_");

        // 确保参数合理
        target_count_ = std::max(1, target_count_);
        interceptor_count_ = std::max(1, interceptor_count_);
        formation_heading_lock_distance_ = std::max(0.1, formation_heading_lock_distance_);
        goal_publish_rate_ = std::max(0.1, goal_publish_rate_);
        goal_position_threshold_ = std::max(0.0, goal_position_threshold_);
    }

    // 创建与目标相关的发布器和订阅器
    void createTargetInterfaces() {
        target_goal_pubs_.resize(target_count_);
        target_waypoint_pubs_.resize(target_count_);

        for (int i = 0; i < target_count_; ++i) {
            const std::string ns = target_prefix_ + std::to_string(i);
            // 发布单个航点（用于规划模块）
            target_goal_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
                ns + "/planning_goal", 1, true);
            // 发布航点路径（用于航点生成器）
            target_waypoint_pubs_[i] = nh_.advertise<nav_msgs::Path>(
                ns + "/waypoint_generator/waypoints", 1, latch_waypoints_);
            // 订阅目标里程计（用于编队计算）
            target_odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                ns + "/odom",
                10,
                [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    targetOdomCallback(msg, i);
                }));
        }
    }

    // 创建与拦截器相关的发布器和订阅器
    void createInterceptorInterfaces() {
        interceptor_goal_pubs_.resize(interceptor_count_);
        interceptor_waypoint_pubs_.resize(interceptor_count_);

        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string ns = interceptor_prefix_ + std::to_string(i);
            // 发布拦截器航点
            interceptor_goal_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
                ns + "/planning_goal", 1, true);
            interceptor_waypoint_pubs_[i] = nh_.advertise<nav_msgs::Path>(
                ns + "/waypoint_generator/waypoints", 1, latch_waypoints_);
            // 订阅来自协同制导节点的捕获目标（capture_goal）
            interceptor_capture_goal_subs_.push_back(
                nh_.subscribe<geometry_msgs::PoseStamped>(
                    ns + "/capture_goal",
                    1,
                    [this, i](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                        publishGoalAndWaypoint(*msg,
                                               interceptor_goal_pubs_[i],
                                               interceptor_waypoint_pubs_[i]);
                    }));
        }
    }

    // 目标里程计回调：更新对应目标的状态
    void targetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, int index) {
        if (index < 0 || index >= target_count_) {
            return;
        }
        target_states_[index].odom = *msg;
        target_states_[index].stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        target_states_[index].valid = true;
    }

    // 攻击目标回调：接收外部指令，更新攻击目标并强制重新发布编队航点
    void attackGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        geometry_msgs::PoseStamped incoming = *msg;
        // 补全缺失的坐标系和时间信息
        if (incoming.header.frame_id.empty()) {
            incoming.header.frame_id = world_frame_;
        }
        if (incoming.pose.position.z < 0.1) {
            incoming.pose.position.z = 2.0;
        }
        incoming.pose.orientation.w = 1.0;

        // 如果新目标与当前目标几乎相同则忽略，避免重复触发
        if (has_goal_) {
            const double dx = incoming.pose.position.x - latest_attack_goal_.pose.position.x;
            const double dy = incoming.pose.position.y - latest_attack_goal_.pose.position.y;
            const double dz = incoming.pose.position.z - latest_attack_goal_.pose.position.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < 0.05 &&
                incoming.header.frame_id == latest_attack_goal_.header.frame_id) {
                return;
            }
        }

        latest_attack_goal_ = incoming;
        formation_heading_locked_ = false;   // 解锁编队朝向，以便重新计算
        has_goal_ = true;
        publishTargetGoals(true);            // 强制立即发布
    }

    // 定时器回调：周期性重新发布编队航点
    void timerCallback(const ros::TimerEvent&) {
        if (has_goal_) {
            publishTargetGoals();
        }
    }

    // 检查所有目标里程计是否都在超时时间内有效
    bool targetOdomFresh(const ros::Time& now) const {
        if (target_count_ != 3) {
            ROS_WARN_THROTTLE(2.0, "moving formation requires target_count=3");
            return false;
        }
        for (const auto& target : target_states_) {
            if (!target.valid || (now - target.stamp).toSec() > odom_timeout_) {
                return false;
            }
        }
        return true;
    }

    // 生成编队目标点：根据攻击目标和目标当前位置计算每个目标的期望位置
    std::vector<geometry_msgs::PoseStamped> makeFormationGoals(
        const ros::Time& now) {
        std::vector<geometry_msgs::PoseStamped> goals(target_count_, latest_attack_goal_);

        // 如果不启用编队分散模式或目标里程计数据不可用，则使用最终编队位置（围绕攻击目标均匀分布）
        if (!spread_target_goals_ || !targetOdomFresh(now)) {
            ROS_WARN_THROTTLE(2.0, "target Odom not ready; using final formation slots");
            for (int i = 0; i < target_count_; ++i) {
                const double angle = 2.0 * kPi * static_cast<double>(i) /
                                     static_cast<double>(target_count_);
                goals[i].pose.position.x += formation_radius_ * std::cos(angle);
                goals[i].pose.position.y += formation_radius_ * std::sin(angle);
            }
            return goals;
        }

        // 计算目标群的几何中心
        geometry_msgs::Point center;
        for (const auto& target : target_states_) {
            center.x += target.odom.pose.pose.position.x;
            center.y += target.odom.pose.pose.position.y;
            center.z += target.odom.pose.pose.position.z;
        }
        center.x /= static_cast<double>(target_count_);
        center.y /= static_cast<double>(target_count_);
        center.z /= static_cast<double>(target_count_);

        // 计算从群中心到攻击目标的方向向量和距离
        const double dx = latest_attack_goal_.pose.position.x - center.x;
        const double dy = latest_attack_goal_.pose.position.y - center.y;
        const double distance = norm2d(dx, dy);

        // 如果距离大于锁定距离，则持续更新编队朝向；否则锁定朝向
        if (!formation_heading_locked_ && distance > formation_heading_lock_distance_) {
            last_heading_ = std::atan2(dy, dx);
        } else if (!formation_heading_locked_ && distance <= formation_heading_lock_distance_) {
            formation_heading_locked_ = true;
            ROS_INFO("target formation heading locked at %.3f rad", last_heading_);
        }

        // 计算前视参考点：沿方向移动 lookahead 距离（不超过实际距离）
        const double step = std::min(formation_lookahead_, distance);
        geometry_msgs::Point reference = center;
        if (distance > 1e-3) {
            reference.x += step * dx / distance;
            reference.y += step * dy / distance;
        }
        reference.z = latest_attack_goal_.pose.position.z;

        // 以参考点为中心，按照锁定朝向均匀分布各目标
        for (int i = 0; i < target_count_; ++i) {
            const double angle = last_heading_ +
                2.0 * kPi * static_cast<double>(i) /
                static_cast<double>(target_count_);
            goals[i] = latest_attack_goal_;
            goals[i].header.stamp = now;
            goals[i].header.frame_id = world_frame_;
            goals[i].pose.position.x = reference.x + formation_radius_ * std::cos(angle);
            goals[i].pose.position.y = reference.y + formation_radius_ * std::sin(angle);
            goals[i].pose.position.z = reference.z;
            goals[i].pose.orientation.x = 0.0;
            goals[i].pose.orientation.y = 0.0;
            goals[i].pose.orientation.z = 0.0;
            goals[i].pose.orientation.w = 1.0;
        }
        return goals;
    }

    // 通用函数：将输入航点同时发布为单个目标（PoseStamped）和路径（Path）
    void publishGoalAndWaypoint(const geometry_msgs::PoseStamped& input,
                                ros::Publisher& goal_pub,
                                ros::Publisher& waypoint_pub) {
        geometry_msgs::PoseStamped goal = input;
        goal.header.stamp = ros::Time::now();
        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = world_frame_;
        }
        goal.pose.orientation.w = 1.0;

        nav_msgs::Path path;
        path.header = goal.header;
        path.poses.push_back(goal);
        goal_pub.publish(goal);
        waypoint_pub.publish(path);
    }

    // 判断是否需要重新发布某个目标的航点（基于发布频率和位置变化阈值）
    bool shouldPublishTargetGoal(int index,
                                 const geometry_msgs::PoseStamped& goal,
                                 const ros::Time& now,
                                 bool force) const {
        if (force || !has_published_target_goal_[index]) return true;
        if ((now - last_target_goal_times_[index]).toSec() < 1.0 / goal_publish_rate_) {
            return false;
        }
        const double dx = goal.pose.position.x - last_target_goals_[index].pose.position.x;
        const double dy = goal.pose.position.y - last_target_goals_[index].pose.position.y;
        const double dz = goal.pose.position.z - last_target_goals_[index].pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) >= goal_position_threshold_;
    }

    // 发布所有目标的编队航点（可选强制发布）
    void publishTargetGoals(bool force = false) {
        const ros::Time now = ros::Time::now();
        const auto goals = makeFormationGoals(now);
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            // 如果满足发布条件则发布
            if (shouldPublishTargetGoal(i, goals[i], now, force)) {
                publishGoalAndWaypoint(
                    goals[i], target_goal_pubs_[i], target_waypoint_pubs_[i]);
                last_target_goals_[i] = goals[i];
                last_target_goal_times_[i] = now;
                has_published_target_goal_[i] = true;
            }

            // 创建可视化标记（红色球体表示编队目标点）
            visualization_msgs::Marker marker;
            marker.header = goals[i].header;
            marker.ns = "target_formation_goals";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = goals[i].pose;
            marker.scale.x = 0.8;
            marker.scale.y = 0.8;
            marker.scale.z = 0.8;
            marker.color.r = 1.0;
            marker.color.g = 0.2;
            marker.color.b = 0.1;
            marker.color.a = 0.9;
            markers.markers.push_back(marker);
        }
        marker_pub_.publish(markers);
    }

    // ROS句柄和定时器
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    // 配置参数
    std::string world_frame_ = "world";
    int target_count_ = 3;
    int interceptor_count_ = 4;
    double publish_rate_ = 5.0;
    double formation_radius_ = 3.0;           // 编队半径
    double formation_lookahead_ = 8.0;        // 编队前视距离
    double formation_heading_lock_distance_ = 5.0; // 锁定朝向的距离阈值
    double odom_timeout_ = 1.0;               // 里程计超时时间
    double goal_publish_rate_ = 2.0;          // 航点发布频率上限
    double goal_position_threshold_ = 0.3;    // 位置变化阈值
    bool spread_target_goals_ = true;         // 是否分散编队目标
    bool latch_waypoints_ = true;             // 航点路径是否持久化发布
    double last_heading_ = 0.0;               // 最近一次计算的编队朝向
    bool formation_heading_locked_ = false;   // 编队朝向是否已锁定

    std::string attack_goal_topic_ = "/attack_goal";
    std::string target_prefix_ = "/target_";
    std::string interceptor_prefix_ = "/interceptor_";

    // 状态变量
    bool has_goal_ = false;
    geometry_msgs::PoseStamped latest_attack_goal_;
    std::vector<TargetState> target_states_;
    std::vector<geometry_msgs::PoseStamped> last_target_goals_;
    std::vector<ros::Time> last_target_goal_times_;
    std::vector<bool> has_published_target_goal_;

    // ROS通信接口
    ros::Subscriber attack_goal_sub_;
    std::vector<ros::Subscriber> target_odom_subs_;
    std::vector<ros::Subscriber> interceptor_capture_goal_subs_;
    std::vector<ros::Publisher> target_goal_pubs_;
    std::vector<ros::Publisher> target_waypoint_pubs_;
    std::vector<ros::Publisher> interceptor_goal_pubs_;
    std::vector<ros::Publisher> interceptor_waypoint_pubs_;
    ros::Publisher marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_manager_node");
    MissionManagerNode node;
    ros::spin();
    return 0;
}