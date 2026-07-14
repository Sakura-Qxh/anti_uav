#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

/**
 * @brief 任务管理器节点类
 *
 * 负责接收攻击目标（/attack_goal），并为多个目标（target）和拦截器（interceptor）
 * 生成对应的规划目标（planning_goal）和航点（waypoints），同时发布可视化标记。
 */
class MissionManagerNode {
public:
    MissionManagerNode() : nh_(), pnh_("~") {
        loadParams();                          // 加载参数
        createTargetInterfaces();              // 创建目标的发布接口
        createInterceptorInterfaces();         // 创建拦截器的发布接口

        // 订阅攻击目标话题
        attack_goal_sub_ = nh_.subscribe(attack_goal_topic_, 1,
                                          &MissionManagerNode::attackGoalCallback,
                                          this);

        // 发布可视化标记数组（所有目标的位置球体）
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/mission_manager/target_goal_markers", 1, true);

        // 创建定时器，按固定频率重新发布目标和标记
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
            &MissionManagerNode::timerCallback,
            this);

        // 初始化最新的攻击目标（默认位于世界原点上方2米）
        latest_attack_goal_.header.frame_id = world_frame_;
        latest_attack_goal_.pose.orientation.w = 1.0;
        latest_attack_goal_.pose.position.z = 2.0;
        has_goal_ = true;                      // 标记已有目标，便于定时器立即发布

        ROS_INFO("mission_manager started: targets=%d interceptors=%d",
                 target_count_, interceptor_count_);
    }

private:
    /**
     * @brief 从参数服务器加载配置参数
     */
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<int>("interceptor_count", interceptor_count_, 6);
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);
        pnh_.param<double>("formation_radius", formation_radius_, 4.0);      // 目标散布半径
        pnh_.param<bool>("spread_target_goals", spread_target_goals_, true);  // 是否将目标分散成圆形
        pnh_.param<bool>("latch_waypoints", latch_waypoints_, true);          // 航点发布是否锁存（latch）
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_, "/attack_goal");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");
        pnh_.param<std::string>("interceptor_prefix", interceptor_prefix_, "/interceptor_");

        // 确保数量至少为1
        target_count_ = std::max(1, target_count_);
        interceptor_count_ = std::max(1, interceptor_count_);
    }

    /**
     * @brief 为目标创建发布器（规划目标+航点）
     */
    void createTargetInterfaces() {
        target_goal_pubs_.resize(target_count_);
        target_waypoint_pubs_.resize(target_count_);

        for (int i = 0; i < target_count_; ++i) {
            const std::string ns = target_prefix_ + std::to_string(i);

            // 发布单个目标的规划目标位姿
            target_goal_pubs_[i] =
                nh_.advertise<geometry_msgs::PoseStamped>(ns + "/planning_goal", 1, true);

            // 发布航点路径（通常只包含一个航点）
            target_waypoint_pubs_[i] =
                nh_.advertise<nav_msgs::Path>(ns + "/waypoint_generator/waypoints",
                                              1,
                                              latch_waypoints_);
        }
    }

    /**
     * @brief 为拦截器创建发布器以及捕获目标订阅器
     */
    void createInterceptorInterfaces() {
        interceptor_goal_pubs_.resize(interceptor_count_);
        interceptor_waypoint_pubs_.resize(interceptor_count_);

        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string ns = interceptor_prefix_ + std::to_string(i);

            // 发布拦截器的规划目标位姿
            interceptor_goal_pubs_[i] =
                nh_.advertise<geometry_msgs::PoseStamped>(ns + "/planning_goal", 1, true);

            // 发布拦截器的航点路径
            interceptor_waypoint_pubs_[i] =
                nh_.advertise<nav_msgs::Path>(ns + "/waypoint_generator/waypoints",
                                              1,
                                              latch_waypoints_);

            // 订阅每个拦截器的捕获目标话题（capture_goal），收到后转发为规划目标和航点
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

    /**
     * @brief 攻击目标回调：更新最新的攻击目标，并立即发布所有目标的目标点
     * @param msg 收到的攻击目标位姿
     */
    void attackGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        latest_attack_goal_ = *msg;

        if (latest_attack_goal_.header.frame_id.empty()) {
            latest_attack_goal_.header.frame_id = world_frame_;
        }

        has_goal_ = true;
        publishTargetGoals();   // 立即发布
    }

    /**
     * @brief 定时器回调：若有目标，则周期性重新发布所有目标的目标点
     */
    void timerCallback(const ros::TimerEvent&) {
        if (has_goal_) {
            publishTargetGoals();
        }
    }

    /**
     * @brief 根据索引生成单个目标的规划目标位姿（可形成圆形队形）
     * @param index 目标编号
     * @return 生成的位姿消息
     */
    geometry_msgs::PoseStamped makeTargetGoal(int index) const {
        geometry_msgs::PoseStamped goal = latest_attack_goal_;
        goal.header.stamp = ros::Time::now();

        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = world_frame_;
        }

        // 如果启用了散布且目标数量大于1，则在水平面上均匀分布在圆周上
        if (spread_target_goals_ && target_count_ > 1) {
            const double angle =
                2.0 * M_PI * static_cast<double>(index) / static_cast<double>(target_count_);
            goal.pose.position.x += formation_radius_ * std::cos(angle);
            goal.pose.position.y += formation_radius_ * std::sin(angle);
        }

        // 强制无旋转（四元数单位元）
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        return goal;
    }

    /**
     * @brief 通用函数：将一个位姿作为规划目标和航点发布
     * @param input 输入位姿
     * @param goal_pub 规划目标发布器
     * @param waypoint_pub 航点发布器
     */
    void publishGoalAndWaypoint(const geometry_msgs::PoseStamped& input,
                                ros::Publisher& goal_pub,
                                ros::Publisher& waypoint_pub) {
        geometry_msgs::PoseStamped goal = input;
        goal.header.stamp = ros::Time::now();

        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = world_frame_;
        }

        // 若四元数为零，则设为无旋转
        if (goal.pose.orientation.w == 0.0 &&
            goal.pose.orientation.x == 0.0 &&
            goal.pose.orientation.y == 0.0 &&
            goal.pose.orientation.z == 0.0) {
            goal.pose.orientation.w = 1.0;
        }

        // 构建航点路径（只包含这一个航点）
        nav_msgs::Path path;
        path.header = goal.header;
        path.poses.push_back(goal);

        goal_pub.publish(goal);
        waypoint_pub.publish(path);
    }

    /**
     * @brief 发布所有目标的规划目标和可视化标记
     */
    void publishTargetGoals() {
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            const geometry_msgs::PoseStamped goal = makeTargetGoal(i);

            // 发布该目标的规划目标和航点
            publishGoalAndWaypoint(goal,
                                   target_goal_pubs_[i],
                                   target_waypoint_pubs_[i]);

            // 创建可视化标记（红色球体）
            visualization_msgs::Marker marker;
            marker.header = goal.header;
            marker.ns = "target_planning_goals";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = goal.pose;
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

    // ROS句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 参数
    std::string world_frame_;               // 世界坐标系
    int target_count_ = 3;                  // 目标数量
    int interceptor_count_ = 6;             // 拦截器数量
    double publish_rate_ = 5.0;             // 发布频率（Hz）
    double formation_radius_ = 4.0;         // 目标散布半径（米）
    bool spread_target_goals_ = true;       // 是否将目标分散成圆形
    bool latch_waypoints_ = true;           // 航点话题是否锁存

    std::string attack_goal_topic_;         // 攻击目标话题
    std::string target_prefix_;             // 目标命名空间前缀
    std::string interceptor_prefix_;        // 拦截器命名空间前缀

    // 状态
    bool has_goal_ = false;                 // 是否已收到有效的攻击目标
    geometry_msgs::PoseStamped latest_attack_goal_;  // 最近一次收到的攻击目标

    // 订阅器
    ros::Subscriber attack_goal_sub_;                             // 攻击目标订阅
    std::vector<ros::Subscriber> interceptor_capture_goal_subs_;  // 各拦截器的捕获目标订阅

    // 发布器
    std::vector<ros::Publisher> target_goal_pubs_;        // 各目标的规划目标发布器
    std::vector<ros::Publisher> target_waypoint_pubs_;    // 各目标的航点发布器
    std::vector<ros::Publisher> interceptor_goal_pubs_;   // 各拦截器的规划目标发布器
    std::vector<ros::Publisher> interceptor_waypoint_pubs_; // 各拦截器的航点发布器

    ros::Publisher marker_pub_;   // 可视化标记数组发布器
    ros::Timer timer_;            // 定时器
};

/**
 * @brief 主函数：初始化节点，创建任务管理器对象，进入ROS事件循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_manager_node");
    MissionManagerNode node;
    ros::spin();
    return 0;
}