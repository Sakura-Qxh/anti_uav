// 包含标准库和ROS消息类型
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>   // 点击点消息
#include <geometry_msgs/PoseStamped.h>    // 位姿消息
#include <ros/ros.h>                      // ROS核心库
#include <visualization_msgs/Marker.h>    // RViz可视化标记消息

/**
 * @brief 目标攻击点节点类
 * 
 * 监听点击点话题，更新攻击目标位姿，并以固定频率发布目标点和可视化标记
 */
class TargetAttackGoalNode {
public:
    /**
     * @brief 构造函数：初始化参数、订阅、发布器和定时器
     */
    TargetAttackGoalNode() : nh_(), pnh_("~") {
        // 从私有命名空间读取参数，若无则使用默认值
        pnh_.param<std::string>("world_frame", world_frame_, "world");           // 世界坐标系名称
        pnh_.param<std::string>("clicked_point_topic", clicked_point_topic_, "/clicked_point"); // 点击点话题
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_, "/attack_goal");       // 攻击目标话题
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);                  // 发布频率（Hz）
        pnh_.param<double>("default_altitude", default_altitude_, 2.0);          // 默认高度（当点击点z≤0.1时使用）

        // 尝试从参数服务器读取默认攻击目标（三维坐标数组），若无效则使用(0,0,default_altitude_)
        std::vector<double> goal;
        if (pnh_.getParam("default_attack_goal", goal) && goal.size() >= 3) {
            attack_goal_.pose.position.x = goal[0];
            attack_goal_.pose.position.y = goal[1];
            attack_goal_.pose.position.z = goal[2];
        } else {
            attack_goal_.pose.position.x = 0.0;
            attack_goal_.pose.position.y = 0.0;
            attack_goal_.pose.position.z = default_altitude_;
        }
        attack_goal_.pose.orientation.w = 1.0;           // 默认无旋转（四元数单位元）
        attack_goal_.header.frame_id = world_frame_;     // 设置初始帧ID

        // 订阅点击点话题，回调函数更新攻击目标
        clicked_sub_ = nh_.subscribe(clicked_point_topic_, 10, &TargetAttackGoalNode::clickedCallback, this);
        // 发布攻击目标位姿（latch=true，确保新订阅者立即收到最新消息）
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(attack_goal_topic_, 1, true);
        // 发布RViz可视化标记（球体）
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/attack_goal_marker", 1, true);
        // 创建定时器，按指定频率触发定时回调
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
                                &TargetAttackGoalNode::timerCallback, this);
    }

private:
    /**
     * @brief 点击点回调：根据接收到的点更新攻击目标
     * @param msg 点击点消息（PointStamped）
     */
    void clickedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        attack_goal_.header.stamp = ros::Time::now();                            // 更新时间戳
        attack_goal_.header.frame_id = msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id; // 使用点击点的帧ID，若为空则用world_frame_
        attack_goal_.pose.position.x = msg->point.x;
        attack_goal_.pose.position.y = msg->point.y;
        attack_goal_.pose.position.z = msg->point.z > 0.1 ? msg->point.z : default_altitude_; // 若点击点高度有效则使用，否则使用默认高度
        attack_goal_.pose.orientation.w = 1.0;                                   // 保持无旋转
        publish();                                                               // 立即发布更新
    }

    /**
     * @brief 定时器回调：周期性重新发布当前攻击目标和标记
     */
    void timerCallback(const ros::TimerEvent&) {
        publish();
    }

    /**
     * @brief 发布当前攻击目标位姿和对应的可视化标记（金色球体）
     */
    void publish() {
        attack_goal_.header.stamp = ros::Time::now();   // 更新时间戳
        goal_pub_.publish(attack_goal_);                // 发布位姿

        // 构造并发布RViz标记（球体，颜色金色，半透明）
        visualization_msgs::Marker marker;
        marker.header = attack_goal_.header;             // 与位姿相同的帧和时间戳
        marker.ns = "attack_goal";                       // 命名空间
        marker.id = 0;                                   // 唯一ID
        marker.type = visualization_msgs::Marker::SPHERE; // 球体形状
        marker.action = visualization_msgs::Marker::ADD; // 添加/更新
        marker.pose = attack_goal_.pose;                 // 位置和姿态
        marker.scale.x = 1.8;                            // 直径（米）
        marker.scale.y = 1.8;
        marker.scale.z = 1.8;
        marker.color.r = 1.0;                            // 红色分量
        marker.color.g = 0.8;                            // 绿色分量
        marker.color.b = 0.0;                            // 蓝色分量 → 金色
        marker.color.a = 0.9;                            // 透明度
        marker_pub_.publish(marker);
    }

    // 成员变量
    ros::NodeHandle nh_;                    // 公共句柄
    ros::NodeHandle pnh_;                   // 私有句柄（用于读取~参数）
    std::string world_frame_;               // 世界坐标系名称
    std::string clicked_point_topic_;       // 点击点话题名
    std::string attack_goal_topic_;         // 攻击目标话题名
    double publish_rate_;                   // 发布频率
    double default_altitude_;               // 默认高度
    geometry_msgs::PoseStamped attack_goal_; // 当前攻击目标位姿
    ros::Subscriber clicked_sub_;           // 点击点订阅器
    ros::Publisher goal_pub_;               // 目标位姿发布器
    ros::Publisher marker_pub_;             // 可视化标记发布器
    ros::Timer timer_;                      // 定时器，用于周期发布
};

/**
 * @brief 主函数：初始化ROS节点，启动节点对象，进入事件循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "target_attack_goal_node"); // 初始化节点名称
    TargetAttackGoalNode node;                         // 创建节点实例
    ros::spin();                                       // 等待回调
    return 0;
}