#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class MissionManagerNode {
public:
    MissionManagerNode() : nh_(), pnh_("~") {
        loadParams();
        createTargetInterfaces();
        createInterceptorInterfaces();

        attack_goal_sub_ = nh_.subscribe(attack_goal_topic_, 1,
                                          &MissionManagerNode::attackGoalCallback,
                                          this);

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/mission_manager/target_goal_markers", 1, true);

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
            &MissionManagerNode::timerCallback,
            this);

        latest_attack_goal_.header.frame_id = world_frame_;
        latest_attack_goal_.pose.orientation.w = 1.0;
        latest_attack_goal_.pose.position.z = 2.0;
        has_goal_ = true;

        ROS_INFO("mission_manager started: targets=%d interceptors=%d",
                 target_count_, interceptor_count_);
    }

private:
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 6);
        pnh_.param<int>("interceptor_count", interceptor_count_, 6);
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);
        pnh_.param<double>("formation_radius", formation_radius_, 4.0);
        pnh_.param<bool>("spread_target_goals", spread_target_goals_, true);
        pnh_.param<bool>("latch_waypoints", latch_waypoints_, true);
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_, "/attack_goal");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");
        pnh_.param<std::string>("interceptor_prefix", interceptor_prefix_, "/interceptor_");

        target_count_ = std::max(1, target_count_);
        interceptor_count_ = std::max(1, interceptor_count_);
    }

    void createTargetInterfaces() {
        target_goal_pubs_.resize(target_count_);
        target_waypoint_pubs_.resize(target_count_);

        for (int i = 0; i < target_count_; ++i) {
            const std::string ns = target_prefix_ + std::to_string(i);

            target_goal_pubs_[i] =
                nh_.advertise<geometry_msgs::PoseStamped>(ns + "/planning_goal", 1, true);

            target_waypoint_pubs_[i] =
                nh_.advertise<nav_msgs::Path>(ns + "/waypoint_generator/waypoints",
                                              1,
                                              latch_waypoints_);
        }
    }

    void createInterceptorInterfaces() {
        interceptor_goal_pubs_.resize(interceptor_count_);
        interceptor_waypoint_pubs_.resize(interceptor_count_);

        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string ns = interceptor_prefix_ + std::to_string(i);

            interceptor_goal_pubs_[i] =
                nh_.advertise<geometry_msgs::PoseStamped>(ns + "/planning_goal", 1, true);

            interceptor_waypoint_pubs_[i] =
                nh_.advertise<nav_msgs::Path>(ns + "/waypoint_generator/waypoints",
                                              1,
                                              latch_waypoints_);

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

    void attackGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        latest_attack_goal_ = *msg;

        if (latest_attack_goal_.header.frame_id.empty()) {
            latest_attack_goal_.header.frame_id = world_frame_;
        }

        has_goal_ = true;
        publishTargetGoals();
    }

    void timerCallback(const ros::TimerEvent&) {
        if (has_goal_) {
            publishTargetGoals();
        }
    }

    geometry_msgs::PoseStamped makeTargetGoal(int index) const {
        geometry_msgs::PoseStamped goal = latest_attack_goal_;
        goal.header.stamp = ros::Time::now();

        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = world_frame_;
        }

        if (spread_target_goals_ && target_count_ > 1) {
            const double angle =
                2.0 * M_PI * static_cast<double>(index) / static_cast<double>(target_count_);
            goal.pose.position.x += formation_radius_ * std::cos(angle);
            goal.pose.position.y += formation_radius_ * std::sin(angle);
        }

        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        return goal;
    }

    void publishGoalAndWaypoint(const geometry_msgs::PoseStamped& input,
                                ros::Publisher& goal_pub,
                                ros::Publisher& waypoint_pub) {
        geometry_msgs::PoseStamped goal = input;
        goal.header.stamp = ros::Time::now();

        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = world_frame_;
        }

        if (goal.pose.orientation.w == 0.0 &&
            goal.pose.orientation.x == 0.0 &&
            goal.pose.orientation.y == 0.0 &&
            goal.pose.orientation.z == 0.0) {
            goal.pose.orientation.w = 1.0;
        }

        nav_msgs::Path path;
        path.header = goal.header;
        path.poses.push_back(goal);

        goal_pub.publish(goal);
        waypoint_pub.publish(path);
    }

    void publishTargetGoals() {
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            const geometry_msgs::PoseStamped goal = makeTargetGoal(i);

            publishGoalAndWaypoint(goal,
                                   target_goal_pubs_[i],
                                   target_waypoint_pubs_[i]);

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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string world_frame_;
    int target_count_ = 6;
    int interceptor_count_ = 6;
    double publish_rate_ = 5.0;
    double formation_radius_ = 4.0;
    bool spread_target_goals_ = true;
    bool latch_waypoints_ = true;

    std::string attack_goal_topic_;
    std::string target_prefix_;
    std::string interceptor_prefix_;

    bool has_goal_ = false;
    geometry_msgs::PoseStamped latest_attack_goal_;

    ros::Subscriber attack_goal_sub_;
    std::vector<ros::Subscriber> interceptor_capture_goal_subs_;

    std::vector<ros::Publisher> target_goal_pubs_;
    std::vector<ros::Publisher> target_waypoint_pubs_;
    std::vector<ros::Publisher> interceptor_goal_pubs_;
    std::vector<ros::Publisher> interceptor_waypoint_pubs_;

    ros::Publisher marker_pub_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_manager_node");
    MissionManagerNode node;
    ros::spin();
    return 0;
}