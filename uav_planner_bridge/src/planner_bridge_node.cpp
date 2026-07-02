#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

struct UavPlanState {
    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped goal;
    bool has_odom = false;
    bool has_goal = false;
    ros::Time last_plan_time;
    geometry_msgs::Point last_goal_point;
};

class PlannerBridgeNode {
public:
    PlannerBridgeNode() : nh_(), pnh_("~") {
        loadParams();
        states_.resize(uav_count_);
        createRosInterfaces();
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(update_rate_, 1e-3)),
            &PlannerBridgeNode::timerCallback,
            this);
        ROS_INFO("planner_bridge_node started: mode=%s, prefix=%s, count=%d",
                 mode_.c_str(), uav_prefix_.c_str(), uav_count_);
    }

private:
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<std::string>("mode", mode_, "target");
        pnh_.param<int>("uav_count", uav_count_, 1);
        pnh_.param<std::string>("uav_prefix", uav_prefix_, "/target_");
        pnh_.param<double>("update_rate", update_rate_, 5.0);
        pnh_.param<int>("path_points", path_points_, 80);
        pnh_.param<double>("min_replan_interval", min_replan_interval_, 0.5);
        pnh_.param<double>("replan_distance_threshold", replan_distance_threshold_, 1.0);
        pnh_.param<std::string>("global_map_topic", global_map_topic_, "/global_map");
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_,
                                "/target_scenario/attack_goal");
    }

    void createRosInterfaces() {
        map_sub_ = nh_.subscribe(global_map_topic_, 1,
                                 &PlannerBridgeNode::mapCallback, this);

        if (mode_ == "target") {
            attack_goal_sub_ = nh_.subscribe(attack_goal_topic_, 10,
                                             &PlannerBridgeNode::attackGoalCallback,
                                             this);
        }

        for (int i = 0; i < uav_count_; ++i) {
            const std::string base = uav_prefix_ + std::to_string(i);
            odom_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    base + "/odom", 10,
                    boost::bind(&PlannerBridgeNode::odomCallback, this, _1, i)));

            if (mode_ == "interceptor") {
                goal_subs_.push_back(
                    nh_.subscribe<geometry_msgs::PoseStamped>(
                        base + "/planning_goal", 10,
                        boost::bind(&PlannerBridgeNode::goalCallback, this, _1, i)));
            }

            path_pubs_.push_back(
                nh_.advertise<nav_msgs::Path>(base + "/planned_path", 1, true));
        }
    }

    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        latest_map_stamp_ = msg->header.stamp;
        has_map_ = true;
    }

    void attackGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        for (auto& state : states_) {
            state.goal = *msg;
            state.has_goal = true;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int idx) {
        if (idx < 0 || idx >= static_cast<int>(states_.size())) {
            return;
        }
        states_[idx].odom = *msg;
        states_[idx].has_odom = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int idx) {
        if (idx < 0 || idx >= static_cast<int>(states_.size())) {
            return;
        }
        states_[idx].goal = *msg;
        states_[idx].has_goal = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        for (int i = 0; i < static_cast<int>(states_.size()); ++i) {
            auto& state = states_[i];
            if (!state.has_odom || !state.has_goal) {
                continue;
            }
            if (!needReplan(state, now)) {
                continue;
            }

            nav_msgs::Path path = makeStraightPath(state.odom, state.goal, now);
            path_pubs_[i].publish(path);
            state.last_plan_time = now;
            state.last_goal_point = state.goal.pose.position;
        }
    }

    bool needReplan(const UavPlanState& state, const ros::Time& now) const {
        if (state.last_plan_time.isZero()) {
            return true;
        }
        if ((now - state.last_plan_time).toSec() < min_replan_interval_) {
            return false;
        }
        const auto& a = state.last_goal_point;
        const auto& b = state.goal.pose.position;
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) >= replan_distance_threshold_;
    }

    nav_msgs::Path makeStraightPath(const nav_msgs::Odometry& odom,
                                    const geometry_msgs::PoseStamped& goal,
                                    const ros::Time& stamp) const {
        nav_msgs::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = world_frame_;
        const auto& start = odom.pose.pose.position;
        const auto& end = goal.pose.position;
        const int n = std::max(2, path_points_);
        for (int k = 0; k < n; ++k) {
            const double s = static_cast<double>(k) / static_cast<double>(n - 1);
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = start.x + s * (end.x - start.x);
            pose.pose.position.y = start.y + s * (end.y - start.y);
            pose.pose.position.z = start.z + s * (end.z - start.z);
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;
    std::string world_frame_ = "world";
    std::string mode_ = "target";
    int uav_count_ = 1;
    std::string uav_prefix_ = "/target_";
    double update_rate_ = 5.0;
    int path_points_ = 80;
    double min_replan_interval_ = 0.5;
    double replan_distance_threshold_ = 1.0;
    std::string global_map_topic_ = "/global_map";
    std::string attack_goal_topic_ = "/target_scenario/attack_goal";
    bool has_map_ = false;
    ros::Time latest_map_stamp_;
    std::vector<UavPlanState> states_;
    ros::Subscriber map_sub_;
    ros::Subscriber attack_goal_sub_;
    std::vector<ros::Subscriber> odom_subs_;
    std::vector<ros::Subscriber> goal_subs_;
    std::vector<ros::Publisher> path_pubs_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_bridge_node");
    PlannerBridgeNode node;
    ros::spin();
    return 0;
}