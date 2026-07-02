#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

class GoalToWaypointsBridge {
public:
    GoalToWaypointsBridge() : pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<bool>("latch_waypoints", latch_waypoints_, true);

        goal_sub_ = pnh_.subscribe("planning_goal", 1, &GoalToWaypointsBridge::goalCallback, this);
        waypoints_pub_ = pnh_.advertise<nav_msgs::Path>("waypoints", 1, latch_waypoints_);

        ROS_INFO("goal_to_waypoints_bridge_node started");
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        nav_msgs::Path path;
        path.header = msg->header;
        if (path.header.frame_id.empty()) {
            path.header.frame_id = world_frame_;
        }
        if (path.header.stamp.isZero()) {
            path.header.stamp = ros::Time::now();
        }

        geometry_msgs::PoseStamped pose = *msg;
        pose.header = path.header;
        if (pose.pose.orientation.w == 0.0 &&
            pose.pose.orientation.x == 0.0 &&
            pose.pose.orientation.y == 0.0 &&
            pose.pose.orientation.z == 0.0) {
            pose.pose.orientation.w = 1.0;
        }

        path.poses.push_back(pose);
        waypoints_pub_.publish(path);
    }

    ros::NodeHandle pnh_;
    std::string world_frame_;
    bool latch_waypoints_ = true;
    ros::Subscriber goal_sub_;
    ros::Publisher waypoints_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_to_waypoints_bridge_node");
    GoalToWaypointsBridge node;
    ros::spin();
    return 0;
}