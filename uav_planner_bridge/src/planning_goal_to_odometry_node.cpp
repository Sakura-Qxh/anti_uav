#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class PlanningGoalToOdometryNode {
public:
    PlanningGoalToOdometryNode() : pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<double>("publish_rate", publish_rate_, 20.0);

        goal_sub_ = pnh_.subscribe("planning_goal", 1,
                                   &PlanningGoalToOdometryNode::goalCallback,
                                   this);
        odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("target_odom", 1, true);
        timer_ = pnh_.createTimer(ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
                                  &PlanningGoalToOdometryNode::timerCallback,
                                  this);
        ROS_INFO("planning_goal_to_odometry_node started");
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        latest_goal_ = *msg;
        has_goal_ = true;
        publish(ros::Time::now());
    }

    void timerCallback(const ros::TimerEvent&) {
        if (has_goal_) {
            publish(ros::Time::now());
        }
    }

    void publish(const ros::Time& stamp) {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = latest_goal_.header.frame_id.empty()
                                   ? world_frame_
                                   : latest_goal_.header.frame_id;
        odom.child_frame_id = "planning_goal";
        odom.pose.pose = latest_goal_.pose;
        odom_pub_.publish(odom);
    }

    ros::NodeHandle pnh_;
    std::string world_frame_ = "world";
    double publish_rate_ = 20.0;
    bool has_goal_ = false;
    geometry_msgs::PoseStamped latest_goal_;
    ros::Subscriber goal_sub_;
    ros::Publisher odom_pub_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_goal_to_odometry_node");
    PlanningGoalToOdometryNode node;
    ros::spin();
    return 0;
}
