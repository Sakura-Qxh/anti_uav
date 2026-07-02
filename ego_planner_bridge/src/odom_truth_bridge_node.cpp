#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class OdomTruthBridge {
public:
    OdomTruthBridge() : pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<std::string>("child_frame_id", child_frame_id_, "");
        pnh_.param<bool>("publish_truth", publish_truth_, false);
        pnh_.param<bool>("latch", latch_, false);

        visual_odom_sub_ = pnh_.subscribe("visual_odom", 10, &OdomTruthBridge::odomCallback, this);
        odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("odom", 10, latch_);
        if (publish_truth_) {
            truth_pub_ = pnh_.advertise<nav_msgs::Odometry>("truth", 10, latch_);
        }

        ROS_INFO("odom_truth_bridge_node started, publish_truth=%s", publish_truth_ ? "true" : "false");
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        nav_msgs::Odometry odom = *msg;
        if (odom.header.frame_id.empty()) {
            odom.header.frame_id = world_frame_;
        }
        if (!child_frame_id_.empty()) {
            odom.child_frame_id = child_frame_id_;
        }

        odom_pub_.publish(odom);
        if (publish_truth_) {
            truth_pub_.publish(odom);
        }
    }

    ros::NodeHandle pnh_;
    std::string world_frame_;
    std::string child_frame_id_;
    bool publish_truth_ = false;
    bool latch_ = false;
    ros::Subscriber visual_odom_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher truth_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_truth_bridge_node");
    OdomTruthBridge node;
    ros::spin();
    return 0;
}