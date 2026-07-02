#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class TargetAttackGoalNode {
public:
    TargetAttackGoalNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<std::string>("clicked_point_topic", clicked_point_topic_, "/clicked_point");
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_, "/attack_goal");
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);
        pnh_.param<double>("default_altitude", default_altitude_, 2.0);

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
        attack_goal_.pose.orientation.w = 1.0;
        attack_goal_.header.frame_id = world_frame_;

        clicked_sub_ = nh_.subscribe(clicked_point_topic_, 10, &TargetAttackGoalNode::clickedCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(attack_goal_topic_, 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/attack_goal_marker", 1, true);
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
                                &TargetAttackGoalNode::timerCallback, this);
    }

private:
    void clickedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        attack_goal_.header.stamp = ros::Time::now();
        attack_goal_.header.frame_id = msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id;
        attack_goal_.pose.position.x = msg->point.x;
        attack_goal_.pose.position.y = msg->point.y;
        attack_goal_.pose.position.z = msg->point.z > 0.1 ? msg->point.z : default_altitude_;
        attack_goal_.pose.orientation.w = 1.0;
        publish();
    }

    void timerCallback(const ros::TimerEvent&) {
        publish();
    }

    void publish() {
        attack_goal_.header.stamp = ros::Time::now();
        goal_pub_.publish(attack_goal_);

        visualization_msgs::Marker marker;
        marker.header = attack_goal_.header;
        marker.ns = "attack_goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = attack_goal_.pose;
        marker.scale.x = 1.8;
        marker.scale.y = 1.8;
        marker.scale.z = 1.8;
        marker.color.r = 1.0;
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 0.9;
        marker_pub_.publish(marker);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string world_frame_;
    std::string clicked_point_topic_;
    std::string attack_goal_topic_;
    double publish_rate_ = 5.0;
    double default_altitude_ = 2.0;
    geometry_msgs::PoseStamped attack_goal_;
    ros::Subscriber clicked_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_attack_goal_node");
    TargetAttackGoalNode node;
    ros::spin();
    return 0;
}