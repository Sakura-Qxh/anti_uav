#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class TargetMotionNode {
public:
    TargetMotionNode() : nh_(), pnh_("~") {
        loadParams();
        createRosInterfaces();

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(publish_rate_, 1e-3)),
            &TargetMotionNode::timerCallback,
            this);

        ROS_INFO("target_motion_node started as attack-goal publisher only");
    }

private:
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);
        pnh_.param<std::string>("control_mode", control_mode_, "rviz_click");
        pnh_.param<std::string>("clicked_point_topic", clicked_point_topic_, "/clicked_point");
        pnh_.param<std::string>("output_goal_topic", output_goal_topic_, "/target_scenario/attack_goal");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");
        pnh_.param<int>("target_count", target_count_, 1);
        pnh_.param<bool>("publish_per_target_goals", publish_per_target_goals_, true);
        pnh_.param<double>("target_goal_altitude", target_goal_altitude_, 8.0);

        std::vector<double> default_goal;
        if (pnh_.getParam("default_attack_goal", default_goal) &&
            default_goal.size() >= 3) {
            attack_goal_.x = default_goal[0];
            attack_goal_.y = default_goal[1];
            attack_goal_.z = default_goal[2];
        } else {
            attack_goal_.x = 0.0;
            attack_goal_.y = 0.0;
            attack_goal_.z = target_goal_altitude_;
        }
        if (target_count_ <= 0) {
            target_count_ = 1;
        }
    }

    void createRosInterfaces() {
        clicked_point_sub_ = nh_.subscribe(
            clicked_point_topic_,
            10,
            &TargetMotionNode::clickedPointCallback,
            this);

        attack_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            output_goal_topic_,
            1,
            true);

        if (publish_per_target_goals_) {
            for (int i = 0; i < target_count_; ++i) {
                const std::string topic =
                    target_prefix_ + std::to_string(i) + "/planning_goal";
                target_goal_pubs_.push_back(
                    nh_.advertise<geometry_msgs::PoseStamped>(topic, 1, true));
            }
        }

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "/target_scenario/attack_goal_marker",
            1,
            true);
    }

    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (control_mode_ != "rviz_click") {
            return;
        }

        attack_goal_.x = msg->point.x;
        attack_goal_.y = msg->point.y;
        attack_goal_.z = msg->point.z > 0.1 ? msg->point.z : target_goal_altitude_;

        ROS_INFO("attack goal updated: %.2f %.2f %.2f",
                 attack_goal_.x,
                 attack_goal_.y,
                 attack_goal_.z);

        publishAttackGoal(ros::Time::now());
    }

    geometry_msgs::PoseStamped makeAttackGoal(const ros::Time& stamp) const {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = stamp;
        goal.header.frame_id = world_frame_;
        goal.pose.position = attack_goal_;
        goal.pose.orientation.w = 1.0;
        return goal;
    }

    geometry_msgs::PoseStamped makeTargetGoal(int target_index,
                                              const ros::Time& stamp) const {
        (void)target_index;
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = stamp;
        goal.header.frame_id = world_frame_;
        goal.pose.position = attack_goal_;
        goal.pose.orientation.w = 1.0;
        return goal;
    }

    visualization_msgs::Marker makeAttackGoalMarker(const ros::Time& stamp) const {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = world_frame_;
        marker.ns = "attack_goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = attack_goal_;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 2.0;
        marker.color.r = 1.0;
        marker.color.g = 0.9;
        marker.color.b = 0.0;
        marker.color.a = 0.9;
        return marker;
    }

    void publishAttackGoal(const ros::Time& stamp) {
        attack_goal_pub_.publish(makeAttackGoal(stamp));
        for (int i = 0; i < static_cast<int>(target_goal_pubs_.size()); ++i) {
            target_goal_pubs_[i].publish(makeTargetGoal(i, stamp));
        }
        marker_pub_.publish(makeAttackGoalMarker(stamp));
    }

    void timerCallback(const ros::TimerEvent&) {
        publishAttackGoal(ros::Time::now());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    std::string world_frame_ = "world";
    double publish_rate_ = 5.0;
    std::string control_mode_ = "rviz_click";
    std::string clicked_point_topic_ = "/clicked_point";
    std::string output_goal_topic_ = "/target_scenario/attack_goal";
    std::string target_prefix_ = "/target_";
    int target_count_ = 1;
    bool publish_per_target_goals_ = true;
    double target_goal_altitude_ = 8.0;

    geometry_msgs::Point attack_goal_;

    ros::Subscriber clicked_point_sub_;
    ros::Publisher attack_goal_pub_;
    std::vector<ros::Publisher> target_goal_pubs_;
    ros::Publisher marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_motion_node");
    TargetMotionNode node;
    ros::spin();
    return 0;
}
