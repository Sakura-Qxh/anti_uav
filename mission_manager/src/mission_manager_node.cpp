#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class MissionManagerNode {
public:
    MissionManagerNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 6);
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);
        pnh_.param<double>("formation_radius", formation_radius_, 4.0);
        pnh_.param<bool>("spread_target_goals", spread_target_goals_, true);
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_, "/attack_goal");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");

        target_count_ = std::max(1, target_count_);
        goal_pubs_.resize(target_count_);
        for (int i = 0; i < target_count_; ++i) {
            goal_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
                target_prefix_ + std::to_string(i) + "/planning_goal", 1, true);
        }

        attack_goal_sub_ = nh_.subscribe(attack_goal_topic_, 1, &MissionManagerNode::attackGoalCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/mission_manager/target_goal_markers", 1, true);
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
                                &MissionManagerNode::timerCallback, this);

        latest_attack_goal_.header.frame_id = world_frame_;
        latest_attack_goal_.pose.orientation.w = 1.0;
        latest_attack_goal_.pose.position.z = 2.0;
        has_goal_ = true;
    }

private:
    void attackGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        latest_attack_goal_ = *msg;
        if (latest_attack_goal_.header.frame_id.empty()) {
            latest_attack_goal_.header.frame_id = world_frame_;
        }
        has_goal_ = true;
        publishGoals();
    }

    void timerCallback(const ros::TimerEvent&) {
        if (has_goal_) {
            publishGoals();
        }
    }

    geometry_msgs::PoseStamped makeTargetGoal(int index) const {
        geometry_msgs::PoseStamped goal = latest_attack_goal_;
        goal.header.stamp = ros::Time::now();
        if (spread_target_goals_ && target_count_ > 1) {
            const double angle = 2.0 * M_PI * static_cast<double>(index) / static_cast<double>(target_count_);
            goal.pose.position.x += formation_radius_ * std::cos(angle);
            goal.pose.position.y += formation_radius_ * std::sin(angle);
        }
        goal.pose.orientation.w = 1.0;
        return goal;
    }

    void publishGoals() {
        visualization_msgs::MarkerArray markers;
        for (int i = 0; i < target_count_; ++i) {
            const geometry_msgs::PoseStamped goal = makeTargetGoal(i);
            goal_pubs_[i].publish(goal);

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
    double publish_rate_ = 5.0;
    double formation_radius_ = 4.0;
    bool spread_target_goals_ = true;
    std::string attack_goal_topic_;
    std::string target_prefix_;
    bool has_goal_ = false;
    geometry_msgs::PoseStamped latest_attack_goal_;
    ros::Subscriber attack_goal_sub_;
    std::vector<ros::Publisher> goal_pubs_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_manager_node");
    MissionManagerNode node;
    ros::spin();
    return 0;
}