#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct TargetState {
    nav_msgs::Odometry odom;
    ros::Time stamp;
    bool valid = false;
};

double norm2d(double x, double y) {
    return std::sqrt(x * x + y * y);
}

}  // namespace

class MissionManagerNode {
public:
    MissionManagerNode() : nh_(), pnh_("~") {
        loadParams();
        target_states_.resize(target_count_);
        last_target_goals_.resize(target_count_);
        last_target_goal_times_.resize(target_count_);
        has_published_target_goal_.assign(target_count_, false);
        createTargetInterfaces();
        createInterceptorInterfaces();

        attack_goal_sub_ = nh_.subscribe(
            attack_goal_topic_, 1, &MissionManagerNode::attackGoalCallback, this);

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/mission_manager/target_goal_markers", 1, true);

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(1e-3, publish_rate_)),
            &MissionManagerNode::timerCallback,
            this);

        latest_attack_goal_.header.frame_id = world_frame_;
        latest_attack_goal_.pose.position.x = 24.0;
        latest_attack_goal_.pose.position.y = 0.0;
        latest_attack_goal_.pose.position.z = 2.0;
        latest_attack_goal_.pose.orientation.w = 1.0;
        has_goal_ = true;

        ROS_INFO("mission_manager started: targets=%d interceptors=%d",
                 target_count_, interceptor_count_);
    }

private:
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<int>("interceptor_count", interceptor_count_, 4);
        pnh_.param<double>("publish_rate", publish_rate_, 5.0);
        pnh_.param<double>("formation_radius", formation_radius_, 3.0);
        pnh_.param<double>("formation_lookahead", formation_lookahead_, 8.0);
        pnh_.param<double>("formation_heading_lock_distance",
                           formation_heading_lock_distance_, 5.0);
        pnh_.param<double>("odom_timeout", odom_timeout_, 1.0);
        pnh_.param<double>("goal_publish_rate", goal_publish_rate_, 2.0);
        pnh_.param<double>("goal_position_threshold", goal_position_threshold_, 0.3);
        pnh_.param<bool>("spread_target_goals", spread_target_goals_, true);
        pnh_.param<bool>("latch_waypoints", latch_waypoints_, true);
        pnh_.param<std::string>("attack_goal_topic", attack_goal_topic_, "/attack_goal");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");
        pnh_.param<std::string>("interceptor_prefix", interceptor_prefix_, "/interceptor_");

        target_count_ = std::max(1, target_count_);
        interceptor_count_ = std::max(1, interceptor_count_);
        formation_heading_lock_distance_ = std::max(0.1, formation_heading_lock_distance_);
        goal_publish_rate_ = std::max(0.1, goal_publish_rate_);
        goal_position_threshold_ = std::max(0.0, goal_position_threshold_);
    }

    void createTargetInterfaces() {
        target_goal_pubs_.resize(target_count_);
        target_waypoint_pubs_.resize(target_count_);

        for (int i = 0; i < target_count_; ++i) {
            const std::string ns = target_prefix_ + std::to_string(i);
            target_goal_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
                ns + "/planning_goal", 1, true);
            target_waypoint_pubs_[i] = nh_.advertise<nav_msgs::Path>(
                ns + "/waypoint_generator/waypoints", 1, latch_waypoints_);
            target_odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                ns + "/odom",
                10,
                [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    targetOdomCallback(msg, i);
                }));
        }
    }

    void createInterceptorInterfaces() {
        interceptor_goal_pubs_.resize(interceptor_count_);
        interceptor_waypoint_pubs_.resize(interceptor_count_);

        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string ns = interceptor_prefix_ + std::to_string(i);
            interceptor_goal_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(
                ns + "/planning_goal", 1, true);
            interceptor_waypoint_pubs_[i] = nh_.advertise<nav_msgs::Path>(
                ns + "/waypoint_generator/waypoints", 1, latch_waypoints_);
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

    void targetOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, int index) {
        if (index < 0 || index >= target_count_) {
            return;
        }
        target_states_[index].odom = *msg;
        target_states_[index].stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        target_states_[index].valid = true;
    }

    void attackGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        geometry_msgs::PoseStamped incoming = *msg;
        if (incoming.header.frame_id.empty()) {
            incoming.header.frame_id = world_frame_;
        }
        if (incoming.pose.position.z < 0.1) {
            incoming.pose.position.z = 2.0;
        }
        incoming.pose.orientation.w = 1.0;

        if (has_goal_) {
            const double dx = incoming.pose.position.x - latest_attack_goal_.pose.position.x;
            const double dy = incoming.pose.position.y - latest_attack_goal_.pose.position.y;
            const double dz = incoming.pose.position.z - latest_attack_goal_.pose.position.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < 0.05 &&
                incoming.header.frame_id == latest_attack_goal_.header.frame_id) {
                return;
            }
        }

        latest_attack_goal_ = incoming;
        formation_heading_locked_ = false;
        has_goal_ = true;
        publishTargetGoals(true);
    }

    void timerCallback(const ros::TimerEvent&) {
        if (has_goal_) {
            publishTargetGoals();
        }
    }

    bool targetOdomFresh(const ros::Time& now) const {
        if (target_count_ != 3) {
            ROS_WARN_THROTTLE(2.0, "moving formation requires target_count=3");
            return false;
        }
        for (const auto& target : target_states_) {
            if (!target.valid || (now - target.stamp).toSec() > odom_timeout_) {
                return false;
            }
        }
        return true;
    }

    std::vector<geometry_msgs::PoseStamped> makeFormationGoals(
        const ros::Time& now) {
        std::vector<geometry_msgs::PoseStamped> goals(target_count_, latest_attack_goal_);

        if (!spread_target_goals_ || !targetOdomFresh(now)) {
            ROS_WARN_THROTTLE(2.0, "target Odom not ready; using final formation slots");
            for (int i = 0; i < target_count_; ++i) {
                const double angle = 2.0 * kPi * static_cast<double>(i) /
                                     static_cast<double>(target_count_);
                goals[i].pose.position.x += formation_radius_ * std::cos(angle);
                goals[i].pose.position.y += formation_radius_ * std::sin(angle);
            }
            return goals;
        }

        geometry_msgs::Point center;
        for (const auto& target : target_states_) {
            center.x += target.odom.pose.pose.position.x;
            center.y += target.odom.pose.pose.position.y;
            center.z += target.odom.pose.pose.position.z;
        }
        center.x /= static_cast<double>(target_count_);
        center.y /= static_cast<double>(target_count_);
        center.z /= static_cast<double>(target_count_);

        const double dx = latest_attack_goal_.pose.position.x - center.x;
        const double dy = latest_attack_goal_.pose.position.y - center.y;
        const double distance = norm2d(dx, dy);

        if (!formation_heading_locked_ && distance > formation_heading_lock_distance_) {
            last_heading_ = std::atan2(dy, dx);
        } else if (!formation_heading_locked_ && distance <= formation_heading_lock_distance_) {
            formation_heading_locked_ = true;
            ROS_INFO("target formation heading locked at %.3f rad", last_heading_);
        }

        const double step = std::min(formation_lookahead_, distance);
        geometry_msgs::Point reference = center;
        if (distance > 1e-3) {
            reference.x += step * dx / distance;
            reference.y += step * dy / distance;
        }
        reference.z = latest_attack_goal_.pose.position.z;

        for (int i = 0; i < target_count_; ++i) {
            const double angle = last_heading_ +
                2.0 * kPi * static_cast<double>(i) /
                static_cast<double>(target_count_);
            goals[i] = latest_attack_goal_;
            goals[i].header.stamp = now;
            goals[i].header.frame_id = world_frame_;
            goals[i].pose.position.x = reference.x + formation_radius_ * std::cos(angle);
            goals[i].pose.position.y = reference.y + formation_radius_ * std::sin(angle);
            goals[i].pose.position.z = reference.z;
            goals[i].pose.orientation.x = 0.0;
            goals[i].pose.orientation.y = 0.0;
            goals[i].pose.orientation.z = 0.0;
            goals[i].pose.orientation.w = 1.0;
        }
        return goals;
    }

    void publishGoalAndWaypoint(const geometry_msgs::PoseStamped& input,
                                ros::Publisher& goal_pub,
                                ros::Publisher& waypoint_pub) {
        geometry_msgs::PoseStamped goal = input;
        goal.header.stamp = ros::Time::now();
        if (goal.header.frame_id.empty()) {
            goal.header.frame_id = world_frame_;
        }
        goal.pose.orientation.w = 1.0;

        nav_msgs::Path path;
        path.header = goal.header;
        path.poses.push_back(goal);
        goal_pub.publish(goal);
        waypoint_pub.publish(path);
    }

    bool shouldPublishTargetGoal(int index,
                                 const geometry_msgs::PoseStamped& goal,
                                 const ros::Time& now,
                                 bool force) const {
        if (force || !has_published_target_goal_[index]) return true;
        if ((now - last_target_goal_times_[index]).toSec() < 1.0 / goal_publish_rate_) {
            return false;
        }
        const double dx = goal.pose.position.x - last_target_goals_[index].pose.position.x;
        const double dy = goal.pose.position.y - last_target_goals_[index].pose.position.y;
        const double dz = goal.pose.position.z - last_target_goals_[index].pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) >= goal_position_threshold_;
    }

    void publishTargetGoals(bool force = false) {
        const ros::Time now = ros::Time::now();
        const auto goals = makeFormationGoals(now);
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            if (shouldPublishTargetGoal(i, goals[i], now, force)) {
                publishGoalAndWaypoint(
                    goals[i], target_goal_pubs_[i], target_waypoint_pubs_[i]);
                last_target_goals_[i] = goals[i];
                last_target_goal_times_[i] = now;
                has_published_target_goal_[i] = true;
            }

            visualization_msgs::Marker marker;
            marker.header = goals[i].header;
            marker.ns = "target_formation_goals";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = goals[i].pose;
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
    ros::Timer timer_;

    std::string world_frame_ = "world";
    int target_count_ = 3;
    int interceptor_count_ = 4;
    double publish_rate_ = 5.0;
    double formation_radius_ = 3.0;
    double formation_lookahead_ = 8.0;
    double formation_heading_lock_distance_ = 5.0;
    double odom_timeout_ = 1.0;
    double goal_publish_rate_ = 2.0;
    double goal_position_threshold_ = 0.3;
    bool spread_target_goals_ = true;
    bool latch_waypoints_ = true;
    double last_heading_ = 0.0;
    bool formation_heading_locked_ = false;

    std::string attack_goal_topic_ = "/attack_goal";
    std::string target_prefix_ = "/target_";
    std::string interceptor_prefix_ = "/interceptor_";

    bool has_goal_ = false;
    geometry_msgs::PoseStamped latest_attack_goal_;
    std::vector<TargetState> target_states_;
    std::vector<geometry_msgs::PoseStamped> last_target_goals_;
    std::vector<ros::Time> last_target_goal_times_;
    std::vector<bool> has_published_target_goal_;

    ros::Subscriber attack_goal_sub_;
    std::vector<ros::Subscriber> target_odom_subs_;
    std::vector<ros::Subscriber> interceptor_capture_goal_subs_;
    std::vector<ros::Publisher> target_goal_pubs_;
    std::vector<ros::Publisher> target_waypoint_pubs_;
    std::vector<ros::Publisher> interceptor_goal_pubs_;
    std::vector<ros::Publisher> interceptor_waypoint_pubs_;
    ros::Publisher marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_manager_node");
    MissionManagerNode node;
    ros::spin();
    return 0;
}
