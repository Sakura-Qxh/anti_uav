#include <algorithm>
#include <cmath>
#include <map>
#include <regex>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double norm2d(double x, double y) {
    return std::sqrt(x * x + y * y);
}

struct TargetState {
    nav_msgs::Odometry estimate;
    ros::Time stamp;
    bool has_estimate = false;
};

}  // namespace

class CooperativeGuidanceNode {
public:
    CooperativeGuidanceNode() : nh_(), pnh_("~") {
        loadParams();

        targets_.resize(target_count_);
        assignments_.resize(interceptor_count_, -1);

        createRosInterfaces();

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(update_rate_, 1e-3)),
            &CooperativeGuidanceNode::timerCallback,
            this);

        ROS_INFO("cooperative_guidance started with %d targets and %d interceptors",
                 target_count_,
                 interceptor_count_);
    }

private:
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 6);
        pnh_.param<int>("interceptor_count", interceptor_count_, 6);
        pnh_.param<double>("update_rate", update_rate_, 10.0);
        pnh_.param<std::string>("assignment_topic", assignment_topic_, "/assignment/result");
        pnh_.param<std::string>("goal_prefix", goal_prefix_, "/interceptor_");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");
        pnh_.param<double>("stale_timeout", stale_timeout_, 2.0);
        pnh_.param<double>("capture_radius", capture_radius_, 8.0);
        pnh_.param<double>("vertical_offset", vertical_offset_, 0.0);
        pnh_.param<double>("lead_time", lead_time_, 1.0);
        pnh_.param<double>("min_goal_altitude", min_goal_altitude_, 2.0);
        pnh_.param<double>("marker_scale", marker_scale_, 0.8);
        pnh_.param<double>("line_width", line_width_, 0.12);
    }

    void createRosInterfaces() {
        assignment_sub_ = nh_.subscribe(assignment_topic_,
                                        10,
                                        &CooperativeGuidanceNode::assignmentCallback,
                                        this);

        for (int i = 0; i < target_count_; ++i) {
            const std::string estimate_topic =
                target_prefix_ + std::to_string(i) + "/estimate";

            target_subs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(
                    estimate_topic,
                    10,
                    [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                        targetEstimateCallback(msg, i);
                    }));
        }

        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string capture_goal_topic =
                goal_prefix_ + std::to_string(i) + "/capture_goal";

            capture_goal_pubs_.push_back(
                nh_.advertise<geometry_msgs::PoseStamped>(capture_goal_topic, 1, true));
        }

        marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/cooperative_guidance/markers", 1);
    }

    void targetEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= target_count_) {
            return;
        }

        targets_[target_index].estimate = *msg;
        targets_[target_index].stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        targets_[target_index].has_estimate = true;
    }

    void assignmentCallback(const std_msgs::String::ConstPtr& msg) {
        std::fill(assignments_.begin(), assignments_.end(), -1);
        grouped_assignments_.clear();

        const std::regex block_regex("target_([0-9]+)\\([^)]*\\)\\s*<-\\s*([^|]+)");
        const std::regex interceptor_regex("interceptor_([0-9]+)");

        auto block_begin =
            std::sregex_iterator(msg->data.begin(), msg->data.end(), block_regex);
        const auto block_end = std::sregex_iterator();

        for (auto block_it = block_begin; block_it != block_end; ++block_it) {
            const int target_id = std::stoi((*block_it)[1].str());
            const std::string interceptor_list = (*block_it)[2].str();

            auto interceptor_begin =
                std::sregex_iterator(interceptor_list.begin(),
                                     interceptor_list.end(),
                                     interceptor_regex);
            const auto interceptor_end = std::sregex_iterator();

            for (auto interceptor_it = interceptor_begin;
                 interceptor_it != interceptor_end;
                 ++interceptor_it) {
                const int interceptor_id = std::stoi((*interceptor_it)[1].str());

                if (target_id < 0 || target_id >= target_count_ ||
                    interceptor_id < 0 || interceptor_id >= interceptor_count_) {
                    continue;
                }

                assignments_[interceptor_id] = target_id;
                grouped_assignments_[target_id].push_back(interceptor_id);
            }
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();

        visualization_msgs::MarkerArray markers;
        int marker_id = 0;

        for (const auto& item : grouped_assignments_) {
            const int target_id = item.first;
            const std::vector<int>& interceptors = item.second;

            if (!isTargetFresh(target_id, now) || interceptors.empty()) {
                continue;
            }

            for (size_t slot = 0; slot < interceptors.size(); ++slot) {
                const int interceptor_id = interceptors[slot];

                geometry_msgs::PoseStamped capture_goal =
                    makeCaptureGoal(target_id,
                                    static_cast<int>(slot),
                                    static_cast<int>(interceptors.size()),
                                    now);

                capture_goal_pubs_[interceptor_id].publish(capture_goal);
                addGoalMarkers(markers,
                               marker_id,
                               interceptor_id,
                               target_id,
                               capture_goal);
            }
        }

        marker_pub_.publish(markers);
    }

    bool isTargetFresh(int target_id, const ros::Time& now) const {
        if (target_id < 0 || target_id >= target_count_) {
            return false;
        }

        const TargetState& target = targets_[target_id];

        if (!target.has_estimate) {
            return false;
        }

        return (now - target.stamp).toSec() <= stale_timeout_;
    }

    geometry_msgs::PoseStamped makeCaptureGoal(int target_id,
                                               int slot,
                                               int slot_count,
                                               const ros::Time& stamp) const {
        const nav_msgs::Odometry& estimate = targets_[target_id].estimate;
        const geometry_msgs::Point& target_pos = estimate.pose.pose.position;
        const geometry_msgs::Vector3& target_vel = estimate.twist.twist.linear;

        double heading = std::atan2(target_vel.y, target_vel.x);
        if (norm2d(target_vel.x, target_vel.y) < 1e-3) {
            heading = 0.0;
        }

        const double angle =
            heading + kPi +
            2.0 * kPi * static_cast<double>(slot) / std::max(1, slot_count);

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = stamp;
        goal.header.frame_id = world_frame_;

        goal.pose.position.x =
            target_pos.x + lead_time_ * target_vel.x + capture_radius_ * std::cos(angle);
        goal.pose.position.y =
            target_pos.y + lead_time_ * target_vel.y + capture_radius_ * std::sin(angle);
        goal.pose.position.z =
            std::max(min_goal_altitude_, target_pos.z + vertical_offset_);

        goal.pose.orientation.w = 1.0;
        return goal;
    }

    void addGoalMarkers(visualization_msgs::MarkerArray& markers,
                        int& marker_id,
                        int interceptor_id,
                        int target_id,
                        const geometry_msgs::PoseStamped& goal) const {
        visualization_msgs::Marker sphere;
        sphere.header = goal.header;
        sphere.ns = "interceptor_capture_goals";
        sphere.id = marker_id++;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose = goal.pose;
        sphere.scale.x = marker_scale_;
        sphere.scale.y = marker_scale_;
        sphere.scale.z = marker_scale_;
        sphere.color.r = 0.1;
        sphere.color.g = 0.9;
        sphere.color.b = 1.0;
        sphere.color.a = 0.9;
        markers.markers.push_back(sphere);

        visualization_msgs::Marker line;
        line.header = goal.header;
        line.ns = "guidance_target_lines";
        line.id = marker_id++;
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = line_width_;
        line.color.r = 0.2;
        line.color.g = 0.8;
        line.color.b = 1.0;
        line.color.a = 0.7;
        line.points.push_back(goal.pose.position);
        line.points.push_back(targets_[target_id].estimate.pose.pose.position);
        markers.markers.push_back(line);

        visualization_msgs::Marker text;
        text.header = goal.header;
        text.ns = "guidance_goal_text";
        text.id = marker_id++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose = goal.pose;
        text.pose.position.z += 1.0;
        text.scale.z = 0.9;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 0.95;
        text.text = "I" + std::to_string(interceptor_id) +
                    " -> T" + std::to_string(target_id);
        markers.markers.push_back(text);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    std::string world_frame_ = "world";
    int target_count_ = 6;
    int interceptor_count_ = 6;
    double update_rate_ = 10.0;

    std::string assignment_topic_ = "/assignment/result";
    std::string goal_prefix_ = "/interceptor_";
    std::string target_prefix_ = "/target_";

    double stale_timeout_ = 2.0;
    double capture_radius_ = 8.0;
    double vertical_offset_ = 0.0;
    double lead_time_ = 1.0;
    double min_goal_altitude_ = 2.0;
    double marker_scale_ = 0.8;
    double line_width_ = 0.12;

    std::vector<TargetState> targets_;
    std::vector<int> assignments_;
    std::map<int, std::vector<int>> grouped_assignments_;

    ros::Subscriber assignment_sub_;
    std::vector<ros::Subscriber> target_subs_;
    std::vector<ros::Publisher> capture_goal_pubs_;

    ros::Publisher marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cooperative_guidance_node");
    CooperativeGuidanceNode node;
    ros::spin();
    return 0;
}