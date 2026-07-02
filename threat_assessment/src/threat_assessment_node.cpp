#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double norm3(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
}

struct TargetThreatState {
    nav_msgs::Odometry estimate;
    nav_msgs::Path prediction;
    ros::Time last_estimate_time;
    bool has_estimate = false;
    bool has_prediction = false;
    double threat = 0.0;
    double distance_score = 0.0;
    double speed_score = 0.0;
    double approach_score = 0.0;
    double altitude_score = 0.0;
    double time_to_go_score = 0.0;
};

struct RankedTarget {
    int id = 0;
    double threat = 0.0;
};

class ThreatAssessmentNode {
public:
    ThreatAssessmentNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 6);
        pnh_.param<double>("update_rate", update_rate_, 10.0);
        pnh_.param<double>("protected_radius", protected_radius_, 5.0);
        pnh_.param<double>("max_assessment_distance", max_assessment_distance_, 120.0);
        pnh_.param<double>("max_target_speed", max_target_speed_, 45.0);
        pnh_.param<double>("max_approach_speed", max_approach_speed_, 45.0);
        pnh_.param<double>("safe_altitude", safe_altitude_, 25.0);
        pnh_.param<double>("critical_altitude", critical_altitude_, 3.0);
        pnh_.param<double>("max_time_to_go", max_time_to_go_, 20.0);
        pnh_.param<double>("weight_distance", weight_distance_, 0.25);
        pnh_.param<double>("weight_speed", weight_speed_, 0.20);
        pnh_.param<double>("weight_approach", weight_approach_, 0.25);
        pnh_.param<double>("weight_altitude", weight_altitude_, 0.15);
        pnh_.param<double>("weight_time_to_go", weight_time_to_go_, 0.15);
        pnh_.param<double>("stale_timeout", stale_timeout_, 2.0);
        pnh_.param<double>("text_height", text_height_, 1.2);
        pnh_.param<double>("marker_scale", marker_scale_, 0.8);

        std::vector<double> protected_center;
        if (pnh_.getParam("protected_center", protected_center) && protected_center.size() >= 3) {
            protected_center_.x = protected_center[0];
            protected_center_.y = protected_center[1];
            protected_center_.z = protected_center[2];
        } else {
            protected_center_.x = 0.0;
            protected_center_.y = 0.0;
            protected_center_.z = 0.0;
        }

        normalizeWeights();

        targets_.resize(target_count_);
        createRosInterfaces();

        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_),
                                &ThreatAssessmentNode::timerCallback,
                                this);

        ROS_INFO("threat_assessment started for %d targets", target_count_);
    }

private:
    void normalizeWeights() {
        const double sum = weight_distance_ + weight_speed_ + weight_approach_ +
                           weight_altitude_ + weight_time_to_go_;
        if (sum <= 1e-6) {
            weight_distance_ = 0.25;
            weight_speed_ = 0.20;
            weight_approach_ = 0.25;
            weight_altitude_ = 0.15;
            weight_time_to_go_ = 0.15;
            return;
        }

        weight_distance_ /= sum;
        weight_speed_ /= sum;
        weight_approach_ /= sum;
        weight_altitude_ /= sum;
        weight_time_to_go_ /= sum;
    }

    void createRosInterfaces() {
        estimate_subs_.clear();
        prediction_subs_.clear();
        threat_pubs_.clear();

        for (int i = 0; i < target_count_; ++i) {
            const std::string estimate_topic = "/target_" + std::to_string(i) + "/estimate";
            const std::string prediction_topic = "/target_" + std::to_string(i) + "/prediction";
            const std::string threat_topic = "/threat/target_" + std::to_string(i);

            estimate_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                estimate_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    estimateCallback(msg, i);
                }));

            prediction_subs_.push_back(nh_.subscribe<nav_msgs::Path>(
                prediction_topic, 10, [this, i](const nav_msgs::Path::ConstPtr& msg) {
                    predictionCallback(msg, i);
                }));

            threat_pubs_.push_back(nh_.advertise<std_msgs::Float64>(threat_topic, 10));
        }

        ranking_pub_ = nh_.advertise<std_msgs::String>("/threat/ranking", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/threat/markers", 1);
    }

    void estimateCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= static_cast<int>(targets_.size())) {
            return;
        }

        targets_[target_index].estimate = *msg;
        targets_[target_index].last_estimate_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        targets_[target_index].has_estimate = true;
    }

    void predictionCallback(const nav_msgs::Path::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= static_cast<int>(targets_.size())) {
            return;
        }

        targets_[target_index].prediction = *msg;
        targets_[target_index].has_prediction = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        std::vector<RankedTarget> ranking;

        for (int i = 0; i < target_count_; ++i) {
            TargetThreatState& target = targets_[i];
            if (!isTargetFresh(target, now)) {
                target.threat = 0.0;
                publishThreat(i, target.threat);
                continue;
            }

            computeThreat(target);
            publishThreat(i, target.threat);
            ranking.push_back({i, target.threat});
        }

        std::sort(ranking.begin(), ranking.end(), [](const RankedTarget& a, const RankedTarget& b) {
            return a.threat > b.threat;
        });

        publishRanking(ranking);
        publishMarkers(now, ranking);
    }

    bool isTargetFresh(const TargetThreatState& target, const ros::Time& now) const {
        if (!target.has_estimate) {
            return false;
        }
        return (now - target.last_estimate_time).toSec() <= stale_timeout_;
    }

    void computeThreat(TargetThreatState& target) {
        const auto& pos = target.estimate.pose.pose.position;
        const auto& vel = target.estimate.twist.twist.linear;

        const double rx = pos.x - protected_center_.x;
        const double ry = pos.y - protected_center_.y;
        const double rz = pos.z - protected_center_.z;
        const double distance_to_center = norm3(rx, ry, rz);
        const double distance_to_zone = std::max(0.0, distance_to_center - protected_radius_);

        const double speed = norm3(vel.x, vel.y, vel.z);

        double approach_speed = 0.0;
        if (distance_to_center > 1e-6) {
            const double radial_velocity = (rx * vel.x + ry * vel.y + rz * vel.z) / distance_to_center;
            approach_speed = std::max(0.0, -radial_velocity);
        }

        const double time_to_go = approach_speed > 1e-3
                                      ? distance_to_zone / approach_speed
                                      : max_time_to_go_;

        const double altitude = pos.z;
        const double descending_speed = std::max(0.0, -vel.z);

        target.distance_score = clamp01(1.0 - distance_to_zone / max_assessment_distance_);
        target.speed_score = clamp01(speed / max_target_speed_);
        target.approach_score = clamp01(approach_speed / max_approach_speed_);

        const double low_altitude_score = clamp01((safe_altitude_ - altitude) /
                                                  std::max(1e-6, safe_altitude_ - critical_altitude_));
        const double descending_score = clamp01(descending_speed / std::max(1.0, max_target_speed_));
        target.altitude_score = clamp01(0.75 * low_altitude_score + 0.25 * descending_score);

        target.time_to_go_score = clamp01(1.0 - time_to_go / max_time_to_go_);

        target.threat = clamp01(weight_distance_ * target.distance_score +
                                weight_speed_ * target.speed_score +
                                weight_approach_ * target.approach_score +
                                weight_altitude_ * target.altitude_score +
                                weight_time_to_go_ * target.time_to_go_score);
    }

    void publishThreat(int target_index, double threat) {
        std_msgs::Float64 msg;
        msg.data = threat;
        threat_pubs_[target_index].publish(msg);
    }

    void publishRanking(const std::vector<RankedTarget>& ranking) {
        std_msgs::String msg;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        for (size_t i = 0; i < ranking.size(); ++i) {
            if (i > 0) {
                oss << " > ";
            }
            oss << "target_" << ranking[i].id << "(" << ranking[i].threat << ")";
        }
        msg.data = oss.str();
        ranking_pub_.publish(msg);
    }

    void publishMarkers(const ros::Time& stamp, const std::vector<RankedTarget>& ranking) {
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            const TargetThreatState& target = targets_[i];
            if (!target.has_estimate) {
                continue;
            }

            const auto& pos = target.estimate.pose.pose.position;
            const double threat = target.threat;

            visualization_msgs::Marker sphere;
            sphere.header.stamp = stamp;
            sphere.header.frame_id = world_frame_;
            sphere.ns = "threat_score_spheres";
            sphere.id = i;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position = pos;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = marker_scale_ * (0.7 + threat);
            sphere.scale.y = marker_scale_ * (0.7 + threat);
            sphere.scale.z = marker_scale_ * (0.7 + threat);
            sphere.color.a = 0.85;
            sphere.color.r = threat;
            sphere.color.g = 1.0 - threat;
            sphere.color.b = 0.15;
            markers.markers.push_back(sphere);

            visualization_msgs::Marker text;
            text.header.stamp = stamp;
            text.header.frame_id = world_frame_;
            text.ns = "threat_score_text";
            text.id = i;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position.x = pos.x;
            text.pose.position.y = pos.y;
            text.pose.position.z = pos.z + text_height_;
            text.pose.orientation.w = 1.0;
            text.scale.z = text_height_;
            text.color.a = 1.0;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;

            std::ostringstream label;
            label << std::fixed << std::setprecision(2)
                  << "T" << i << " threat=" << threat;
            text.text = label.str();
            markers.markers.push_back(text);
        }

        for (size_t rank = 0; rank < ranking.size(); ++rank) {
            const int target_id = ranking[rank].id;
            if (target_id < 0 || target_id >= target_count_ || !targets_[target_id].has_estimate) {
                continue;
            }

            const auto& pos = targets_[target_id].estimate.pose.pose.position;
            visualization_msgs::Marker rank_text;
            rank_text.header.stamp = stamp;
            rank_text.header.frame_id = world_frame_;
            rank_text.ns = "threat_rank_text";
            rank_text.id = static_cast<int>(rank);
            rank_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            rank_text.action = visualization_msgs::Marker::ADD;
            rank_text.pose.position.x = pos.x;
            rank_text.pose.position.y = pos.y;
            rank_text.pose.position.z = pos.z + 2.0 * text_height_;
            rank_text.pose.orientation.w = 1.0;
            rank_text.scale.z = 0.9 * text_height_;
            rank_text.color.a = 1.0;
            rank_text.color.r = 1.0;
            rank_text.color.g = 0.8;
            rank_text.color.b = 0.1;

            std::ostringstream label;
            label << "rank " << (rank + 1);
            rank_text.text = label.str();
            markers.markers.push_back(rank_text);
        }

        marker_pub_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    std::string world_frame_;
    int target_count_ = 6;
    double update_rate_ = 10.0;
    geometry_msgs::Point protected_center_;
    double protected_radius_ = 5.0;
    double max_assessment_distance_ = 120.0;
    double max_target_speed_ = 45.0;
    double max_approach_speed_ = 45.0;
    double safe_altitude_ = 25.0;
    double critical_altitude_ = 3.0;
    double max_time_to_go_ = 20.0;
    double weight_distance_ = 0.25;
    double weight_speed_ = 0.20;
    double weight_approach_ = 0.25;
    double weight_altitude_ = 0.15;
    double weight_time_to_go_ = 0.15;
    double stale_timeout_ = 2.0;
    double text_height_ = 1.2;
    double marker_scale_ = 0.8;

    std::vector<TargetThreatState> targets_;
    std::vector<ros::Subscriber> estimate_subs_;
    std::vector<ros::Subscriber> prediction_subs_;
    std::vector<ros::Publisher> threat_pubs_;
    ros::Publisher ranking_pub_;
    ros::Publisher marker_pub_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "threat_assessment_node");
    ThreatAssessmentNode node;
    ros::spin();
    return 0;
}