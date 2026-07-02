#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

double distance3(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

struct TargetState {
    nav_msgs::Odometry estimate;
    ros::Time last_estimate_time;
    ros::Time last_threat_time;
    bool has_estimate = false;
    bool has_threat = false;
    double threat = 0.0;
    std::vector<int> assigned_interceptors;
};

struct InterceptorState {
    int id = 0;
    geometry_msgs::Point position;
    ros::Time last_odom_time;
    bool has_odom = false;
    bool assigned = false;
    int assigned_target = -1;
};

struct TargetOrder {
    int id = 0;
    double threat = 0.0;
};

struct Candidate {
    int interceptor_id = -1;
    double cost = std::numeric_limits<double>::infinity();
};

class CooperativeAssignmentNode {
public:
    CooperativeAssignmentNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 6);
        pnh_.param<int>("interceptor_count", interceptor_count_, 6);
        pnh_.param<double>("update_rate", update_rate_, 5.0);
        pnh_.param<int>("max_interceptors_per_target", max_interceptors_per_target_, 3);
        pnh_.param<double>("high_threat_threshold", high_threat_threshold_, 0.70);
        pnh_.param<double>("medium_threat_threshold", medium_threat_threshold_, 0.40);
        pnh_.param<int>("high_threat_interceptors", high_threat_interceptors_, 3);
        pnh_.param<int>("medium_threat_interceptors", medium_threat_interceptors_, 2);
        pnh_.param<int>("low_threat_interceptors", low_threat_interceptors_, 1);
        pnh_.param<double>("interceptor_speed", interceptor_speed_, 30.0);
        pnh_.param<double>("max_assignment_distance", max_assignment_distance_, 300.0);
        pnh_.param<double>("stale_timeout", stale_timeout_, 2.0);
        pnh_.param<double>("interceptor_odom_timeout", interceptor_odom_timeout_, 2.0);
        pnh_.param<std::string>("interceptor_prefix", interceptor_prefix_, "/interceptor_");
        pnh_.param<double>("cost_distance_weight", cost_distance_weight_, 0.55);
        pnh_.param<double>("cost_time_weight", cost_time_weight_, 0.35);
        pnh_.param<double>("cost_threat_weight", cost_threat_weight_, 0.45);
        pnh_.param<double>("interceptor_marker_scale", interceptor_marker_scale_, 1.0);
        pnh_.param<double>("line_width", line_width_, 0.18);
        pnh_.param<double>("text_height", text_height_, 1.2);

        targets_.resize(target_count_);
        loadInterceptors();
        createRosInterfaces();

        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_),
                                &CooperativeAssignmentNode::timerCallback,
                                this);

        ROS_INFO("cooperative_assignment started with %d targets and %d interceptors",
                 target_count_, interceptor_count_);
    }

private:
    void loadInterceptors() {
        interceptors_.clear();
        for (int i = 0; i < interceptor_count_; ++i) {
            InterceptorState interceptor;
            interceptor.id = i;

            const std::string key = "interceptors/" + std::to_string(i);
            std::vector<double> pos;
            if (pnh_.getParam(key, pos) && pos.size() >= 3) {
                interceptor.position.x = pos[0];
                interceptor.position.y = pos[1];
                interceptor.position.z = pos[2];
            } else {
                interceptor.position.x = -40.0;
                interceptor.position.y = -30.0 + 12.0 * i;
                interceptor.position.z = 6.0;
            }

            interceptors_.push_back(interceptor);
        }
    }

    void createRosInterfaces() {
        estimate_subs_.clear();
        threat_subs_.clear();
        interceptor_odom_subs_.clear();

        for (int i = 0; i < target_count_; ++i) {
            const std::string estimate_topic = "/target_" + std::to_string(i) + "/estimate";
            const std::string threat_topic = "/threat/target_" + std::to_string(i);

            estimate_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                estimate_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    estimateCallback(msg, i);
                }));

            threat_subs_.push_back(nh_.subscribe<std_msgs::Float64>(
                threat_topic, 10, [this, i](const std_msgs::Float64::ConstPtr& msg) {
                    threatCallback(msg, i);
                }));
        }

        ranking_sub_ = nh_.subscribe<std_msgs::String>("/threat/ranking", 10,
                                                       &CooperativeAssignmentNode::rankingCallback,
                                                       this);
        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string odom_topic =
                interceptor_prefix_ + std::to_string(i) + "/odom";
            interceptor_odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                odom_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    interceptorOdomCallback(msg, i);
                }));
        }
        result_pub_ = nh_.advertise<std_msgs::String>("/assignment/result", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/assignment/markers", 1);
    }

    void estimateCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= target_count_) {
            return;
        }
        targets_[target_index].estimate = *msg;
        targets_[target_index].last_estimate_time = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        targets_[target_index].has_estimate = true;
    }

    void threatCallback(const std_msgs::Float64::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= target_count_) {
            return;
        }
        targets_[target_index].threat = clamp01(msg->data);
        targets_[target_index].last_threat_time = ros::Time::now();
        targets_[target_index].has_threat = true;
    }

    void rankingCallback(const std_msgs::String::ConstPtr& msg) {
        latest_ranking_text_ = msg->data;
    }

    void interceptorOdomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                                 int interceptor_index) {
        if (interceptor_index < 0 || interceptor_index >= interceptor_count_) {
            return;
        }
        interceptors_[interceptor_index].position = msg->pose.pose.position;
        interceptors_[interceptor_index].last_odom_time =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        interceptors_[interceptor_index].has_odom = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        clearAssignments();

        std::vector<TargetOrder> ordered_targets = makeOrderedTargets(now);
        for (const auto& target_order : ordered_targets) {
            assignInterceptorsToTarget(target_order.id);
        }

        publishResult(now, ordered_targets);
        publishMarkers(now);
    }

    void clearAssignments() {
        for (auto& target : targets_) {
            target.assigned_interceptors.clear();
        }
        for (auto& interceptor : interceptors_) {
            interceptor.assigned = false;
            interceptor.assigned_target = -1;
        }
    }

    bool isTargetFresh(const TargetState& target, const ros::Time& now) const {
        if (!target.has_estimate || !target.has_threat) {
            return false;
        }
        const double estimate_age = (now - target.last_estimate_time).toSec();
        const double threat_age = (now - target.last_threat_time).toSec();
        return estimate_age <= stale_timeout_ && threat_age <= stale_timeout_;
    }

    std::vector<TargetOrder> makeOrderedTargets(const ros::Time& now) const {
        std::vector<TargetOrder> ordered;
        for (int i = 0; i < target_count_; ++i) {
            if (isTargetFresh(targets_[i], now) && targets_[i].threat > 1e-3) {
                ordered.push_back({i, targets_[i].threat});
            }
        }

        std::sort(ordered.begin(), ordered.end(), [](const TargetOrder& a, const TargetOrder& b) {
            return a.threat > b.threat;
        });
        return ordered;
    }

    int desiredInterceptorCount(double threat) const {
        if (threat >= high_threat_threshold_) {
            return high_threat_interceptors_;
        }
        if (threat >= medium_threat_threshold_) {
            return medium_threat_interceptors_;
        }
        return low_threat_interceptors_;
    }

    void assignInterceptorsToTarget(int target_id) {
        if (target_id < 0 || target_id >= target_count_) {
            return;
        }

        TargetState& target = targets_[target_id];
        const int desired_count = std::min(max_interceptors_per_target_,
                                           desiredInterceptorCount(target.threat));

        std::vector<Candidate> candidates;
        for (const auto& interceptor : interceptors_) {
            if (interceptor.assigned) {
                continue;
            }
            if (!isInterceptorFresh(interceptor, ros::Time::now())) {
                continue;
            }

            const double cost = assignmentCost(interceptor, target);
            if (std::isfinite(cost)) {
                candidates.push_back({interceptor.id, cost});
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.cost < b.cost;
        });

        for (const auto& candidate : candidates) {
            if (static_cast<int>(target.assigned_interceptors.size()) >= desired_count) {
                break;
            }
            if (candidate.interceptor_id < 0 || candidate.interceptor_id >= interceptor_count_) {
                continue;
            }

            InterceptorState& interceptor = interceptors_[candidate.interceptor_id];
            if (interceptor.assigned) {
                continue;
            }

            interceptor.assigned = true;
            interceptor.assigned_target = target_id;
            target.assigned_interceptors.push_back(interceptor.id);
        }
    }

    bool isInterceptorFresh(const InterceptorState& interceptor,
                            const ros::Time& now) const {
        if (!interceptor.has_odom) {
            return true;
        }
        return (now - interceptor.last_odom_time).toSec() <= interceptor_odom_timeout_;
    }

    double assignmentCost(const InterceptorState& interceptor, const TargetState& target) const {
        geometry_msgs::Point target_position = target.estimate.pose.pose.position;
        const double distance = distance3(interceptor.position, target_position);
        if (distance > max_assignment_distance_) {
            return std::numeric_limits<double>::infinity();
        }

        const double time_to_go = distance / std::max(1.0, interceptor_speed_);
        const double distance_score = distance / std::max(1.0, max_assignment_distance_);
        const double time_score = time_to_go / std::max(1.0, max_assignment_distance_ / std::max(1.0, interceptor_speed_));

        return cost_distance_weight_ * distance_score +
               cost_time_weight_ * time_score -
               cost_threat_weight_ * target.threat;
    }

    void publishResult(const ros::Time& stamp, const std::vector<TargetOrder>& ordered_targets) {
        std_msgs::String msg;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "stamp=" << stamp.toSec();

        if (!latest_ranking_text_.empty()) {
            oss << " | threat_ranking=[" << latest_ranking_text_ << "]";
        }

        for (const auto& item : ordered_targets) {
            const TargetState& target = targets_[item.id];
            if (target.assigned_interceptors.empty()) {
                continue;
            }

            oss << " | target_" << item.id << "(" << target.threat << ") <- ";
            for (size_t k = 0; k < target.assigned_interceptors.size(); ++k) {
                if (k > 0) {
                    oss << ",";
                }
                oss << "interceptor_" << target.assigned_interceptors[k];
            }
        }

        msg.data = oss.str();
        result_pub_.publish(msg);
    }

    void publishMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        for (const auto& interceptor : interceptors_) {
            visualization_msgs::Marker cube;
            cube.header.stamp = stamp;
            cube.header.frame_id = world_frame_;
            cube.ns = "assignment_interceptors";
            cube.id = interceptor.id;
            cube.type = visualization_msgs::Marker::CUBE;
            cube.action = visualization_msgs::Marker::ADD;
            cube.pose.position = interceptor.position;
            cube.pose.orientation.w = 1.0;
            cube.scale.x = interceptor_marker_scale_;
            cube.scale.y = interceptor_marker_scale_;
            cube.scale.z = interceptor_marker_scale_;
            cube.color.a = 1.0;
            cube.color.r = 0.1;
            cube.color.g = interceptor.assigned ? 0.7 : 0.3;
            cube.color.b = 1.0;
            markers.markers.push_back(cube);

            visualization_msgs::Marker text;
            text.header.stamp = stamp;
            text.header.frame_id = world_frame_;
            text.ns = "assignment_interceptor_text";
            text.id = interceptor.id;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position = interceptor.position;
            text.pose.position.z += text_height_;
            text.pose.orientation.w = 1.0;
            text.scale.z = text_height_;
            text.color.a = 1.0;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;

            std::ostringstream label;
            label << "I" << interceptor.id;
            if (interceptor.assigned_target >= 0) {
                label << "->T" << interceptor.assigned_target;
            }
            text.text = label.str();
            markers.markers.push_back(text);
        }

        int line_id = 0;
        int label_id = 0;
        for (const auto& interceptor : interceptors_) {
            if (!interceptor.assigned || interceptor.assigned_target < 0 ||
                interceptor.assigned_target >= target_count_ ||
                !targets_[interceptor.assigned_target].has_estimate) {
                continue;
            }

            const geometry_msgs::Point target_position =
                targets_[interceptor.assigned_target].estimate.pose.pose.position;

            visualization_msgs::Marker line;
            line.header.stamp = stamp;
            line.header.frame_id = world_frame_;
            line.ns = "assignment_lines";
            line.id = line_id++;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.scale.x = line_width_;
            line.color.a = 0.95;
            line.color.r = 0.2;
            line.color.g = 0.8;
            line.color.b = 1.0;
            line.points.push_back(interceptor.position);
            line.points.push_back(target_position);
            markers.markers.push_back(line);

            visualization_msgs::Marker label;
            label.header.stamp = stamp;
            label.header.frame_id = world_frame_;
            label.ns = "assignment_line_text";
            label.id = label_id++;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position.x = 0.5 * (interceptor.position.x + target_position.x);
            label.pose.position.y = 0.5 * (interceptor.position.y + target_position.y);
            label.pose.position.z = 0.5 * (interceptor.position.z + target_position.z) + text_height_;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.8 * text_height_;
            label.color.a = 1.0;
            label.color.r = 1.0;
            label.color.g = 0.9;
            label.color.b = 0.2;

            std::ostringstream text;
            text << "I" << interceptor.id << " -> T" << interceptor.assigned_target;
            label.text = text.str();
            markers.markers.push_back(label);
        }

        marker_pub_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    std::string world_frame_;
    int target_count_ = 6;
    int interceptor_count_ = 6;
    double update_rate_ = 5.0;
    int max_interceptors_per_target_ = 3;
    double high_threat_threshold_ = 0.70;
    double medium_threat_threshold_ = 0.40;
    int high_threat_interceptors_ = 3;
    int medium_threat_interceptors_ = 2;
    int low_threat_interceptors_ = 1;
    double interceptor_speed_ = 30.0;
    double max_assignment_distance_ = 300.0;
    double stale_timeout_ = 2.0;
    double interceptor_odom_timeout_ = 2.0;
    std::string interceptor_prefix_ = "/interceptor_";
    double cost_distance_weight_ = 0.55;
    double cost_time_weight_ = 0.35;
    double cost_threat_weight_ = 0.45;
    double interceptor_marker_scale_ = 1.0;
    double line_width_ = 0.18;
    double text_height_ = 1.2;

    std::vector<TargetState> targets_;
    std::vector<InterceptorState> interceptors_;
    std::vector<ros::Subscriber> estimate_subs_;
    std::vector<ros::Subscriber> threat_subs_;
    std::vector<ros::Subscriber> interceptor_odom_subs_;
    ros::Subscriber ranking_sub_;
    ros::Publisher result_pub_;
    ros::Publisher marker_pub_;
    std::string latest_ranking_text_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "cooperative_assignment_node");
    CooperativeAssignmentNode node;
    ros::spin();
    return 0;
}
