#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

class ConstantVelocityKalmanFilter {
public:
    void configure(double process_noise_position,
                   double process_noise_velocity,
                   double measurement_noise_position,
                   double measurement_noise_velocity,
                   double initial_position_variance,
                   double initial_velocity_variance) {
        q_pos_ = process_noise_position;
        q_vel_ = process_noise_velocity;
        r_pos_ = measurement_noise_position;
        r_vel_ = measurement_noise_velocity;
        p_pos0_ = initial_position_variance;
        p_vel0_ = initial_velocity_variance;
    }

    void initialize(const nav_msgs::Odometry& measurement) {
        x_.setZero();
        x_(0) = measurement.pose.pose.position.x;
        x_(1) = measurement.pose.pose.position.y;
        x_(2) = measurement.pose.pose.position.z;
        x_(3) = measurement.twist.twist.linear.x;
        x_(4) = measurement.twist.twist.linear.y;
        x_(5) = measurement.twist.twist.linear.z;

        P_.setZero();
        P_(0, 0) = p_pos0_;
        P_(1, 1) = p_pos0_;
        P_(2, 2) = p_pos0_;
        P_(3, 3) = p_vel0_;
        P_(4, 4) = p_vel0_;
        P_(5, 5) = p_vel0_;

        initialized_ = true;
    }

    bool initialized() const {
        return initialized_;
    }

    void predict(double dt) {
        if (!initialized_) {
            return;
        }

        dt = std::max(1e-3, std::min(dt, 0.2));

        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;

        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        Q(0, 0) = q_pos_ * dt * dt;
        Q(1, 1) = q_pos_ * dt * dt;
        Q(2, 2) = q_pos_ * dt * dt;
        Q(3, 3) = q_vel_ * dt;
        Q(4, 4) = q_vel_ * dt;
        Q(5, 5) = q_vel_ * dt;

        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    void update(const nav_msgs::Odometry& measurement) {
        if (!initialized_) {
            initialize(measurement);
            return;
        }

        Eigen::Matrix<double, 6, 1> z;
        z << measurement.pose.pose.position.x,
             measurement.pose.pose.position.y,
             measurement.pose.pose.position.z,
             measurement.twist.twist.linear.x,
             measurement.twist.twist.linear.y,
             measurement.twist.twist.linear.z;

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
        R(0, 0) = r_pos_ * r_pos_;
        R(1, 1) = r_pos_ * r_pos_;
        R(2, 2) = r_pos_ * r_pos_;
        R(3, 3) = r_vel_ * r_vel_;
        R(4, 4) = r_vel_ * r_vel_;
        R(5, 5) = r_vel_ * r_vel_;

        const Eigen::Matrix<double, 6, 1> y = z - H * x_;
        const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
        const Eigen::Matrix<double, 6, 6> K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;
        const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        P_ = (I - K * H) * P_;
    }

    double positionDistanceTo(const nav_msgs::Odometry& measurement) const {
        if (!initialized_) {
            return 0.0;
        }
        const double dx = measurement.pose.pose.position.x - x_(0);
        const double dy = measurement.pose.pose.position.y - x_(1);
        const double dz = measurement.pose.pose.position.z - x_(2);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    const Eigen::Matrix<double, 6, 1>& state() const {
        return x_;
    }

private:
    bool initialized_ = false;
    Eigen::Matrix<double, 6, 1> x_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> P_ = Eigen::Matrix<double, 6, 6>::Identity();
    double q_pos_ = 0.05;
    double q_vel_ = 0.5;
    double r_pos_ = 0.5;
    double r_vel_ = 0.2;
    double p_pos0_ = 2.0;
    double p_vel0_ = 5.0;
};

struct TargetTrack {
    ConstantVelocityKalmanFilter filter;
    ros::Time last_update_time;
    ros::Time last_measurement_time;
    bool has_measurement = false;
    bool last_update_rejected = false;
    nav_msgs::Path estimate_path;
};

class TargetTrackingNode {
public:
    TargetTrackingNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 6);
        pnh_.param<double>("update_rate", update_rate_, 50.0);
        pnh_.param<double>("process_noise_position", process_noise_position_, 0.05);
        pnh_.param<double>("process_noise_velocity", process_noise_velocity_, 0.5);
        pnh_.param<double>("measurement_noise_position", measurement_noise_position_, 0.5);
        pnh_.param<double>("measurement_noise_velocity", measurement_noise_velocity_, 0.2);
        pnh_.param<double>("initial_position_variance", initial_position_variance_, 2.0);
        pnh_.param<double>("initial_velocity_variance", initial_velocity_variance_, 5.0);
        pnh_.param<double>("gating_distance", gating_distance_, 8.0);
        pnh_.param<double>("max_prediction_gap", max_prediction_gap_, 2.0);
        pnh_.param<double>("prediction_horizon", prediction_horizon_, 2.0);
        pnh_.param<double>("prediction_dt", prediction_dt_, 0.2);
        pnh_.param<int>("path_max_points", path_max_points_, 300);
        pnh_.param<double>("estimate_marker_scale", estimate_marker_scale_, 0.55);
        pnh_.param<double>("prediction_marker_scale", prediction_marker_scale_, 0.08);

        tracks_.resize(target_count_);
        for (auto& track : tracks_) {
            track.filter.configure(process_noise_position_,
                                   process_noise_velocity_,
                                   measurement_noise_position_,
                                   measurement_noise_velocity_,
                                   initial_position_variance_,
                                   initial_velocity_variance_);
        }

        createRosInterfaces();
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_),
                                &TargetTrackingNode::timerCallback,
                                this);

        ROS_INFO("robust_target_tracking started for %d targets", target_count_);
    }

private:
    void createRosInterfaces() {
        measurement_subs_.clear();
        estimate_pubs_.clear();
        prediction_pubs_.clear();
        estimate_path_pubs_.clear();

        for (int i = 0; i < target_count_; ++i) {
            const std::string measurement_topic = "/radar/target_" + std::to_string(i) + "/measurement";
            const std::string estimate_topic = "/target_" + std::to_string(i) + "/estimate";
            const std::string prediction_topic = "/target_" + std::to_string(i) + "/prediction";
            const std::string estimate_path_topic = "/target_" + std::to_string(i) + "/estimate_path";

            measurement_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                measurement_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    measurementCallback(msg, i);
                }));

            estimate_pubs_.push_back(nh_.advertise<nav_msgs::Odometry>(estimate_topic, 10));
            prediction_pubs_.push_back(nh_.advertise<nav_msgs::Path>(prediction_topic, 1));
            estimate_path_pubs_.push_back(nh_.advertise<nav_msgs::Path>(estimate_path_topic, 1));
        }

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/target_tracking/markers", 1);
    }

    void measurementCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= static_cast<int>(tracks_.size())) {
            return;
        }

        TargetTrack& track = tracks_[target_index];
        const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

        if (!track.filter.initialized()) {
            track.filter.initialize(*msg);
            track.last_update_time = stamp;
            track.last_measurement_time = stamp;
            track.has_measurement = true;
            track.last_update_rejected = false;
            return;
        }

        const double dt = (stamp - track.last_update_time).toSec();
        if (dt > 0.0) {
            track.filter.predict(dt);
            track.last_update_time = stamp;
        }

        const double innovation_distance = track.filter.positionDistanceTo(*msg);
        if (innovation_distance > gating_distance_) {
            track.last_update_rejected = true;
            return;
        }

        track.filter.update(*msg);
        track.last_update_time = stamp;
        track.last_measurement_time = stamp;
        track.has_measurement = true;
        track.last_update_rejected = false;
    }

    void timerCallback(const ros::TimerEvent& event) {
        const ros::Time stamp = ros::Time::now();

        for (int i = 0; i < target_count_; ++i) {
            TargetTrack& track = tracks_[i];
            if (!track.filter.initialized()) {
                continue;
            }

            const double dt = track.last_update_time.isZero()
                                  ? 1.0 / update_rate_
                                  : (stamp - track.last_update_time).toSec();
            if (dt > 0.0) {
                track.filter.predict(dt);
                track.last_update_time = stamp;
            }

            const double gap = track.last_measurement_time.isZero()
                                   ? 0.0
                                   : (stamp - track.last_measurement_time).toSec();
            if (gap <= max_prediction_gap_) {
                publishEstimate(i, stamp);
                publishPrediction(i, stamp);
            }
        }

        publishMarkers(stamp);
    }

    void publishEstimate(int target_index, const ros::Time& stamp) {
        const auto& state = tracks_[target_index].filter.state();

        nav_msgs::Odometry estimate;
        estimate.header.stamp = stamp;
        estimate.header.frame_id = world_frame_;
        estimate.child_frame_id = "target_" + std::to_string(target_index) + "_estimate";
        estimate.pose.pose.position.x = state(0);
        estimate.pose.pose.position.y = state(1);
        estimate.pose.pose.position.z = state(2);
        estimate.pose.pose.orientation.w = 1.0;
        estimate.twist.twist.linear.x = state(3);
        estimate.twist.twist.linear.y = state(4);
        estimate.twist.twist.linear.z = state(5);

        estimate_pubs_[target_index].publish(estimate);
        appendEstimatePath(target_index, estimate);
    }

    void appendEstimatePath(int target_index, const nav_msgs::Odometry& estimate) {
        geometry_msgs::PoseStamped pose;
        pose.header = estimate.header;
        pose.pose = estimate.pose.pose;

        nav_msgs::Path& path = tracks_[target_index].estimate_path;
        path.header.stamp = estimate.header.stamp;
        path.header.frame_id = world_frame_;
        path.poses.push_back(pose);

        if (static_cast<int>(path.poses.size()) > path_max_points_) {
            path.poses.erase(path.poses.begin());
        }

        estimate_path_pubs_[target_index].publish(path);
    }

    void publishPrediction(int target_index, const ros::Time& stamp) {
        const auto& state = tracks_[target_index].filter.state();

        nav_msgs::Path prediction;
        prediction.header.stamp = stamp;
        prediction.header.frame_id = world_frame_;

        for (double t = prediction_dt_; t <= prediction_horizon_ + 1e-6; t += prediction_dt_) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = stamp + ros::Duration(t);
            pose.header.frame_id = world_frame_;
            pose.pose.position.x = state(0) + state(3) * t;
            pose.pose.position.y = state(1) + state(4) * t;
            pose.pose.position.z = state(2) + state(5) * t;
            pose.pose.orientation.w = 1.0;
            prediction.poses.push_back(pose);
        }

        prediction_pubs_[target_index].publish(prediction);
    }

    void publishMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            if (!tracks_[i].filter.initialized()) {
                continue;
            }

            const auto& state = tracks_[i].filter.state();

            visualization_msgs::Marker estimate_marker;
            estimate_marker.header.stamp = stamp;
            estimate_marker.header.frame_id = world_frame_;
            estimate_marker.ns = "target_estimates";
            estimate_marker.id = i;
            estimate_marker.type = visualization_msgs::Marker::CUBE;
            estimate_marker.action = visualization_msgs::Marker::ADD;
            estimate_marker.pose.position.x = state(0);
            estimate_marker.pose.position.y = state(1);
            estimate_marker.pose.position.z = state(2);
            estimate_marker.pose.orientation.w = 1.0;
            estimate_marker.scale.x = estimate_marker_scale_;
            estimate_marker.scale.y = estimate_marker_scale_;
            estimate_marker.scale.z = estimate_marker_scale_;
            estimate_marker.color.a = 1.0;
            estimate_marker.color.r = tracks_[i].last_update_rejected ? 1.0 : 0.1;
            estimate_marker.color.g = tracks_[i].last_update_rejected ? 0.2 : 0.8;
            estimate_marker.color.b = 1.0;
            markers.markers.push_back(estimate_marker);

            visualization_msgs::Marker prediction_marker;
            prediction_marker.header.stamp = stamp;
            prediction_marker.header.frame_id = world_frame_;
            prediction_marker.ns = "target_predictions";
            prediction_marker.id = i;
            prediction_marker.type = visualization_msgs::Marker::LINE_STRIP;
            prediction_marker.action = visualization_msgs::Marker::ADD;
            prediction_marker.scale.x = prediction_marker_scale_;
            prediction_marker.color.a = 0.9;
            prediction_marker.color.r = 0.2;
            prediction_marker.color.g = 0.9;
            prediction_marker.color.b = 1.0;

            for (double t = 0.0; t <= prediction_horizon_ + 1e-6; t += prediction_dt_) {
                geometry_msgs::Point point;
                point.x = state(0) + state(3) * t;
                point.y = state(1) + state(4) * t;
                point.z = state(2) + state(5) * t;
                prediction_marker.points.push_back(point);
            }
            markers.markers.push_back(prediction_marker);
        }

        marker_pub_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    std::string world_frame_;
    int target_count_ = 6;
    double update_rate_ = 50.0;
    double process_noise_position_ = 0.05;
    double process_noise_velocity_ = 0.5;
    double measurement_noise_position_ = 0.5;
    double measurement_noise_velocity_ = 0.2;
    double initial_position_variance_ = 2.0;
    double initial_velocity_variance_ = 5.0;
    double gating_distance_ = 8.0;
    double max_prediction_gap_ = 2.0;
    double prediction_horizon_ = 2.0;
    double prediction_dt_ = 0.2;
    int path_max_points_ = 300;
    double estimate_marker_scale_ = 0.55;
    double prediction_marker_scale_ = 0.08;

    std::vector<TargetTrack> tracks_;
    std::vector<ros::Subscriber> measurement_subs_;
    std::vector<ros::Publisher> estimate_pubs_;
    std::vector<ros::Publisher> prediction_pubs_;
    std::vector<ros::Publisher> estimate_path_pubs_;
    ros::Publisher marker_pub_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_tracking_node");
    TargetTrackingNode node;
    ros::spin();
    return 0;
}