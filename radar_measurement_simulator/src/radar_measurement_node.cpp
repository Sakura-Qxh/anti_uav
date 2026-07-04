#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

struct TargetMeasurementState {
    nav_msgs::Odometry latest_truth;
    bool has_truth = false;
    bool in_burst_loss = false;
    ros::Time burst_loss_end;
};

class RadarMeasurementNode {
public:
    RadarMeasurementNode() : nh_(), pnh_("~"), rng_(std::random_device{}()) {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 6);
        pnh_.param<double>("position_noise_std", position_noise_std_, 0.5);
        pnh_.param<double>("velocity_noise_std", velocity_noise_std_, 0.2);
        pnh_.param<double>("random_drop_prob", random_drop_prob_, 0.1);
        pnh_.param<double>("outlier_prob", outlier_prob_, 0.03);
        pnh_.param<double>("outlier_position_jump", outlier_position_jump_, 8.0);
        pnh_.param<bool>("enable_burst_loss", enable_burst_loss_, true);
        pnh_.param<double>("burst_loss_prob", burst_loss_prob_, 0.02);
        pnh_.param<double>("burst_loss_duration", burst_loss_duration_, 1.0);
        pnh_.param<double>("marker_scale", marker_scale_, 0.35);

        targets_.resize(target_count_);
        latest_measurements_.resize(target_count_);
        latest_valid_.assign(target_count_, false);
        createSubscribersAndPublishers();

        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar/measurements/markers", 1);

        ROS_INFO("radar_measurement_simulator started for %d targets", target_count_);
    }

private:
    void createSubscribersAndPublishers() {
        truth_subs_.clear();
        measurement_pubs_.clear();

        for (int i = 0; i < target_count_; ++i) {
            const std::string truth_topic = "/target_" + std::to_string(i) + "/truth";
            const std::string measurement_topic = "/radar/target_" + std::to_string(i) + "/measurement";

            truth_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                truth_topic, 10, [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    truthCallback(msg, i);
                }));

            measurement_pubs_.push_back(nh_.advertise<nav_msgs::Odometry>(measurement_topic, 10));
        }
    }

    void truthCallback(const nav_msgs::Odometry::ConstPtr& msg, int target_index) {
        if (target_index < 0 || target_index >= static_cast<int>(targets_.size())) {
            return;
        }

        auto& target = targets_[target_index];
        target.latest_truth = *msg;
        target.has_truth = true;

        const ros::Time now = ros::Time::now();
        if (shouldDropMeasurement(target, now)) {
            publishMarkers(now);
            return;
        }

        nav_msgs::Odometry measurement = makeMeasurement(*msg, target_index);
        measurement_pubs_[target_index].publish(measurement);
        latest_measurements_[target_index] = measurement;
        latest_valid_[target_index] = true;

        publishMarkers(now);
    }

    bool shouldDropMeasurement(TargetMeasurementState& target, const ros::Time& now) {
        if (target.in_burst_loss) {
            if (now < target.burst_loss_end) {
                return true;
            }
            target.in_burst_loss = false;
        }

        if (enable_burst_loss_ && uniform01_() < burst_loss_prob_) {
            target.in_burst_loss = true;
            target.burst_loss_end = now + ros::Duration(burst_loss_duration_);
            return true;
        }

        return uniform01_() < random_drop_prob_;
    }

    nav_msgs::Odometry makeMeasurement(const nav_msgs::Odometry& truth, int target_index) {
        nav_msgs::Odometry measurement = truth;
        measurement.header.stamp = ros::Time::now();
        measurement.header.frame_id = world_frame_;
        measurement.child_frame_id = "radar_target_" + std::to_string(target_index);

        measurement.pose.pose.position.x += normal_(position_noise_std_);
        measurement.pose.pose.position.y += normal_(position_noise_std_);
        measurement.pose.pose.position.z += normal_(position_noise_std_);

        measurement.twist.twist.linear.x += normal_(velocity_noise_std_);
        measurement.twist.twist.linear.y += normal_(velocity_noise_std_);
        measurement.twist.twist.linear.z += normal_(velocity_noise_std_);

        if (uniform01_() < outlier_prob_) {
            measurement.pose.pose.position.x += normal_(outlier_position_jump_);
            measurement.pose.pose.position.y += normal_(outlier_position_jump_);
            measurement.pose.pose.position.z += normal_(0.5 * outlier_position_jump_);
        }

        return measurement;
    }

    double uniform01_() {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(rng_);
    }

    double normal_(double stddev) {
        if (stddev <= 0.0) {
            return 0.0;
        }
        std::normal_distribution<double> dist(0.0, stddev);
        return dist(rng_);
    }

    void publishMarkers(const ros::Time& stamp) {
        visualization_msgs::MarkerArray markers;

        for (int i = 0; i < target_count_; ++i) {
            visualization_msgs::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = world_frame_;
            marker.ns = "radar_measurements";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = latest_valid_[i] ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

            if (latest_valid_[i]) {
                marker.pose = latest_measurements_[i].pose.pose;
                marker.scale.x = marker_scale_;
                marker.scale.y = marker_scale_;
                marker.scale.z = marker_scale_;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.1;
            }

            markers.markers.push_back(marker);
        }

        marker_pub_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string world_frame_;
    int target_count_ = 6;
    double position_noise_std_ = 0.5;
    double velocity_noise_std_ = 0.2;
    double random_drop_prob_ = 0.1;
    double outlier_prob_ = 0.03;
    double outlier_position_jump_ = 8.0;
    bool enable_burst_loss_ = true;
    double burst_loss_prob_ = 0.02;
    double burst_loss_duration_ = 1.0;
    double marker_scale_ = 0.35;

    std::mt19937 rng_;
    std::vector<TargetMeasurementState> targets_;
    std::vector<ros::Subscriber> truth_subs_;
    std::vector<ros::Publisher> measurement_pubs_;
    std::vector<nav_msgs::Odometry> latest_measurements_;
    std::vector<bool> latest_valid_;
    ros::Publisher marker_pub_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_measurement_node");
    RadarMeasurementNode node;
    ros::spin();
    return 0;
}
