#include <algorithm>
#include <cmath>
#include <cerrno>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

namespace {

struct VehicleRecord {
    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped slot_goal;
    nav_msgs::Path path;
    ros::Time last_sample;
    bool has_odom = false;
    bool has_slot = false;
};

bool ensureDirectory(const std::string& path) {
    if (path.empty()) return false;
    std::string current;
    for (size_t i = 0; i < path.size(); ++i) {
        current.push_back(path[i]);
        if (path[i] != '/' || current.size() == 1) continue;
        if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
    if (::mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) return false;
    return true;
}

double distance3(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

class ExperimentRecorderNode {
public:
    ExperimentRecorderNode() : nh_(), pnh_("~") {
        loadParams();
        targets_.resize(target_count_);
        interceptors_.resize(interceptor_count_);

        const std::string run_directory = output_directory_ + "/" + run_id_ + "_" +
            std::to_string(ros::WallTime::now().toNSec());
        if (!ensureDirectory(run_directory)) {
            ROS_FATAL("cannot create output directory: %s", run_directory.c_str());
            ros::shutdown();
            return;
        }

        vehicle_csv_.open(run_directory + "/vehicle.csv", std::ios::out | std::ios::trunc);
        encirclement_csv_.open(run_directory + "/encirclement.csv", std::ios::out | std::ios::trunc);
        if (!vehicle_csv_.is_open() || !encirclement_csv_.is_open()) {
            ROS_FATAL("cannot open recorder CSV files in %s", run_directory.c_str());
            ros::shutdown();
            return;
        }

        vehicle_csv_ << "time,vehicle_type,vehicle_id,x,y,z,vx,vy,vz\n";
        encirclement_csv_
            << "time,state,center_x,center_y,center_z,radius,vehicle_id,"
               "relative_x,relative_y,relative_angle,slot_error\n";
        vehicle_csv_ << std::fixed << std::setprecision(6);
        encirclement_csv_ << std::fixed << std::setprecision(6);

        createVehicleInterfaces();
        center_sub_ = nh_.subscribe("/cooperative_guidance/center", 10,
                                    &ExperimentRecorderNode::centerCallback, this);
        radius_sub_ = nh_.subscribe("/cooperative_guidance/radius", 10,
                                    &ExperimentRecorderNode::radiusCallback, this);
        state_sub_ = nh_.subscribe("/cooperative_guidance/state", 10,
                                   &ExperimentRecorderNode::stateCallback, this);

        ROS_INFO("experiment recorder output: %s", run_directory.c_str());
    }

    ~ExperimentRecorderNode() {
        vehicle_csv_.flush();
        encirclement_csv_.flush();
    }

private:
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<int>("interceptor_count", interceptor_count_, 4);
        pnh_.param<double>("sample_period", sample_period_, 0.1);
        pnh_.param<int>("path_max_points", path_max_points_, 600);
        pnh_.param<std::string>("output_directory", output_directory_,
                                "/tmp/anti_uav_experiment_results");
        pnh_.param<std::string>("run_id", run_id_, "scene_01_seed_01");
        target_count_ = std::max(1, target_count_);
        interceptor_count_ = std::max(1, interceptor_count_);
        path_max_points_ = std::max(1, path_max_points_);
        sample_period_ = std::max(0.01, sample_period_);
    }

    void createVehicleInterfaces() {
        for (int i = 0; i < target_count_; ++i) {
            const std::string base = "/target_" + std::to_string(i);
            target_path_pubs_.push_back(
                nh_.advertise<nav_msgs::Path>("/experiment/target_" +
                                              std::to_string(i) + "/path", 1, true));
            target_odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                base + "/odom", 50,
                [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    odomCallback(msg, "target", i, targets_[i], target_path_pubs_[i]);
                }));
        }

        for (int i = 0; i < interceptor_count_; ++i) {
            const std::string base = "/interceptor_" + std::to_string(i);
            interceptor_path_pubs_.push_back(
                nh_.advertise<nav_msgs::Path>("/experiment/interceptor_" +
                                              std::to_string(i) + "/path", 1, true));
            interceptor_odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                base + "/odom", 50,
                [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    odomCallback(msg, "interceptor", i,
                                 interceptors_[i], interceptor_path_pubs_[i]);
                }));
            slot_goal_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(
                base + "/capture_goal", 10,
                [this, i](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    interceptors_[i].slot_goal = *msg;
                    interceptors_[i].has_slot = true;
                }));
        }
    }

    void centerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        center_ = msg->point;
        has_center_ = true;
    }

    void radiusCallback(const std_msgs::Float64::ConstPtr& msg) {
        radius_ = msg->data;
    }

    void stateCallback(const std_msgs::String::ConstPtr& msg) {
        state_ = msg->data;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg,
                      const std::string& type,
                      int id,
                      VehicleRecord& record,
                      ros::Publisher& path_pub) {
        const ros::Time stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        record.odom = *msg;
        record.has_odom = true;

        if (!record.last_sample.isZero() &&
            (stamp - record.last_sample).toSec() < sample_period_) {
            return;
        }
        record.last_sample = stamp;

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = world_frame_;
        pose.pose = msg->pose.pose;
        record.path.header = pose.header;
        record.path.poses.push_back(pose);
        if (static_cast<int>(record.path.poses.size()) > path_max_points_) {
            record.path.poses.erase(record.path.poses.begin());
        }
        path_pub.publish(record.path);

        vehicle_csv_ << stamp.toSec() << ',' << type << ',' << id << ','
                     << msg->pose.pose.position.x << ','
                     << msg->pose.pose.position.y << ','
                     << msg->pose.pose.position.z << ','
                     << msg->twist.twist.linear.x << ','
                     << msg->twist.twist.linear.y << ','
                     << msg->twist.twist.linear.z << '\n';

        if (type == "interceptor" && has_center_) {
            const double relative_x = msg->pose.pose.position.x - center_.x;
            const double relative_y = msg->pose.pose.position.y - center_.y;
            const double relative_angle = std::atan2(relative_y, relative_x);
            const double slot_error = record.has_slot
                ? distance3(msg->pose.pose.position, record.slot_goal.pose.position)
                : -1.0;
            encirclement_csv_ << stamp.toSec() << ',' << state_ << ','
                              << center_.x << ',' << center_.y << ',' << center_.z << ','
                              << radius_ << ',' << id << ','
                              << relative_x << ',' << relative_y << ','
                              << relative_angle << ',' << slot_error << '\n';
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string world_frame_ = "world";
    int target_count_ = 3;
    int interceptor_count_ = 4;
    double sample_period_ = 0.1;
    int path_max_points_ = 600;
    std::string output_directory_ = "/tmp/anti_uav_experiment_results";
    std::string run_id_ = "scene_01_seed_01";

    geometry_msgs::Point center_;
    bool has_center_ = false;
    double radius_ = 0.0;
    std::string state_ = "WAIT_TARGETS";
    std::ofstream vehicle_csv_;
    std::ofstream encirclement_csv_;
    std::vector<VehicleRecord> targets_;
    std::vector<VehicleRecord> interceptors_;
    std::vector<ros::Subscriber> target_odom_subs_;
    std::vector<ros::Subscriber> interceptor_odom_subs_;
    std::vector<ros::Subscriber> slot_goal_subs_;
    std::vector<ros::Publisher> target_path_pubs_;
    std::vector<ros::Publisher> interceptor_path_pubs_;
    ros::Subscriber center_sub_;
    ros::Subscriber radius_sub_;
    ros::Subscriber state_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "experiment_recorder_node");
    ExperimentRecorderNode node;
    ros::spin();
    return 0;
}
