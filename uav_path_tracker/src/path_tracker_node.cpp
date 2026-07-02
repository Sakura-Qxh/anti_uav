#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <XmlRpcValue.h>

class PathTrackerNode {
public:
    PathTrackerNode() : nh_(), pnh_("~") {
        loadParams();
        initState();
        createRosInterfaces();

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(update_rate_, 1e-3)),
            &PathTrackerNode::timerCallback,
            this);

        ROS_INFO("path_tracker_node started: role=%s, uav_id=%d",
                 role_.c_str(), uav_id_);
    }

private:
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("uav_id", uav_id_, 0);
        pnh_.param<std::string>("role", role_, "target");
        pnh_.param<double>("update_rate", update_rate_, 50.0);
        pnh_.param<double>("max_speed", max_speed_, 8.0);
        pnh_.param<double>("path_reach_radius", path_reach_radius_, 1.0);
        pnh_.param<int>("path_max_points", path_max_points_, 1500);
        pnh_.param<std::string>("mesh_resource", mesh_resource_,
                                "package://target_scenario_generator/meshes/hummingbird.mesh");
        pnh_.param<double>("mesh_scale", mesh_scale_, 1.0);
        readVectorParam("init_pos", init_pos_, {0.0, 0.0, 10.0});
        pnh_.param<double>("color/r", color_r_, role_ == "target" ? 1.0 : 0.1);
        pnh_.param<double>("color/g", color_g_, role_ == "target" ? 0.2 : 0.8);
        pnh_.param<double>("color/b", color_b_, role_ == "target" ? 0.05 : 1.0);
        pnh_.param<double>("color/a", color_a_, 1.0);
    }

    void readVectorParam(const std::string& name,
                         std::vector<double>& out,
                         const std::vector<double>& fallback) {
        XmlRpc::XmlRpcValue value;
        if (!pnh_.getParam(name, value) ||
            value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            value.size() < 3) {
            out = fallback;
            return;
        }

        out.clear();
        for (int i = 0; i < 3; ++i) {
            if (value[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                out.push_back(static_cast<int>(value[i]));
            } else if (value[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                out.push_back(static_cast<double>(value[i]));
            } else {
                out = fallback;
                return;
            }
        }
    }

    void initState() {
        position_.x = init_pos_[0];
        position_.y = init_pos_[1];
        position_.z = init_pos_[2];
        velocity_.x = 0.0;
        velocity_.y = 0.0;
        velocity_.z = 0.0;
        last_velocity_ = velocity_;
        path_index_ = 0;
        has_path_ = false;
        trajectory_.header.frame_id = world_frame_;
    }

    void createRosInterfaces() {
        planned_path_sub_ = pnh_.subscribe("planned_path", 1,
                                           &PathTrackerNode::plannedPathCallback,
                                           this);
        odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("odom", 1);
        trajectory_pub_ = pnh_.advertise<nav_msgs::Path>("trajectory", 1, true);
        marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("marker", 1, true);

        if (role_ == "target") {
            truth_pub_ = pnh_.advertise<nav_msgs::Odometry>("truth", 1);
            path_pub_ = pnh_.advertise<nav_msgs::Path>("path", 1, true);
        } else {
            guidance_cmd_pub_ = pnh_.advertise<geometry_msgs::AccelStamped>("guidance_cmd", 1);
        }
    }

    void plannedPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        planned_path_ = *msg;
        path_index_ = 0;
        has_path_ = !planned_path_.poses.empty();
        if (has_path_) {
            ROS_INFO_THROTTLE(2.0, "%s_%d received planned_path with %zu poses",
                              role_.c_str(), uav_id_, planned_path_.poses.size());
        }
    }

    void timerCallback(const ros::TimerEvent& event) {
        const ros::Time stamp = ros::Time::now();
        double dt = (event.current_real - event.last_real).toSec();
        if (dt <= 0.0 || !std::isfinite(dt)) {
            dt = 1.0 / std::max(update_rate_, 1e-3);
        }

        last_velocity_ = velocity_;
        if (has_path_) {
            followPath(dt);
        } else {
            velocity_.x = 0.0;
            velocity_.y = 0.0;
            velocity_.z = 0.0;
        }

        appendTrajectory(stamp);
        nav_msgs::Odometry odom = makeOdom(stamp);
        odom_pub_.publish(odom);
        trajectory_pub_.publish(trajectory_);
        marker_pub_.publish(makeMarker(stamp));

        if (role_ == "target") {
            truth_pub_.publish(odom);
            path_pub_.publish(trajectory_);
        } else {
            guidance_cmd_pub_.publish(makeGuidanceCmd(stamp, dt));
        }
    }

    void followPath(double dt) {
        if (planned_path_.poses.empty()) {
            has_path_ = false;
            return;
        }

        while (path_index_ < planned_path_.poses.size()) {
            const auto& point = planned_path_.poses[path_index_].pose.position;
            if (distance(position_, point) > path_reach_radius_) {
                break;
            }
            ++path_index_;
        }

        if (path_index_ >= planned_path_.poses.size()) {
            velocity_.x = 0.0;
            velocity_.y = 0.0;
            velocity_.z = 0.0;
            has_path_ = false;
            return;
        }

        const auto& target = planned_path_.poses[path_index_].pose.position;
        const double dx = target.x - position_.x;
        const double dy = target.y - position_.y;
        const double dz = target.z - position_.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < 1e-6) {
            velocity_.x = 0.0;
            velocity_.y = 0.0;
            velocity_.z = 0.0;
            return;
        }

        const double step = std::min(max_speed_ * dt, dist);
        const double inv_dist = 1.0 / dist;
        const geometry_msgs::Point old_position = position_;
        position_.x += dx * inv_dist * step;
        position_.y += dy * inv_dist * step;
        position_.z += dz * inv_dist * step;
        velocity_.x = (position_.x - old_position.x) / dt;
        velocity_.y = (position_.y - old_position.y) / dt;
        velocity_.z = (position_.z - old_position.z) / dt;
    }

    double distance(const geometry_msgs::Point& a,
                    const geometry_msgs::Point& b) const {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void appendTrajectory(const ros::Time& stamp) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = world_frame_;
        pose.pose.position = position_;
        pose.pose.orientation.w = 1.0;
        trajectory_.header.stamp = stamp;
        trajectory_.header.frame_id = world_frame_;
        trajectory_.poses.push_back(pose);
        if (static_cast<int>(trajectory_.poses.size()) > path_max_points_) {
            trajectory_.poses.erase(trajectory_.poses.begin());
        }
    }

    nav_msgs::Odometry makeOdom(const ros::Time& stamp) const {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = world_frame_;
        odom.child_frame_id = role_ + "_" + std::to_string(uav_id_);
        odom.pose.pose.position = position_;
        odom.pose.pose.orientation.w = 1.0;
        odom.twist.twist.linear = velocity_;
        return odom;
    }

    geometry_msgs::AccelStamped makeGuidanceCmd(const ros::Time& stamp,
                                                double dt) const {
        geometry_msgs::AccelStamped cmd;
        cmd.header.stamp = stamp;
        cmd.header.frame_id = world_frame_;
        cmd.accel.linear.x = (velocity_.x - last_velocity_.x) / dt;
        cmd.accel.linear.y = (velocity_.y - last_velocity_.y) / dt;
        cmd.accel.linear.z = (velocity_.z - last_velocity_.z) / dt;
        return cmd;
    }

    visualization_msgs::Marker makeMarker(const ros::Time& stamp) const {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = world_frame_;
        marker.ns = role_ + "_uav";
        marker.id = uav_id_;
        marker.action = visualization_msgs::Marker::ADD;
        if (!mesh_resource_.empty()) {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.mesh_resource = mesh_resource_;
            marker.mesh_use_embedded_materials = false;
        } else {
            marker.type = visualization_msgs::Marker::SPHERE;
        }
        marker.pose.position = position_;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = mesh_scale_;
        marker.scale.y = mesh_scale_;
        marker.scale.z = mesh_scale_;
        marker.color.r = color_r_;
        marker.color.g = color_g_;
        marker.color.b = color_b_;
        marker.color.a = color_a_;
        return marker;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;
    std::string world_frame_ = "world";
    int uav_id_ = 0;
    std::string role_ = "target";
    double update_rate_ = 50.0;
    double max_speed_ = 8.0;
    double path_reach_radius_ = 1.0;
    int path_max_points_ = 1500;
    std::vector<double> init_pos_;
    std::string mesh_resource_;
    double mesh_scale_ = 1.0;
    double color_r_ = 1.0;
    double color_g_ = 0.2;
    double color_b_ = 0.05;
    double color_a_ = 1.0;
    geometry_msgs::Point position_;
    geometry_msgs::Vector3 velocity_;
    geometry_msgs::Vector3 last_velocity_;
    nav_msgs::Path planned_path_;
    nav_msgs::Path trajectory_;
    size_t path_index_ = 0;
    bool has_path_ = false;
    ros::Subscriber planned_path_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher truth_pub_;
    ros::Publisher path_pub_;
    ros::Publisher guidance_cmd_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_tracker_node");
    PathTrackerNode node;
    ros::spin();
    return 0;
}