#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <ros/ros.h>

class PolynomialTrajectoryToPathNode {
public:
    PolynomialTrajectoryToPathNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<double>("sample_dt", sample_dt_, 0.05);
        pnh_.param<double>("max_duration", max_duration_, 20.0);
        traj_sub_ = pnh_.subscribe("trajectory", 2,
                                   &PolynomialTrajectoryToPathNode::trajectoryCallback,
                                   this);
        path_pub_ = pnh_.advertise<nav_msgs::Path>("planned_path", 1, true);
        ROS_INFO("polynomial_trajectory_to_path_node started");
    }

private:
    void trajectoryCallback(const quadrotor_msgs::PolynomialTrajectory::ConstPtr& msg) {
        if (msg->action != quadrotor_msgs::PolynomialTrajectory::ACTION_ADD ||
            msg->num_segment == 0 || msg->time.empty()) {
            return;
        }

        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id;

        double elapsed = 0.0;
        int coef_base = 0;
        for (int seg = 0; seg < static_cast<int>(msg->num_segment); ++seg) {
            if (seg >= static_cast<int>(msg->time.size()) ||
                seg >= static_cast<int>(msg->order.size())) {
                break;
            }

            const int order = static_cast<int>(msg->order[seg]);
            const double duration = std::max(0.0, msg->time[seg]);
            const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt_)));

            for (int k = 0; k <= sample_count; ++k) {
                const double t = std::min(duration, k * sample_dt_);
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = evaluate(msg->coef_x, coef_base, order, t);
                pose.pose.position.y = evaluate(msg->coef_y, coef_base, order, t);
                pose.pose.position.z = evaluate(msg->coef_z, coef_base, order, t);
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }

            elapsed += duration;
            coef_base += order + 1;
            if (elapsed >= max_duration_) {
                break;
            }
        }

        if (!path.poses.empty()) {
            path_pub_.publish(path);
        }
    }

    double evaluate(const std::vector<double>& coefs, int base, int order, double t) const {
        double value = 0.0;
        double power = 1.0;
        for (int i = 0; i <= order; ++i) {
            const int idx = base + i;
            if (idx >= 0 && idx < static_cast<int>(coefs.size())) {
                value += coefs[idx] * power;
            }
            power *= t;
        }
        return value;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string world_frame_ = "world";
    double sample_dt_ = 0.05;
    double max_duration_ = 20.0;
    ros::Subscriber traj_sub_;
    ros::Publisher path_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "polynomial_trajectory_to_path_node");
    PolynomialTrajectoryToPathNode node;
    ros::spin();
    return 0;
}
