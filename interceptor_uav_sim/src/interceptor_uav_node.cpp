#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <XmlRpcValue.h>

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

class InterceptorUavNode
{
public:
  InterceptorUavNode()
    : nh_(), pnh_("~")
  {
    loadParams();

    pos_.x = initial_x_;
    pos_.y = initial_y_;
    pos_.z = initial_z_;
    yaw_ = initial_yaw_;

    pos_cmd_sub_ = pnh_.subscribe(pos_cmd_topic_, 10, &InterceptorUavNode::posCmdCallback, this);

    odom_pub_ = pnh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    truth_pub_ = pnh_.advertise<nav_msgs::Odometry>(truth_topic_, 10);

    if (marker_enabled_)
    {
      marker_pub_ = pnh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
    }

    last_update_time_ = ros::Time::now();
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &InterceptorUavNode::update, this);

    ROS_INFO("interceptor_uav_node started. pos_cmd=%s odom=%s truth=%s",
             pos_cmd_topic_.c_str(), odom_topic_.c_str(), truth_topic_.c_str());
  }

private:
  void loadParams()
  {
    pnh_.param("pos_cmd_topic", pos_cmd_topic_, std::string("planning/pos_cmd"));
    pnh_.param("odom_topic", odom_topic_, std::string("odom"));
    pnh_.param("truth_topic", truth_topic_, std::string("truth"));

    pnh_.param("interceptor_uav/publish_rate", publish_rate_, 50.0);

    pnh_.param("interceptor_uav/initial_x", initial_x_, 0.0);
    pnh_.param("interceptor_uav/initial_y", initial_y_, 0.0);
    pnh_.param("interceptor_uav/initial_z", initial_z_, 2.0);
    pnh_.param("interceptor_uav/initial_yaw", initial_yaw_, 0.0);
    loadNamespacedInitialState();

    pnh_.param("interceptor_uav/kp_pos", kp_pos_, 2.0);
    pnh_.param("interceptor_uav/kd_vel", kd_vel_, 1.6);

    pnh_.param("interceptor_uav/max_speed", max_speed_, 4.0);
    pnh_.param("interceptor_uav/max_acc", max_acc_, 3.0);
    pnh_.param("interceptor_uav/command_timeout", command_timeout_, 0.5);

    pnh_.param("interceptor_uav/track_yaw_from_cmd", track_yaw_from_cmd_, true);

    pnh_.param("interceptor_uav/frame_id", frame_id_, std::string("world"));
    pnh_.param("interceptor_uav/child_frame_id", child_frame_id_, std::string("interceptor_0/base_link"));

    pnh_.param("interceptor_uav/marker_enabled", marker_enabled_, true);
    pnh_.param("interceptor_uav/marker_topic", marker_topic_, std::string("marker"));
    pnh_.param("interceptor_uav/marker_ns", marker_ns_, std::string("interceptor_uav"));
    pnh_.param("interceptor_uav/mesh_resource", mesh_resource_,
               std::string("package://target_scenario_generator/meshes/hummingbird.mesh"));
    pnh_.param("interceptor_uav/mesh_use_embedded_materials", mesh_use_embedded_materials_, true);

    pnh_.param("interceptor_uav/mesh_scale_x", mesh_scale_x_, 1.0);
    pnh_.param("interceptor_uav/mesh_scale_y", mesh_scale_y_, 1.0);
    pnh_.param("interceptor_uav/mesh_scale_z", mesh_scale_z_, 1.0);
    pnh_.param("interceptor_uav/marker_color_r", marker_color_r_, 0.0);
    pnh_.param("interceptor_uav/marker_color_g", marker_color_g_, 0.35);
    pnh_.param("interceptor_uav/marker_color_b", marker_color_b_, 1.0);
    pnh_.param("interceptor_uav/marker_color_a", marker_color_a_, 1.0);
  }

  std::string uavName() const
  {
    std::string ns = ros::this_node::getNamespace();
    if (ns.empty() || ns == "/")
    {
      return "";
    }

    const size_t slash = ns.find_last_of('/');
    return slash == std::string::npos ? ns : ns.substr(slash + 1);
  }

  void loadNamespacedInitialState()
  {
    XmlRpc::XmlRpcValue initial_states;
    const std::string name = uavName();
    if (name.empty() || !pnh_.getParam("interceptor_uav/initial_states", initial_states))
    {
      return;
    }
    if (initial_states.getType() != XmlRpc::XmlRpcValue::TypeStruct || !initial_states.hasMember(name))
    {
      return;
    }

    XmlRpc::XmlRpcValue state = initial_states[name];
    if (state.getType() != XmlRpc::XmlRpcValue::TypeArray || state.size() < 3)
    {
      ROS_WARN("Invalid initial state for %s. Expected [x, y, z] or [x, y, z, yaw].", name.c_str());
      return;
    }

    initial_x_ = static_cast<double>(state[0]);
    initial_y_ = static_cast<double>(state[1]);
    initial_z_ = static_cast<double>(state[2]);
    if (state.size() >= 4)
    {
      initial_yaw_ = static_cast<double>(state[3]);
    }
  }

  void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
  {
    desired_pos_.x = msg->position.x;
    desired_pos_.y = msg->position.y;
    desired_pos_.z = msg->position.z;

    desired_vel_.x = msg->velocity.x;
    desired_vel_.y = msg->velocity.y;
    desired_vel_.z = msg->velocity.z;

    desired_acc_.x = msg->acceleration.x;
    desired_acc_.y = msg->acceleration.y;
    desired_acc_.z = msg->acceleration.z;

    desired_yaw_ = msg->yaw;
    has_cmd_ = true;
    last_cmd_time_ = ros::Time::now();
  }

  void update(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    double dt = (now - last_update_time_).toSec();
    last_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
    {
      dt = 1.0 / publish_rate_;
    }

    integrateState(dt, now);
    publishState(now);
  }

  void integrateState(double dt, const ros::Time& now)
  {
    Vec3 acc_cmd;

    const bool command_valid =
        has_cmd_ && ((now - last_cmd_time_).toSec() <= command_timeout_);

    if (command_valid)
    {
      acc_cmd.x = desired_acc_.x + kp_pos_ * (desired_pos_.x - pos_.x) + kd_vel_ * (desired_vel_.x - vel_.x);
      acc_cmd.y = desired_acc_.y + kp_pos_ * (desired_pos_.y - pos_.y) + kd_vel_ * (desired_vel_.y - vel_.y);
      acc_cmd.z = desired_acc_.z + kp_pos_ * (desired_pos_.z - pos_.z) + kd_vel_ * (desired_vel_.z - vel_.z);

      limitVector(acc_cmd, max_acc_);

      vel_.x += acc_cmd.x * dt;
      vel_.y += acc_cmd.y * dt;
      vel_.z += acc_cmd.z * dt;
      limitVector(vel_, max_speed_);

      pos_.x += vel_.x * dt;
      pos_.y += vel_.y * dt;
      pos_.z += vel_.z * dt;

      if (track_yaw_from_cmd_)
      {
        yaw_ = desired_yaw_;
      }
      else if (std::hypot(vel_.x, vel_.y) > 0.1)
      {
        yaw_ = std::atan2(vel_.y, vel_.x);
      }
    }
    else
    {
      const double damping = 1.5;
      vel_.x += -damping * vel_.x * dt;
      vel_.y += -damping * vel_.y * dt;
      vel_.z += -damping * vel_.z * dt;

      pos_.x += vel_.x * dt;
      pos_.y += vel_.y * dt;
      pos_.z += vel_.z * dt;
    }
  }

  void publishState(const ros::Time& stamp)
  {
    geometry_msgs::Quaternion q = yawToQuaternion(yaw_);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = pos_.x;
    odom.pose.pose.position.y = pos_.y;
    odom.pose.pose.position.z = pos_.z;
    odom.pose.pose.orientation = q;

    odom.twist.twist.linear.x = vel_.x;
    odom.twist.twist.linear.y = vel_.y;
    odom.twist.twist.linear.z = vel_.z;

    odom_pub_.publish(odom);

    truth_pub_.publish(odom);

    if (marker_enabled_)
    {
      visualization_msgs::Marker marker;
      marker.header = odom.header;
      marker.ns = marker_ns_;
      marker.id = 0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = odom.pose.pose;
      marker.scale.x = mesh_scale_x_;
      marker.scale.y = mesh_scale_y_;
      marker.scale.z = mesh_scale_z_;
      marker.mesh_resource = mesh_resource_;
      marker.mesh_use_embedded_materials = mesh_use_embedded_materials_;

      marker.color.r = marker_color_r_;
      marker.color.g = marker_color_g_;
      marker.color.b = marker_color_b_;
      marker.color.a = marker_color_a_;

      marker.lifetime = ros::Duration(0.2);
      marker_pub_.publish(marker);
    }
  }

  static geometry_msgs::Quaternion yawToQuaternion(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
  }

  static void limitVector(Vec3& v, double max_norm)
  {
    if (max_norm <= 0.0)
    {
      return;
    }

    const double norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (norm > max_norm)
    {
      const double scale = max_norm / norm;
      v.x *= scale;
      v.y *= scale;
      v.z *= scale;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pos_cmd_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher truth_pub_;
  ros::Publisher marker_pub_;
  ros::Timer update_timer_;

  std::string pos_cmd_topic_;
  std::string odom_topic_;
  std::string truth_topic_;

  double publish_rate_ = 50.0;

  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_z_ = 2.0;
  double initial_yaw_ = 0.0;

  double kp_pos_ = 2.0;
  double kd_vel_ = 1.6;
  double max_speed_ = 4.0;
  double max_acc_ = 3.0;
  double command_timeout_ = 0.5;

  bool track_yaw_from_cmd_ = true;

  std::string frame_id_ = "world";
  std::string child_frame_id_ = "interceptor_0/base_link";

  bool marker_enabled_ = true;
  std::string marker_topic_ = "marker";
  std::string marker_ns_ = "interceptor_uav";
  std::string mesh_resource_;
  bool mesh_use_embedded_materials_ = true;
  double mesh_scale_x_ = 1.0;
  double mesh_scale_y_ = 1.0;
  double mesh_scale_z_ = 1.0;
  double marker_color_r_ = 0.0;
  double marker_color_g_ = 0.35;
  double marker_color_b_ = 1.0;
  double marker_color_a_ = 1.0;

  Vec3 pos_;
  Vec3 vel_;

  Vec3 desired_pos_;
  Vec3 desired_vel_;
  Vec3 desired_acc_;
  double desired_yaw_ = 0.0;

  double yaw_ = 0.0;
  bool has_cmd_ = false;

  ros::Time last_cmd_time_;
  ros::Time last_update_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interceptor_uav_node");
  InterceptorUavNode node;
  ros::spin();
  return 0;
}
