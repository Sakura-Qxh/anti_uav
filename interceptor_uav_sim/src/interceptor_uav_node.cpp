#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <XmlRpcValue.h>

/**
 * @brief 三维向量结构体，用于存储位置、速度、加速度等
 */
struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/**
 * @brief 拦截器无人机仿真节点类
 *
 * 与 TargetUavNode 功能相同，但用于拦截器无人机。
 * 订阅位置命令话题（planning/pos_cmd），使用PD控制器跟踪命令，
 * 通过简单的质点动力学模型（加速度限幅、速度限幅）积分得到状态，
 * 定期发布里程计（odom）和真值（truth），并可选发布3D网格模型标记（默认蓝色）。
 */
class InterceptorUavNode
{
public:
  InterceptorUavNode()
    : nh_(), pnh_("~")
  {
    loadParams();   // 加载所有参数

    // 初始化位置和偏航角
    pos_.x = initial_x_;
    pos_.y = initial_y_;
    pos_.z = initial_z_;
    yaw_ = initial_yaw_;

    // 订阅位置命令话题
    pos_cmd_sub_ = pnh_.subscribe(pos_cmd_topic_, 10, &InterceptorUavNode::posCmdCallback, this);

    // 创建里程计和真值发布器
    odom_pub_ = pnh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    truth_pub_ = pnh_.advertise<nav_msgs::Odometry>(truth_topic_, 10);

    // 如果启用了标记，创建标记发布器
    if (marker_enabled_)
    {
      marker_pub_ = pnh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
    }

    last_update_time_ = ros::Time::now();
    // 创建定时器，按指定频率更新状态
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &InterceptorUavNode::update, this);

    ROS_INFO("interceptor_uav_node started. pos_cmd=%s odom=%s truth=%s",
             pos_cmd_topic_.c_str(), odom_topic_.c_str(), truth_topic_.c_str());
  }

private:
  /**
   * @brief 加载所有配置参数
   */
  void loadParams()
  {
    // 话题名称
    pnh_.param("pos_cmd_topic", pos_cmd_topic_, std::string("planning/pos_cmd"));
    pnh_.param("odom_topic", odom_topic_, std::string("odom"));
    pnh_.param("truth_topic", truth_topic_, std::string("truth"));

    // 发布频率
    pnh_.param("interceptor_uav/publish_rate", publish_rate_, 50.0);

    // 初始状态
    pnh_.param("interceptor_uav/initial_x", initial_x_, 0.0);
    pnh_.param("interceptor_uav/initial_y", initial_y_, 0.0);
    pnh_.param("interceptor_uav/initial_z", initial_z_, 2.0);
    pnh_.param("interceptor_uav/initial_yaw", initial_yaw_, 0.0);
    loadNamespacedInitialState();   // 尝试加载命名空间特定的初始状态

    // PD控制器参数
    pnh_.param("interceptor_uav/kp_pos", kp_pos_, 2.0);
    pnh_.param("interceptor_uav/kd_vel", kd_vel_, 1.6);

    // 运动学限制
    pnh_.param("interceptor_uav/max_speed", max_speed_, 4.0);
    pnh_.param("interceptor_uav/max_acc", max_acc_, 3.0);
    pnh_.param("interceptor_uav/command_timeout", command_timeout_, 0.5);

    // 偏航控制方式
    pnh_.param("interceptor_uav/track_yaw_from_cmd", track_yaw_from_cmd_, true);

    // 坐标系
    pnh_.param("interceptor_uav/frame_id", frame_id_, std::string("world"));
    pnh_.param("interceptor_uav/child_frame_id", child_frame_id_, std::string("interceptor_0/base_link"));

    // 可视化标记参数
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

  /**
   * @brief 获取当前节点的命名空间中的无人机名称
   * @return 例如 "/interceptor_0" 返回 "interceptor_0"，根命名空间返回空字符串
   */
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

  /**
   * @brief 从参数服务器加载命名空间特定的初始状态
   *
   * 参数格式：interceptor_uav/initial_states = { interceptor_0: [x, y, z, yaw], ... }
   * 可以覆盖通过普通参数设置的初始值。
   */
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

  /**
   * @brief 位置命令回调：更新期望的位置、速度、加速度和偏航
   * @param msg 位置命令消息
   */
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

  /**
   * @brief 定时器回调：执行状态积分并发布
   */
  void update(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    double dt = (now - last_update_time_).toSec();
    last_update_time_ = now;

    // 限制时间步长，避免数值不稳定
    if (dt <= 0.0 || dt > 0.2)
    {
      dt = 1.0 / publish_rate_;
    }

    integrateState(dt, now);
    publishState(now);
  }

  /**
   * @brief 根据命令或阻尼运动进行状态积分
   * @param dt 时间步长
   * @param now 当前时间
   *
   * 如果有有效命令（未超时），则使用PD控制器计算加速度指令，
   * 否则施加阻尼使无人机减速至静止。
   */
  void integrateState(double dt, const ros::Time& now)
  {
    Vec3 acc_cmd;

    const bool command_valid =
        has_cmd_ && ((now - last_cmd_time_).toSec() <= command_timeout_);

    if (command_valid)
    {
      // PD控制器：加速度 = 前馈 + 位置误差比例 + 速度误差微分
      acc_cmd.x = desired_acc_.x + kp_pos_ * (desired_pos_.x - pos_.x) + kd_vel_ * (desired_vel_.x - vel_.x);
      acc_cmd.y = desired_acc_.y + kp_pos_ * (desired_pos_.y - pos_.y) + kd_vel_ * (desired_vel_.y - vel_.y);
      acc_cmd.z = desired_acc_.z + kp_pos_ * (desired_pos_.z - pos_.z) + kd_vel_ * (desired_vel_.z - vel_.z);

      // 限幅加速度
      limitVector(acc_cmd, max_acc_);

      // 积分速度
      vel_.x += acc_cmd.x * dt;
      vel_.y += acc_cmd.y * dt;
      vel_.z += acc_cmd.z * dt;
      limitVector(vel_, max_speed_);  // 限幅速度

      // 积分位置
      pos_.x += vel_.x * dt;
      pos_.y += vel_.y * dt;
      pos_.z += vel_.z * dt;

      // 偏航角处理
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
      // 无命令时：施加阻尼使速度衰减
      const double damping = 1.5;
      vel_.x += -damping * vel_.x * dt;
      vel_.y += -damping * vel_.y * dt;
      vel_.z += -damping * vel_.z * dt;

      // 继续移动
      pos_.x += vel_.x * dt;
      pos_.y += vel_.y * dt;
      pos_.z += vel_.z * dt;
    }
  }

  /**
   * @brief 发布里程计、真值和可视化标记
   * @param stamp 时间戳
   */
  void publishState(const ros::Time& stamp)
  {
    geometry_msgs::Quaternion q = yawToQuaternion(yaw_);

    // 构建里程计消息（同时用作odom和truth）
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
    truth_pub_.publish(odom);  // 真值与里程计相同（仿真中无噪声）

    // 可选：发布3D网格模型标记
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

      marker.lifetime = ros::Duration(0.2);  // 短暂生命周期，由定时器刷新
      marker_pub_.publish(marker);
    }
  }

  /**
   * @brief 将偏航角转换为四元数（翻滚和俯仰为0）
   * @param yaw 偏航角（弧度）
   * @return 对应的四元数消息
   */
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

  /**
   * @brief 将向量的模限制在最大值以内
   * @param v 待限幅的向量
   * @param max_norm 最大模长，若<=0则不操作
   */
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
  // ROS句柄
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 通信接口
  ros::Subscriber pos_cmd_sub_;     // 位置命令订阅器
  ros::Publisher odom_pub_;         // 里程计发布器
  ros::Publisher truth_pub_;        // 真值发布器
  ros::Publisher marker_pub_;       // 可视化标记发布器
  ros::Timer update_timer_;         // 更新定时器

  // 话题名称
  std::string pos_cmd_topic_;
  std::string odom_topic_;
  std::string truth_topic_;

  // 更新频率
  double publish_rate_ = 50.0;

  // 初始状态
  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_z_ = 2.0;
  double initial_yaw_ = 0.0;

  // PD控制器参数
  double kp_pos_ = 2.0;   // 位置比例增益
  double kd_vel_ = 1.6;   // 速度微分增益
  double max_speed_ = 4.0; // 最大速度（m/s）
  double max_acc_ = 3.0;   // 最大加速度（m/s²）
  double command_timeout_ = 0.5; // 命令超时时间（秒）

  // 偏航控制
  bool track_yaw_from_cmd_ = true; // 是否直接从命令获取偏航

  // 坐标系
  std::string frame_id_ = "world";
  std::string child_frame_id_ = "interceptor_0/base_link";

  // 可视化标记参数
  bool marker_enabled_ = true;
  std::string marker_topic_ = "marker";
  std::string marker_ns_ = "interceptor_uav";
  std::string mesh_resource_;                // 网格资源路径
  bool mesh_use_embedded_materials_ = true;  // 是否使用嵌入材质
  double mesh_scale_x_ = 1.0;
  double mesh_scale_y_ = 1.0;
  double mesh_scale_z_ = 1.0;
  double marker_color_r_ = 0.0;   // 默认蓝色（R=0, G=0.35, B=1.0）
  double marker_color_g_ = 0.35;
  double marker_color_b_ = 1.0;
  double marker_color_a_ = 1.0;

  // 当前状态
  Vec3 pos_;   // 当前位置
  Vec3 vel_;   // 当前速度

  // 期望状态（来自命令）
  Vec3 desired_pos_;
  Vec3 desired_vel_;
  Vec3 desired_acc_;
  double desired_yaw_ = 0.0;

  // 当前偏航角
  double yaw_ = 0.0;
  bool has_cmd_ = false;  // 是否收到过命令

  // 时间管理
  ros::Time last_cmd_time_;
  ros::Time last_update_time_;
};

/**
 * @brief 主函数：初始化节点，创建拦截器无人机节点实例，进入ROS事件循环
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "interceptor_uav_node");
  InterceptorUavNode node;
  ros::spin();
  return 0;
}