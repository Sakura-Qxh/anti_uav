#ifndef _PLANMANAGE_H
#define _PLANMANAGE_H

#include <bezier_prediction/bezier_predict.h>         // Bezier曲线预测器
#include <grid_path_searcher/hybridAstar_searcher.h>  // 混合A*搜索器
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>  // 轨迹消息类型
#include <quadrotor_msgs/PositionCommand.h>       // 位姿指令消息类型
#include <ros/ros.h>
#include <sfc_generation/sfc.h>  // 生成安全飞行走廊 (SFC)
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <vector>
#include "Optraj.h"  // 优化轨迹相关类

#define BUDGET_TIME 0.06  // 轨迹规划时的时间预算（秒）
// #define _MAX_SEG 10       // 目标观测的最大段数（原代码中已使用）

// 有限状态机的三种状态
enum STATE_TYPE {
    EMERGENCY,   // 紧急避障或停止
    RELOCATING,  // 重新定位（目标丢失等情况）
    TRACKING     // 正常跟踪
};

// 用于存储其他无人机信息
struct OtherDroneInfo {
    bool valid = false;  // 是否已经收到过消息
    nav_msgs::Odometry odom;
    quadrotor_msgs::PolynomialTrajectory traj;
};

class plan_manage {
   private:
    ros::NodeHandle nh_global;  // 全局节点句柄作为成员变量
    // --- 1) 跟踪与规划相关的工具类对象 ---
    hybridAstar_searcher kinosearch;  // 混合A*搜索器
    Bezierpredict tgpredict;          // Bezier预测器
    FlightCorridor sfc;               // 安全飞行走廊生成器
    GridMap::Ptr env;                 // 栅格地图环境指针

    // --- 2) 系统自身状态变量 ---
    Eigen::Vector3d position;  // 无人机当前位置信息
    Eigen::Quaterniond q;      // 无人机当前姿态（四元数）
    double yaw;                // 当前航向角

    // --- 3) 目标检测相关 ---
    // 这里存储了若干历史检测到的目标位置及时间 [px, py, pz, t]
    std::vector<Eigen::Vector4d> target_detect_list;
    // 也可以额外存储当前目标位置，方便排序
    Eigen::Vector3d target_position_ = Eigen::Vector3d::Zero();

    // --- 4) 轨迹相关变量 ---
    quadrotor_msgs::PolynomialTrajectory traj_msg;  // 要发布的轨迹消息
    OpTrajectory traj;                              // 本次规划得到的轨迹
    OpTrajectory last_traj;                         // 上一次规划的轨迹
    double last_rcvtime;                            // 上一次接收到目标信息的时间

    // --- 5) 有限状态机相关 ---
    bool relocate_init = false;  // 标记是否已完成一次RELOCATING状态
    bool init_flag = false;      // 标记是否已完成初次规划
    STATE_TYPE fsm;              // 当前FSM状态

    // --- 6) 参数 ---
    double v_max;                    // 最大速度
    double a_max;                    // 最大加速度
    double replan_frequency = 12.0;  // 重规划频率（Hz）
    double mytarget_h = 1.2;
    int rank;

    // --- 7) ROS 相关 ---
    ros::Timer replan_timer;               // 定时器，用于周期性执行fsm_timer_cb
    ros::Subscriber detect_sub, odom_sub;  // 订阅器：目标检测和自身odom
    ros::Publisher kino_search_vispub, traj_vispub, pretraj_vispub, cor_vispub;
    ros::Publisher TrackTrajPub;  // 发布轨迹
    ros::Publisher height_pub_;   // 在类定义中添加发布者成员变量

    // === 多无人机相关 ===
    int drone_id_;                                                    // 本机编号
    int drone_num_;                                                   // 集群中无人机总数
    std::vector<OtherDroneInfo> other_drones_;                        // 存储其他无人机信息
    std::vector<ros::Subscriber> other_odom_subs_, other_traj_subs_;  // 订阅器列表
    double safe_distance_ = 1.2;
    /**
+     * @brief 根据其他无人机发布的 PolynomialTrajectory 轨迹消息，在绝对时刻 t_abs（秒）上插值出位置
+     * @param other_id  其他无人机编号
+     * @param t_abs     绝对 ros::Time（toSec()）时间
+     * @return 对应时刻的位置 (x,y,z)
+     */
    Eigen::Vector3d evaluateOtherTrajectory(int other_id, double t_abs) const;  // 简易安全距离阈值

   public:
    // --- 构造与析构 ---
    plan_manage(ros::NodeHandle& nh);
    ~plan_manage() {}

    // --- 回调函数 ---
    void fsm_timer_cb(const ros::TimerEvent& event);       // 有限状态机定时器回调
    void tg_list_cb(const nav_msgs::Odometry& car_state);  // 接收目标 (car) 的状态
    void odom_cb(const nav_msgs::Odometry& odom);          // 接收无人机自身odom

    // 用于接收其他无人机的里程计和轨迹
    void otherDroneOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, int other_id);
    void otherDroneTrajectoryCallback(const quadrotor_msgs::PolynomialTrajectory::ConstPtr& traj_msg, int other_id);

    // --- 轨迹评估函数 (evalute) ---
    // 根据时间t获取上一次轨迹的 位置/速度/加速度
    Eigen::Vector3d evaluteP(double t);
    Eigen::Vector3d evaluteV(double t);
    Eigen::Vector3d evaluteA(double t);

    // 将OpTrajectory格式转化为多项式轨迹消息
    quadrotor_msgs::PolynomialTrajectory traj2msg(OpTrajectory traj);

    // --- 可视化 ---
    void visualize_pre(std::vector<Eigen::Vector3d> poslist);
    void visualize_traj(OpTrajectory traj);
    void visualize_relocate(OpTrajectory traj);

    // === 新增功能：多无人机距离排序 & 碰撞检测 ===
    int computePriorityRank();                                      // 计算本机相对于目标的排序优先级
    bool checkCollision(const std::vector<Eigen::Vector3d>& path);  // 简易碰撞检测
};

#endif
