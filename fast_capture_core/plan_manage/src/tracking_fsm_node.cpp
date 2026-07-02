/*
该程序是一个基于ROS的无人机轨迹规划与跟踪系统，核心功能包括：
实时轨迹规划：根据目标位置和障碍物地图生成安全飞行路径。
状态机管理：处理紧急停止、重新定位、目标跟踪三种状态。
可视化与通信：发布轨迹数据到控制器，并通过ROS话题进行可视化。

输入：
/target：目标检测信息（nav_msgs/Odometry）。
/odom：无人机里程计（位置、姿态）。

输出：
/trajectory：优化后的轨迹（quadrotor_msgs/PolynomialTrajectory）。
可视化话题：轨迹、预测路径（visualization_msgs/Marker）。
*/
#include <plan_manage/plan_manage.h>
int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_fsm_node");
    // ros::MultiThreadedSpinner spinner(16);
    ros::NodeHandle nh_priv("~");
    plan_manage tracking_fsm(nh_priv);
    // spinner.spin();
    ros::spin();
    return 0;
}