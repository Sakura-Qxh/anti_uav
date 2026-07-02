// bezier_predict_node.cpp
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>  // 包含 tanh 函数所需头文件
#include <vector>

// 这里需要包含贝塞尔预测算法的头文件，假设该头文件定义了 Bezierpredict 类
#include <bezier_prediction/bezier_predict.h>  // Bezier曲线预测器

// 定义使用的最大历史数据点数，应与 bezier_predict.cpp 中 _MAX_SEG 保持一致
#define MAX_SEG 50
#define _PREDICT_SEG 30

class BezierPredictNode {
   public:
    BezierPredictNode(ros::NodeHandle& nh) {
        // 订阅目标检测信息（均值估计结果），消息类型 nav_msgs::Odometry
        target_sub_ = nh.subscribe("/fxxzxec_estimator_target_target", 10, &BezierPredictNode::targetCallback, this);
        // 新增：订阅当前位置消息，格式同/mean_estimator_target，话题名为 /mean_center_target
        center_sub_ = nh.subscribe("/mean_center_target", 10, &BezierPredictNode::centerCallback, this);
        // 发布预测目标位置，话题名为 /mean_predict_target，消息格式与/mean_estimator_target相同
        predict_pub_ = nh.advertise<nav_msgs::Odometry>("/mean_predict_target", 10);

        // 初始化当前位置为零向量
        center_position_ = Eigen::Vector3d::Zero();
    }

   private:
    // 回调函数：保存目标检测数据，当达到 MAX_SEG 个时调用预测函数
    void targetCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 将接收到的目标状态转换为4维向量： [x, y, z, time]
        Eigen::Vector4d target_state;
        target_state << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, msg->header.stamp.toSec();
        // 保存到缓冲区
        target_buffer_.push_back(target_state);

        // 若缓冲区超过设定数量则删除最旧的
        if (target_buffer_.size() > MAX_SEG) {
            target_buffer_.erase(target_buffer_.begin());
        }

        // 如果缓冲区数据足够，则调用贝塞尔预测算法
        if (target_buffer_.size() == MAX_SEG) {
            // 给定最大速度与加速度（可根据实际需求调整）
            double max_vel = 5.0;
            double max_acc = 5.0;
            // 调用 TrackingGeneration 进行贝塞尔轨迹预测
            int ierr = predictor_.TrackingGeneration(max_vel, max_acc, target_buffer_);
            static std::vector<Eigen::Matrix<double, 6, 1>> predict_state_list;  // 每个元素包含 [x, y, z, vx, vy, vz]
            if (ierr == 0) {
                // 获取预测状态列表，数量为 _PREDICT_SEG 个
                predict_state_list = predictor_.getStateListFromBezier(_PREDICT_SEG);

                // 计算当前位置与目标检测位置之间的距离 d
                // 此处取当前目标检测数据中的最后一个点作为目标检测位置
                Eigen::Vector3d target_pt = target_buffer_.back().head(3);
                double d = (center_position_ - target_pt).norm();

                // 根据距离计算索引，len 表示预测状态列表最大索引
                int len = static_cast<int>(predict_state_list.size()) - 1;
                // 计算 index = floor( len * tanh(d) )
                int index = static_cast<int>(std::floor(static_cast<double>(len) * 0.3 * std::tanh(d)));
                // 保证 index 不超过数组范围
                if (index < 0)
                    index = 0;
                if (index > len)
                    index = len;

                // 使用预测状态列表中 index 处的状态（取前3个分量作为位置）发布预测消息
                Eigen::Vector3d pred_point = predict_state_list[index].head(3);
                nav_msgs::Odometry pred_msg;
                pred_msg.header.stamp = ros::Time::now();
                pred_msg.header.frame_id = msg->header.frame_id;  // 保持与接收消息一致
                pred_msg.pose.pose.position.x = pred_point(0);
                pred_msg.pose.pose.position.y = pred_point(1);
                pred_msg.pose.pose.position.z = pred_point(2);

                predict_pub_.publish(pred_msg);
            } else {
                ROS_WARN("Bezier prediction error: %d", ierr);
            }
        }
    }

    // 新增回调函数：接收当前位置消息（/mean_center_target），更新中心位置
    void centerCallback(const nav_msgs::Odometry::ConstPtr& msg) { center_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z; }

    ros::Subscriber target_sub_;  // 目标检测话题订阅
    ros::Subscriber center_sub_;  // 当前位置话题订阅
    ros::Publisher predict_pub_;  // 预测消息发布

    // 缓冲区用于存储最近的目标检测数据，每个数据为4维向量 [x, y, z, time]
    std::vector<Eigen::Vector4d> target_buffer_;

    // 当前的位置，用于计算距离
    Eigen::Vector3d center_position_;

    // 贝塞尔预测对象（需要在 bezier_predict.h 中定义 Bezierpredict 类及其 TrackingGeneration 成员函数）
    Bezierpredict predictor_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bezier_predict_node");
    ros::NodeHandle nh;
    BezierPredictNode node(nh);
    ros::spin();
    return 0;
}
