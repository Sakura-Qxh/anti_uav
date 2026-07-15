#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

// 计算三维空间中两点之间的欧氏距离
double distance3(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 将数值限制在 [low, high] 范围内
double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

// 存储载具状态（里程计数据 + 时间戳 + 有效性标志）
struct VehicleState {
    nav_msgs::Odometry odom;
    ros::Time stamp;
    bool valid = false;
};

// 存储威胁状态（威胁值 + 时间戳 + 有效性标志）
struct ThreatState {
    double value = 0.0;
    ros::Time stamp;
    bool valid = false;
};

// 包围任务的状态枚举
enum class EncirclementState {
    APPROACH,      // 接近目标群
    FORM_RING,     // 形成包围环
    ROTATE_TRACK   // 旋转跟踪
};

// 将状态枚举转换为可读字符串
std::string stateName(EncirclementState state) {
    switch (state) {
        case EncirclementState::APPROACH: return "APPROACH";
        case EncirclementState::FORM_RING: return "FORM_RING";
        case EncirclementState::ROTATE_TRACK: return "ROTATE_TRACK";
    }
    return "APPROACH";
}

}  // namespace

// 协同制导节点类：管理多个拦截器对多个目标的包围引导
class CooperativeGuidanceNode {
public:
    CooperativeGuidanceNode() : nh_(), pnh_("~") {
        loadParams();               // 加载参数
        targets_.resize(target_count_);          // 初始化目标数组
        threats_.resize(target_count_);          // 初始化威胁数组
        interceptors_.resize(interceptor_count_);// 初始化拦截器数组
        slot_goals_.resize(interceptor_count_);  // 初始化槽位目标点数组
        interceptor_to_slot_.resize(interceptor_count_); // 初始化拦截器到槽位的映射
        last_published_goals_.resize(interceptor_count_); // 上次发布的航点
        last_goal_publish_times_.resize(interceptor_count_); // 上次发布航点的时间
        has_published_goal_.assign(interceptor_count_, false); // 是否已发布过航点
        createRosInterfaces();      // 创建ROS订阅和发布接口

        last_timer_time_ = ros::Time::now();
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(update_rate_, 1e-3)),
            &CooperativeGuidanceNode::timerCallback,
            this);

        ROS_INFO("moving encirclement guidance: targets=%d interceptors=%d",
                 target_count_, interceptor_count_);
    }

private:
    // 从参数服务器加载所有配置参数
    void loadParams() {
        pnh_.param<std::string>("world_frame", world_frame_, "world");
        pnh_.param<int>("target_count", target_count_, 3);
        pnh_.param<int>("interceptor_count", interceptor_count_, 4);
        pnh_.param<double>("update_rate", update_rate_, 20.0);
        pnh_.param<std::string>("goal_prefix", goal_prefix_, "/interceptor_");
        pnh_.param<std::string>("target_prefix", target_prefix_, "/target_");
        pnh_.param<std::string>("interceptor_prefix", interceptor_prefix_, "/interceptor_");
        pnh_.param<std::string>("threat_prefix", threat_prefix_, "/threat/target_");
        pnh_.param<double>("stale_timeout", stale_timeout_, 1.0);
        pnh_.param<double>("lead_time", lead_time_, 0.5);
        pnh_.param<double>("safety_margin", safety_margin_, 5.0);
        pnh_.param<double>("min_radius", min_radius_, 8.0);
        pnh_.param<double>("max_radius", max_radius_, 15.0);
        pnh_.param<double>("angular_velocity", angular_velocity_, 0.0);
        pnh_.param<double>("ring_entry_tolerance", ring_entry_tolerance_, 1.5);
        pnh_.param<double>("ring_exit_tolerance", ring_exit_tolerance_, 3.5);
        pnh_.param<double>("ring_hold_time", ring_hold_time_, 1.0);
        pnh_.param<double>("approach_reassign_timeout", approach_reassign_timeout_, 30.0);
        pnh_.param<double>("threat_start_threshold", threat_start_threshold_, 0.05);
        pnh_.param<double>("min_goal_altitude", min_goal_altitude_, 2.0);
        pnh_.param<double>("goal_publish_rate", goal_publish_rate_, 2.0);
        pnh_.param<double>("goal_position_threshold", goal_position_threshold_, 0.3);
        pnh_.param<int>("phase_search_steps", phase_search_steps_, 36);
        pnh_.param<double>("marker_scale", marker_scale_, 0.8);
        pnh_.param<double>("line_width", line_width_, 0.12);

        if (target_count_ != 3 || interceptor_count_ != 4) {
            ROS_WARN("this guidance is designed for target_count=3, interceptor_count=4");
        }
        if (min_radius_ > max_radius_) {
            std::swap(min_radius_, max_radius_);
        }
        goal_publish_rate_ = std::max(0.1, goal_publish_rate_);
        goal_position_threshold_ = std::max(0.0, goal_position_threshold_);
        phase_search_steps_ = std::max(4, phase_search_steps_);
    }

    // 创建所有订阅器和发布器
    void createRosInterfaces() {
        // 订阅每个目标的里程计话题
        for (int i = 0; i < target_count_; ++i) {
            target_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                target_prefix_ + std::to_string(i) + "/estimate",
                10,
                [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    targetCallback(msg, i);
                }));
            // 订阅每个目标的威胁话题
            threat_subs_.push_back(nh_.subscribe<std_msgs::Float64>(
                threat_prefix_ + std::to_string(i),
                10,
                [this, i](const std_msgs::Float64::ConstPtr& msg) {
                    threatCallback(msg, i);
                }));
        }

        // 订阅每个拦截器的里程计话题，并为每个拦截器创建航点发布器
        for (int i = 0; i < interceptor_count_; ++i) {
            interceptor_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                interceptor_prefix_ + std::to_string(i) + "/odom",
                10,
                [this, i](const nav_msgs::Odometry::ConstPtr& msg) {
                    interceptorCallback(msg, i);
                }));
            capture_goal_pubs_.push_back(
                nh_.advertise<geometry_msgs::PoseStamped>(
                    goal_prefix_ + std::to_string(i) + "/capture_goal", 1, true));
        }

        // 可视化标记、圆心、半径和状态的发布器
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/cooperative_guidance/markers", 1);
        center_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
            "/cooperative_guidance/center", 10);
        radius_pub_ = nh_.advertise<std_msgs::Float64>(
            "/cooperative_guidance/radius", 10);
        state_pub_ = nh_.advertise<std_msgs::String>(
            "/cooperative_guidance/state", 10);
    }

    // 目标里程计回调：更新对应目标的状态
    void targetCallback(const nav_msgs::Odometry::ConstPtr& msg, int index) {
        if (index < 0 || index >= target_count_) return;
        targets_[index].odom = *msg;
        targets_[index].stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        targets_[index].valid = true;
    }

    // 威胁值回调：更新对应目标的威胁值（限制在0~1之间）
    void threatCallback(const std_msgs::Float64::ConstPtr& msg, int index) {
        if (index < 0 || index >= target_count_) return;
        threats_[index].value = clamp(msg->data, 0.0, 1.0);
        threats_[index].stamp = ros::Time::now();
        threats_[index].valid = true;
    }

    // 拦截器里程计回调：更新对应拦截器的状态
    void interceptorCallback(const nav_msgs::Odometry::ConstPtr& msg, int index) {
        if (index < 0 || index >= interceptor_count_) return;
        interceptors_[index].odom = *msg;
        interceptors_[index].stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        interceptors_[index].valid = true;
    }

    // 检查车辆状态是否新鲜（有效且未超时）
    bool fresh(const VehicleState& state, const ros::Time& now) const {
        return state.valid && (now - state.stamp).toSec() <= stale_timeout_;
    }

    // 检查所有目标是否都新鲜
    bool allTargetsFresh(const ros::Time& now) const {
        return std::all_of(targets_.begin(), targets_.end(),
                           [this, &now](const VehicleState& target) {
                               return fresh(target, now);
                           });
    }

    // 检查所有威胁是否都新鲜
    bool allThreatsFresh(const ros::Time& now) const {
        return std::all_of(threats_.begin(), threats_.end(),
                           [this, &now](const ThreatState& threat) {
                               return threat.valid &&
                                   (now - threat.stamp).toSec() <= stale_timeout_;
                           });
    }

    // 检查所有拦截器是否都新鲜
    bool allInterceptorsFresh(const ros::Time& now) const {
        return std::all_of(interceptors_.begin(), interceptors_.end(),
                           [this, &now](const VehicleState& interceptor) {
                               return fresh(interceptor, now);
                           });
    }

    // 计算目标群的几何中心、平均速度和包围半径
    void computeGroup(geometry_msgs::Point& center,
                      geometry_msgs::Vector3& velocity,
                      double& radius) const {
        // 累加所有目标的位置和速度
        for (const auto& target : targets_) {
            center.x += target.odom.pose.pose.position.x;
            center.y += target.odom.pose.pose.position.y;
            center.z += target.odom.pose.pose.position.z;
            velocity.x += target.odom.twist.twist.linear.x;
            velocity.y += target.odom.twist.twist.linear.y;
            velocity.z += target.odom.twist.twist.linear.z;
        }
        const double count = static_cast<double>(target_count_);
        center.x /= count;
        center.y /= count;
        center.z /= count;
        velocity.x /= count;
        velocity.y /= count;
        velocity.z /= count;

        // 取最大距离作为群半径，并加上安全裕度，再限制到[min_radius, max_radius]
        double group_radius = 0.0;
        for (const auto& target : targets_) {
            group_radius = std::max(
                group_radius,
                distance3(target.odom.pose.pose.position, center));
        }
        radius = clamp(group_radius + safety_margin_, min_radius_, max_radius_);
    }

    // 获取当前最大的威胁值（用于判断是否需要启动包围）
    double groupThreat() const {
        double value = 0.0;
        for (const auto& threat : threats_) value = std::max(value, threat.value);
        return value;
    }

    // 根据预测的中心位置、半径和当前相位，为每个拦截器生成槽位目标点
    void makeSlotGoals(const geometry_msgs::Point& predicted_center,
                       double radius,
                       const ros::Time& stamp) {
        for (int i = 0; i < interceptor_count_; ++i) {
            const int slot = interceptor_to_slot_[i]; // 当前拦截器对应的槽位编号
            const double angle = phase_ + 2.0 * kPi * static_cast<double>(slot) /
                static_cast<double>(interceptor_count_);
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = stamp;
            goal.header.frame_id = world_frame_;
            goal.pose.position.x = predicted_center.x + radius * std::cos(angle);
            goal.pose.position.y = predicted_center.y + radius * std::sin(angle);
            goal.pose.position.z = std::max(min_goal_altitude_, predicted_center.z);
            goal.pose.orientation.w = 1.0;
            slot_goals_[i] = goal;
        }
    }

    // 辅助函数：计算某个槽位的具体位置（不保存，用于代价计算）
    geometry_msgs::Point slotPoint(const geometry_msgs::Point& center,
                                   double radius,
                                   double phase,
                                   int slot) const {
        const double angle = phase + 2.0 * kPi * static_cast<double>(slot) /
            static_cast<double>(interceptor_count_);
        geometry_msgs::Point point;
        point.x = center.x + radius * std::cos(angle);
        point.y = center.y + radius * std::sin(angle);
        point.z = std::max(min_goal_altitude_, center.z);
        return point;
    }

    // 通过最小化总距离代价，为拦截器分配最优槽位和初始相位
    bool assignSlotsByMinimumCost(const geometry_msgs::Point& center,
                                  double radius,
                                  const ros::Time& now) {
        if (!allInterceptorsFresh(now)) return false;

        std::vector<int> permutation(interceptor_count_);
        std::iota(permutation.begin(), permutation.end(), 0);
        std::vector<int> best_assignment = permutation;
        double best_phase = 0.0;
        double best_cost = std::numeric_limits<double>::infinity();

        // 遍历不同相位偏移和所有排列组合，选择总距离最小的方案
        for (int phase_index = 0; phase_index < phase_search_steps_; ++phase_index) {
            const double candidate_phase = 2.0 * kPi * static_cast<double>(phase_index) /
                static_cast<double>(phase_search_steps_);
            std::iota(permutation.begin(), permutation.end(), 0);
            do {
                double cost = 0.0;
                for (int interceptor = 0; interceptor < interceptor_count_; ++interceptor) {
                    const geometry_msgs::Point point = slotPoint(
                        center, radius, candidate_phase, permutation[interceptor]);
                    cost += distance3(interceptors_[interceptor].odom.pose.pose.position, point);
                }
                if (cost < best_cost) {
                    best_cost = cost;
                    best_phase = candidate_phase;
                    best_assignment = permutation;
                }
            } while (std::next_permutation(permutation.begin(), permutation.end()));
        }

        phase_ = best_phase;
        interceptor_to_slot_ = best_assignment;
        phase_initialized_ = true;
        assignment_valid_ = true;
        approach_start_ = now;
        ROS_INFO("encirclement slots assigned: phase=%.3f total_distance=%.3f",
                 phase_, best_cost);
        return true;
    }

    // 判断是否需要重新发布航点（基于频率和位置变化阈值）
    bool shouldPublishGoal(int index,
                           const geometry_msgs::PoseStamped& goal,
                           const ros::Time& now,
                           bool force) const {
        if (force || !has_published_goal_[index]) return true;
        const double min_period = 1.0 / goal_publish_rate_;
        if ((now - last_goal_publish_times_[index]).toSec() < min_period) return false;
        return distance3(last_published_goals_[index].pose.position,
                         goal.pose.position) >= goal_position_threshold_;
    }

    // 发布所有拦截器的航点（如果满足条件）
    void publishCaptureGoals(const ros::Time& now, bool force) {
        for (int i = 0; i < interceptor_count_; ++i) {
            if (!shouldPublishGoal(i, slot_goals_[i], now, force)) continue;
            capture_goal_pubs_[i].publish(slot_goals_[i]);
            last_published_goals_[i] = slot_goals_[i];
            last_goal_publish_times_[i] = now;
            has_published_goal_[i] = true;
        }
    }

    // 计算每个拦截器当前位置与对应槽位目标之间的距离误差
    std::vector<double> slotErrors(const ros::Time& now) const {
        std::vector<double> errors(interceptor_count_, 1e9);
        for (int i = 0; i < interceptor_count_; ++i) {
            if (fresh(interceptors_[i], now)) {
                errors[i] = distance3(interceptors_[i].odom.pose.pose.position,
                                      slot_goals_[i].pose.position);
            }
        }
        return errors;
    }

    // 检查所有误差是否均小于等于阈值
    bool allBelow(const std::vector<double>& values, double threshold) const {
        return std::all_of(values.begin(), values.end(),
                           [threshold](double value) { return value <= threshold; });
    }

    // 检查是否存在任何误差大于阈值
    bool anyAbove(const std::vector<double>& values, double threshold) const {
        return std::any_of(values.begin(), values.end(),
                           [threshold](double value) { return value > threshold; });
    }

    // 根据误差更新包围状态机
    bool updateState(const std::vector<double>& errors, const ros::Time& now) {
        if (state_ == EncirclementState::APPROACH) {
            // 所有拦截器进入容差范围 → 切换到 FORM_RING
            if (allBelow(errors, ring_entry_tolerance_)) {
                state_ = EncirclementState::FORM_RING;
                hold_start_ = now;
                ROS_INFO("encirclement state -> FORM_RING");
                return true;
            }
            return false;
        }

        if (state_ == EncirclementState::FORM_RING) {
            // 如果有拦截器超出退出容差 → 退回 APPROACH
            if (anyAbove(errors, ring_exit_tolerance_)) {
                state_ = EncirclementState::APPROACH;
                approach_start_ = now;
                ROS_WARN("encirclement state -> APPROACH (ring lost)");
                return true;
            } else if (!allBelow(errors, ring_entry_tolerance_)) {
                // 尚未全部进入紧容差，重置保持计时
                hold_start_ = now;
            } else if ((now - hold_start_).toSec() >= ring_hold_time_) {
                // 稳定保持一段时间后 → 切换到 ROTATE_TRACK
                state_ = EncirclementState::ROTATE_TRACK;
                ROS_INFO("encirclement state -> ROTATE_TRACK");
                return true;
            }
            return false;
        }

        if (state_ == EncirclementState::ROTATE_TRACK &&
            anyAbove(errors, ring_exit_tolerance_)) {
            // 旋转跟踪过程中若误差过大 → 退回 APPROACH
            state_ = EncirclementState::APPROACH;
            approach_start_ = now;
            ROS_WARN("encirclement state -> APPROACH (slot error)");
            return true;
        }
        return false;
    }

    // 发布圆心、半径和状态文本（用于调试和监控）
    void publishStatus(const geometry_msgs::Point& center,
                       double radius,
                       const std::string& state_text,
                       const ros::Time& now) {
        geometry_msgs::PointStamped center_msg;
        center_msg.header.stamp = now;
        center_msg.header.frame_id = world_frame_;
        center_msg.point = center;
        center_pub_.publish(center_msg);

        std_msgs::Float64 radius_msg;
        radius_msg.data = radius;
        radius_pub_.publish(radius_msg);

        std_msgs::String state_msg;
        state_msg.data = state_text;
        state_pub_.publish(state_msg);
    }

    // 发布可视化标记（圆心球体、包围圆环、槽位点、误差线、状态文字）
    void publishMarkers(const geometry_msgs::Point& center,
                        double radius,
                        const std::string& state_text,
                        const ros::Time& now) {
        visualization_msgs::MarkerArray markers;

        // 圆心标记（黄色球体）
        visualization_msgs::Marker center_marker;
        center_marker.header.stamp = now;
        center_marker.header.frame_id = world_frame_;
        center_marker.ns = "encirclement_center";
        center_marker.id = 0;
        center_marker.type = visualization_msgs::Marker::SPHERE;
        center_marker.action = visualization_msgs::Marker::ADD;
        center_marker.pose.position = center;
        center_marker.pose.orientation.w = 1.0;
        center_marker.scale.x = marker_scale_;
        center_marker.scale.y = marker_scale_;
        center_marker.scale.z = marker_scale_;
        center_marker.color.r = 1.0;
        center_marker.color.g = 0.8;
        center_marker.color.b = 0.0;
        center_marker.color.a = 1.0;
        markers.markers.push_back(center_marker);

        // 包围圆环（青色线条）
        visualization_msgs::Marker circle;
        circle.header = center_marker.header;
        circle.ns = "encirclement_ring";
        circle.id = 0;
        circle.type = visualization_msgs::Marker::LINE_STRIP;
        circle.action = visualization_msgs::Marker::ADD;
        circle.scale.x = line_width_;
        circle.color.r = 0.0;
        circle.color.g = 0.9;
        circle.color.b = 1.0;
        circle.color.a = 0.9;
        for (int k = 0; k <= 64; ++k) {
            const double angle = 2.0 * kPi * static_cast<double>(k) / 64.0;
            geometry_msgs::Point point;
            point.x = center.x + radius * std::cos(angle);
            point.y = center.y + radius * std::sin(angle);
            point.z = std::max(min_goal_altitude_, center.z);
            circle.points.push_back(point);
        }
        markers.markers.push_back(circle);

        // 每个拦截器的槽位点和误差连线
        for (int i = 0; i < interceptor_count_; ++i) {
            visualization_msgs::Marker slot;
            slot.header = center_marker.header;
            slot.ns = "encirclement_slots";
            slot.id = i;
            slot.type = visualization_msgs::Marker::SPHERE;
            slot.action = visualization_msgs::Marker::ADD;
            slot.pose = slot_goals_[i].pose;
            slot.scale.x = marker_scale_;
            slot.scale.y = marker_scale_;
            slot.scale.z = marker_scale_;
            slot.color.r = 0.0;
            slot.color.g = 0.8;
            slot.color.b = 1.0;
            slot.color.a = 0.9;
            markers.markers.push_back(slot);

            if (interceptors_[i].valid) {
                visualization_msgs::Marker line;
                line.header = center_marker.header;
                line.ns = "slot_error_lines";
                line.id = i;
                line.type = visualization_msgs::Marker::LINE_LIST;
                line.action = visualization_msgs::Marker::ADD;
                line.scale.x = 0.5 * line_width_;
                line.color.r = 0.2;
                line.color.g = 0.7;
                line.color.b = 1.0;
                line.color.a = 0.6;
                line.points.push_back(interceptors_[i].odom.pose.pose.position);
                line.points.push_back(slot_goals_[i].pose.position);
                markers.markers.push_back(line);
            }
        }

        // 状态文字（白色，显示在圆心上方）
        visualization_msgs::Marker text;
        text.header = center_marker.header;
        text.ns = "encirclement_state";
        text.id = 0;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position = center;
        text.pose.position.z += 3.0;
        text.pose.orientation.w = 1.0;
        text.scale.z = 1.2;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.text = state_text;
        markers.markers.push_back(text);

        marker_pub_.publish(markers);
    }

    // 定时器回调：主控制循环，负责状态更新、航点计算和发布
    void timerCallback(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        const double dt = clamp((now - last_timer_time_).toSec(), 0.0, 0.2);
        last_timer_time_ = now;

        // 等待所有目标数据就绪
        if (!allTargetsFresh(now)) {
            publishStatus(geometry_msgs::Point(), 0.0, "WAIT_TARGETS", now);
            return;
        }
        // 等待威胁数据或威胁值低于启动阈值时保持空闲
        if (!allThreatsFresh(now) || groupThreat() < threat_start_threshold_) {
            phase_initialized_ = false;
            assignment_valid_ = false;
            state_ = EncirclementState::APPROACH;
            publishStatus(geometry_msgs::Point(), 0.0, "WAIT_THREAT", now);
            return;
        }

        // 计算目标群中心、平均速度和包围半径
        geometry_msgs::Point center;
        geometry_msgs::Vector3 velocity;
        double radius = min_radius_;
        computeGroup(center, velocity, radius);

        // 根据前导时间预测未来中心位置
        geometry_msgs::Point predicted_center = center;
        predicted_center.x += lead_time_ * velocity.x;
        predicted_center.y += lead_time_ * velocity.y;
        predicted_center.z += lead_time_ * velocity.z;

        if (!allInterceptorsFresh(now)) {
            publishStatus(predicted_center, radius, "WAIT_INTERCEPTORS", now);
            return;
        }

        // 首次分配或重新分配槽位
        if (!assignment_valid_ || !phase_initialized_) {
            if (!assignSlotsByMinimumCost(predicted_center, radius, now)) return;
        } else if (state_ == EncirclementState::APPROACH &&
                   !approach_start_.isZero() &&
                   (now - approach_start_).toSec() >= approach_reassign_timeout_) {
            // APPROACH 状态下超时未进入环，重新分配以优化路径
            assignSlotsByMinimumCost(predicted_center, radius, now);
        }

        // 旋转跟踪阶段：更新相位角
        if (state_ == EncirclementState::ROTATE_TRACK) {
            phase_ += angular_velocity_ * dt;
            if (phase_ > 2.0 * kPi) phase_ -= 2.0 * kPi;
        }

        // 生成槽位目标点，计算误差，更新状态机
        makeSlotGoals(predicted_center, radius, now);
        const auto errors = slotErrors(now);
        const bool state_changed = updateState(errors, now);
        publishCaptureGoals(now, state_changed);

        // 发布状态信息和可视化标记
        const std::string text = stateName(state_);
        publishStatus(predicted_center, radius, text, now);
        publishMarkers(predicted_center, radius, text, now);
    }

    // ROS 句柄和定时器
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;
    ros::Time last_timer_time_;
    ros::Time hold_start_;

    // 配置参数（默认值）
    std::string world_frame_ = "world";
    int target_count_ = 3;
    int interceptor_count_ = 4;
    double update_rate_ = 20.0;
    std::string goal_prefix_ = "/interceptor_";
    std::string target_prefix_ = "/target_";
    std::string interceptor_prefix_ = "/interceptor_";
    std::string threat_prefix_ = "/threat/target_";

    double stale_timeout_ = 1.0;
    double lead_time_ = 0.5;
    double safety_margin_ = 5.0;
    double min_radius_ = 8.0;
    double max_radius_ = 15.0;
    double angular_velocity_ = 0.0;
    double ring_entry_tolerance_ = 1.5;
    double ring_exit_tolerance_ = 3.5;
    double ring_hold_time_ = 1.0;
    double approach_reassign_timeout_ = 30.0;
    double threat_start_threshold_ = 0.05;
    double min_goal_altitude_ = 2.0;
    double goal_publish_rate_ = 2.0;
    double goal_position_threshold_ = 0.3;
    int phase_search_steps_ = 36;
    double marker_scale_ = 0.8;
    double line_width_ = 0.12;

    // 状态变量
    EncirclementState state_ = EncirclementState::APPROACH;
    double phase_ = 0.0;               // 当前环绕相位角
    bool phase_initialized_ = false;   // 相位是否已初始化
    bool assignment_valid_ = false;    // 槽位分配是否有效
    ros::Time approach_start_;         // 进入APPROACH状态的起始时间

    // 数据缓存
    std::vector<VehicleState> targets_;
    std::vector<ThreatState> threats_;
    std::vector<VehicleState> interceptors_;
    std::vector<geometry_msgs::PoseStamped> slot_goals_;
    std::vector<int> interceptor_to_slot_;
    std::vector<geometry_msgs::PoseStamped> last_published_goals_;
    std::vector<ros::Time> last_goal_publish_times_;
    std::vector<bool> has_published_goal_;

    // ROS通信接口
    std::vector<ros::Subscriber> target_subs_;
    std::vector<ros::Subscriber> threat_subs_;
    std::vector<ros::Subscriber> interceptor_subs_;
    std::vector<ros::Publisher> capture_goal_pubs_;
    ros::Publisher marker_pub_;
    ros::Publisher center_pub_;
    ros::Publisher radius_pub_;
    ros::Publisher state_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cooperative_guidance_node");
    CooperativeGuidanceNode node;
    ros::spin();
    return 0;
}