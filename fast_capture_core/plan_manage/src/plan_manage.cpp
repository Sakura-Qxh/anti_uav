#include <plan_manage/plan_manage.h>
#include <std_msgs/Float32.h>

//----------------------------
// 构造函数：初始化环境、订阅与发布、定时器等
//----------------------------
plan_manage::plan_manage(ros::NodeHandle& nh) : nh_global() {
    // 1) 初始化环境地图
    env.reset(new GridMap);
    env->initMap(nh);

    // 2) 设置混合A*搜索器参数、绑定环境
    kinosearch.setParam(nh);
    kinosearch.setEnvironment(env);

    // 3) 安全飞行走廊绑定地图
    sfc.set_map(env);

    // 4) 从ROS参数服务器中获取最大速度、加速度
    nh.param("traj/vmax", v_max, 2.0);
    nh.param("traj/amax", a_max, 3.0);
    nh.param("drone_id_", drone_id_, 0);
    nh.param("drone_num_", drone_num_, 4);
    // 5) 订阅目标检测结果和无人机自身odom
    //    - 目标话题： "target" (nav_msgs::Odometry)
    //    - 无人机话题："odom"   (nav_msgs::Odometry)
    detect_sub = nh.subscribe("mean_estimator_target", 1, &plan_manage::tg_list_cb, this);
    odom_sub = nh.subscribe("odom", 1, &plan_manage::odom_cb, this);

    // 6) 广播一些可视化Marker和轨迹消息
    traj_vispub = nh.advertise<visualization_msgs::Marker>("visualization/vis_smooth_traj", 1);
    pretraj_vispub = nh.advertise<visualization_msgs::Marker>("visualization/vis_pre_traj", 1);
    TrackTrajPub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
    height_pub_ = nh.advertise<std_msgs::Float32>("mytarget_height", 1);  // 新增

    // 7) 创建定时器：根据 replan_frequency 来周期调用 fsm_timer_cb
    replan_timer = nh.createTimer(ros::Duration(1 / replan_frequency), &plan_manage::fsm_timer_cb, this);

    // === 初始化 other_drones_ 容器 ===
    other_drones_.resize(drone_num_);

    // === 为每个其他无人机订阅其 odom & trajectory ===
    ros::NodeHandle nh_global;  // 新增：全局节点句柄
    for (int i = 0; i < drone_num_; i++) {
        if (i == drone_id_)
            continue;  // 跳过自己
        std::string odom_topic = "drone_" + std::to_string(i) + "_visual_slam/odom";
        std::string traj_topic = "drone_" + std::to_string(i) + "_trajectory";
        // 这里用boost::bind把other_id传递到回调
        ros::Subscriber sub_odom = nh_global.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&plan_manage::otherDroneOdomCallback, this, _1, i));
        ros::Subscriber sub_traj = nh_global.subscribe<quadrotor_msgs::PolynomialTrajectory>(traj_topic, 1, boost::bind(&plan_manage::otherDroneTrajectoryCallback, this, _1, i));
        other_odom_subs_.push_back(sub_odom);
        other_traj_subs_.push_back(sub_traj);
    }
    // 其他初始化
    // last_rcvtime = -1.0;
    // fsm = RELOCATING;  // 初始状态可根据需要设定
}

//----------------------------
// odom回调：更新无人机自身的位姿、航向
//----------------------------
void plan_manage::odom_cb(const nav_msgs::Odometry& odom) {
    // 提取位置
    position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;

    // 提取四元数并计算yaw
    q.w() = odom.pose.pose.orientation.w;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    q = q.normalized();
    yaw = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}

//----------------------------
// 目标检测回调：将目标位置(x,y,z)及时间t保存到 target_detect_list
//----------------------------
void plan_manage::tg_list_cb(const nav_msgs::Odometry& car_state) {
    static bool initialize = false;

    // 构造目标状态 [px, py, pz, time]
    Eigen::Vector4d state(car_state.pose.pose.position.x,
                          car_state.pose.pose.position.y,
                          car_state.pose.pose.position.z,
                          car_state.header.stamp.toSec());

    // 初次阶段，将数据push_back直到达到上限
    if (!initialize) {
        target_detect_list.push_back(state);
        if (target_detect_list.size() >= _MAX_SEG) {
            initialize = true;
        }
    } else {
        // 达到上限后，移除最早的，插入最新的
        target_detect_list.erase(target_detect_list.begin());
        target_detect_list.push_back(state);
    }

    // 同时更新成员变量 target_position_，只保存前三个分量
    target_position_ << car_state.pose.pose.position.x,
                        car_state.pose.pose.position.y,
                        car_state.pose.pose.position.z;

    // 更新最后一次接收目标信息的时刻
    last_rcvtime = ros::Time::now().toSec();
}

//----------------------------
// 其他无人机 Odom 回调
//----------------------------
void plan_manage::otherDroneOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg, int other_id) {
    if (other_id < 0 || other_id >= (int)other_drones_.size())
        return;
    other_drones_[other_id].odom = *odom_msg;
    other_drones_[other_id].valid = true;
}

//----------------------------
// 其他无人机 Trajectory 回调
//----------------------------
void plan_manage::otherDroneTrajectoryCallback(const quadrotor_msgs::PolynomialTrajectory::ConstPtr& traj_msg, int other_id) {
    if (other_id < 0 || other_id >= (int)other_drones_.size())
        return;
    other_drones_[other_id].traj = *traj_msg;
    other_drones_[other_id].valid = true;
}

//----------------------------
// 评估函数：从上一次规划的轨迹中取位置
//----------------------------
Eigen::Vector3d plan_manage::evaluteP(double t) {
    if (t >= last_traj.getTotalDuration())
        t = last_traj.getTotalDuration();
    return last_traj.getPos(t);
}

// 同理，评估速度
Eigen::Vector3d plan_manage::evaluteV(double t) {
    if (t >= last_traj.getTotalDuration())
        t = last_traj.getTotalDuration();
    return last_traj.getVel(t);
}

// 同理，评估加速度
Eigen::Vector3d plan_manage::evaluteA(double t) {
    if (t >= last_traj.getTotalDuration())
        t = last_traj.getTotalDuration();
    return last_traj.getAcc(t);
}

//----------------------------
// 可视化轨迹 (黑色Sphere列表)
//----------------------------
void plan_manage::visualize_traj(OpTrajectory traj) {
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "world";
    _traj_vis.ns = "/tracking_traj";
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.1;
    _traj_vis.scale.y = 0.1;
    _traj_vis.scale.z = 0.1;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;
    if (drone_id_ == 0) {
        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 0.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 0.0;
    } else if (drone_id_ == 1) {
        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 1.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 0.0;
    } else if (drone_id_ == 2) {
        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 0.0;
        _traj_vis.color.g = 1.0;
        _traj_vis.color.b = 0.0;
    } else if (drone_id_ == 3) {
        _traj_vis.color.a = 1.0;
        _traj_vis.color.r = 0.0;
        _traj_vis.color.g = 0.0;
        _traj_vis.color.b = 1.0;
    }
    _traj_vis.id = 0;
    // 采样轨迹上的点
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    for (double t = 0; t < traj.getTotalDuration(); t += 0.01) {
        pos = traj.getPos(t);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _traj_vis.points.push_back(pt);
    }
    traj_vispub.publish(_traj_vis);
}

// 与上类似，但轨迹显示为红色 (RELOCATING时使用)
void plan_manage::visualize_relocate(OpTrajectory traj) {
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "world";
    _traj_vis.ns = "/tracking_traj";
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.1;
    _traj_vis.scale.y = 0.1;
    _traj_vis.scale.z = 0.1;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;  // 红色

    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    for (double t = 0; t < traj.getTotalDuration(); t += 0.01) {
        pos = traj.getPos(t);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _traj_vis.points.push_back(pt);
    }
    traj_vispub.publish(_traj_vis);
}

// 可视化预测轨迹 (绿色)
void plan_manage::visualize_pre(std::vector<Eigen::Vector3d> poslist) {
    visualization_msgs::Marker _pred_vis;
    _pred_vis.header.stamp = ros::Time::now();
    _pred_vis.header.frame_id = "world";
    _pred_vis.ns = "/tracking_pred";
    _pred_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _pred_vis.action = visualization_msgs::Marker::ADD;
    _pred_vis.scale.x = 0.1;
    _pred_vis.scale.y = 0.1;
    _pred_vis.scale.z = 0.1;
    _pred_vis.pose.orientation.x = 0.0;
    _pred_vis.pose.orientation.y = 0.0;
    _pred_vis.pose.orientation.z = 0.0;
    _pred_vis.pose.orientation.w = 1.0;
    _pred_vis.color.a = 1.0;
    _pred_vis.color.r = 0.0;
    _pred_vis.color.g = 1.0;
    _pred_vis.color.b = 0.0;  // 绿色
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    for (int i = 0; i < poslist.size(); i++) {
        pos = poslist[i];
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _pred_vis.points.push_back(pt);
    }
    pretraj_vispub.publish(_pred_vis);
}

//----------------------------
// 将优化轨迹 (OpTrajectory) 转为 PolynomialTrajectory 消息
//----------------------------
quadrotor_msgs::PolynomialTrajectory plan_manage::traj2msg(OpTrajectory traj) {
    static int count = 0;  // 用于给轨迹编号
    quadrotor_msgs::PolynomialTrajectory traj_msg;

    traj_msg.header.seq = count;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "world";
    traj_msg.trajectory_id = count;
    traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

    // 每段多项式的阶数
    traj_msg.num_order = traj[0].getOrder();
    // 分段数量
    traj_msg.num_segment = traj.getPieceNum();

    // 起止航向(这里设为0)
    traj_msg.start_yaw = 0;
    traj_msg.final_yaw = 0;

    // 遍历轨迹中的每一段
    for (unsigned int i = 0; i < traj_msg.num_segment; i++) {
        // 将每段的系数矩阵写入消息
        for (unsigned int j = 0; j <= traj[i].getOrder(); j++) {
            Eigen::Matrix<double, 3, 6> coemat = traj[i].normalizedCoeffMat();
            traj_msg.coef_x.push_back(coemat(0, j));
            traj_msg.coef_y.push_back(coemat(1, j));
            traj_msg.coef_z.push_back(coemat(2, j));
        }
        // 记录每段的持续时间、阶数
        traj_msg.time.push_back(traj[i].getDuration());
        traj_msg.order.push_back(traj[i].getOrder());
    }

    traj_msg.mag_coeff = 1;
    count++;
    return traj_msg;
}

//----------------------------
// 计算本机到目标距离的排序优先级
//----------------------------
int plan_manage::computePriorityRank() {
    std::vector<std::pair<int, double>> dist_list;
    dist_list.reserve(drone_num_);

    for (int i = 0; i < drone_num_; i++) {
        double d;
        if (i == drone_id_) {
            // 自己：用当前 position
            d = (position - target_position_).norm();
            d = fabs(d);
        } else if (other_drones_[i].valid) {
            // 只有 valid 的无人机才计算实际距离
            const auto& p = other_drones_[i].odom.pose.pose.position;
            Eigen::Vector3d op(p.x, p.y, p.z);
            d = (op - target_position_).norm();
            d = fabs(d);
            // debug 输出
        } else {
            // 尚未收到该无人机信息：赋予无限大，排在最后
            d = std::numeric_limits<double>::infinity();
        }
        dist_list.emplace_back(i, d);
    }

    // 按距离从小到大排序
    std::sort(dist_list.begin(), dist_list.end(), [](auto& a, auto& b) { return a.second < b.second; });

    // 找到自己在排序后的位置
    for (int rank = 0; rank < (int)dist_list.size(); rank++) {
        if (dist_list[rank].first == drone_id_) {
            ROS_INFO("[plan_manage] drone %d priority rank = %d", drone_id_, rank);
            return rank;
        }
    }

    // 兜底
    return drone_num_ - 1;
}

//----------------------------
// 根据其他无人机的 PolynomialTrajectory 在 t_abs 时刻插值位置
//----------------------------
/**
 * @brief 评估其他无人机的多项式轨迹，计算指定绝对时间点的预测位置
 * @param other_id 目标无人机的ID编号
 * @param t_abs 绝对时间戳（单位：秒）
 * @return Eigen::Vector3d 预测的三维位置坐标(x,y,z)
 *
 * 函数执行流程：
 * 1. 有效性检查：当轨迹无效时返回当前odom位置
 * 2. 时间转换：计算相对于轨迹起始时间的相对时间
 * 3. 轨迹分段定位：确定目标时间所在的轨迹段落
 * 4. 多项式计算：根据系数进行三维坐标插值
 */
Eigen::Vector3d plan_manage::evaluateOtherTrajectory(int other_id, double t_abs) const {
    const auto& traj = other_drones_[other_id].traj;
    // 相对时间（sec）
    // 检查轨迹是否有效
    if (traj.num_segment <= 0 || traj.time.size() != traj.num_segment) {
        // 无效轨迹，返回该无人机的当前odom位置
        const auto& odom = other_drones_[other_id].odom;
        return Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    }
    // 计算相对时间 = 绝对时间 - 轨迹起始时间
    double t_rel = t_abs - traj.header.stamp.toSec();
    if (t_rel < 0)
        t_rel = 0;
    /* 轨迹分段定位 */
    double accum = 0.0;  // 累计时间
    int seg = 0;         // 当前段索引
    // 遍历所有轨迹段，找到包含目标时间的段落
    for (; seg < traj.num_segment; ++seg) {
        if (t_rel <= accum + traj.time[seg])  // 判断是否在当前时间段内
            break;
        accum += traj.time[seg];  // 累计已过去的时间
    }
    // 处理超出最后一段的情况
    if (seg >= traj.num_segment) {
        seg = traj.num_segment - 1;  // 定位到最后一段
        t_rel = traj.time[seg];      // 取该段最大时间
    } else {
        t_rel = t_rel - accum;  // 计算段内相对时间
    }
    // 段内多项式阶数
    int order = traj.order[seg];  // 当前段多项式阶数
    // 计算系数数组的起始索引
    int base = 0;
    for (int s = 0; s < seg; ++s)
        base += traj.order[s] + 1;  // 累加前序段的系数数量
    // 评估多项式
    double px = 0, py = 0, pz = 0;
    for (int j = 0; j <= order; ++j) {
        double c = std::pow(t_rel, j);
        px += traj.coef_x[base + j] * c;
        py += traj.coef_y[base + j] * c;
        pz += traj.coef_z[base + j] * c;
    }
    return {px, py, pz};
}

//----------------------------
// 简易碰撞检测
//----------------------------
bool plan_manage::checkCollision(const std::vector<Eigen::Vector3d>& path) {
    // 对传入的路径 path 中的每个采样点，都检查与所有其它无人机之间的距离
    // 如果在某个采样点上，与某架无人机的距离小于预设的安全距离 safe_distance_，
    // 则认为存在碰撞风险，返回 true；否则返回 false。
    // 如果路径为空，则认为没有碰撞风险
    if (path.empty())
        return false;
    // 只检查路径中的第一个采样点（即下一时刻的位置），你也可以选择检查前 N 个采样点
    size_t checkIndex = 42;  // 检查第N个采样点
    // 遍历所有其他无人机
    for (int i = 0; i < drone_num_; i++) {
        if (i == drone_id_ || !other_drones_[i].valid)
            continue;
        // 假设 path[0] 发生在当前时刻 t0
        double t0 = ros::Time::now().toSec();
        const double dt = 0.01;
        // 检查前N个点或整个路径，不超过路径长度
        size_t N = std::min<size_t>(checkIndex, path.size());
        for (size_t idx = 0; idx < N; ++idx) {
            // 计算采样点对应的绝对时间
            double t_sample = t0 + idx * dt;
            // 从其他无人机的 PolynomialTrajectory 消息中插值
            Eigen::Vector3d other_p = evaluateOtherTrajectory(i, t_sample);
            if ((path[idx] - other_p).norm() < safe_distance_) {
                ROS_WARN("[plan_manage] Collision risk with drone %d at idx=%zu", i, idx);
                return true;
            }
        }
    }
    // 如果遍历完所有无人机及路径采样点后，都未检测到距离小于安全阈值的情况，
    // 则返回 false，表示不存在碰撞风险
    return false;
}

//----------------------------
// 定时器回调：FSM的核心逻辑
//----------------------------
void plan_manage::fsm_timer_cb(const ros::TimerEvent& event) {
    // 一些局部静态变量，用于在多次回调之间存储数据
    static std::vector<Eigen::Matrix<double, 6, 1>> predict_state_list;
    static std::vector<Eigen::Vector3d> Sample_list;
    static std::vector<Eigen::Vector3d> Freegridpath;

    static double publish_time;  // 上一次发布轨迹的时间
    static double last_occtime;
    double start_time;  // 本次开始规划的时间
    double rt;          // 轨迹剩余时间
    /*state*/
    // 如果目标观测数量不足，直接return
    if (target_detect_list.size() < _MAX_SEG)
        return;

    // 初始化起点状态
    Eigen::Matrix3d initState, finState;
    Eigen::Vector3d start_pt, start_vel, start_acc;

    // 如果还没规划过轨迹
    if (!init_flag) {
        initState << position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        start_pt = position;
        start_vel = Eigen::Vector3d::Zero();
        start_acc = Eigen::Vector3d::Zero();
    } else {
        // 如果已经有上一条轨迹，则在其剩余部分进行衔接
        start_time = ros::Time::now().toSec();
        rt = start_time - publish_time + BUDGET_TIME;  // 轨迹可用剩余时间
        start_pt = evaluteP(rt);
        start_vel = evaluteV(rt);
        start_acc = evaluteA(rt);
        initState << start_pt, start_vel, start_acc;
    }

    // 状态判断
    // 1) 如果当前位置在环境中是占据(撞墙等)
    if (env->getInflateOccupancy(start_pt)) {
        fsm = EMERGENCY;
        ROS_INFO("EME");
    }
    // 2) 如果长时间未收到目标信息(>0.4s)
    else if (ros::Time::now().toSec() - last_rcvtime > 0.4) {
        fsm = RELOCATING;
        ROS_INFO("RELOCATE");
    }
    // 3) 否则正常跟踪
    else {
        fsm = TRACKING;
        ROS_INFO("TRACKING");
    }

    // 进入状态机分支
    switch (fsm) {
        //======================
        // A) EMERGENCY
        //======================
        case EMERGENCY: {
            init_flag = false;
            relocate_init = false;
            // 发布一个停止命令
            traj_msg.action = quadrotor_msgs::PositionCommand::ACTION_STOP;
            TrackTrajPub.publish(traj_msg);
            return;
        }

        //======================
        // B) RELOCATING
        //======================
        case RELOCATING: {
            if (!relocate_init) {
                // 第一次进入RELOCATING
                if (Sample_list.size() <= 1) {
                    ROS_ERROR("OCC Sample list is nearly empty!");
                    return;
                }
                // 将预测得到的采样点转换为可用的网格路径
                Freegridpath = kinosearch.poslist2freegrid(Sample_list);
                if (Freegridpath.size() <= 1) {
                    ROS_ERROR("OCC failed");
                    return;
                }
                // 再次基于Freegridpath做栅格路径搜索
                std::vector<Eigen::Vector3d> traj_gridpath = kinosearch.occ_gridpath(Freegridpath, start_pt);

                // 生成末端状态
                finState << traj_gridpath[traj_gridpath.size() - 1], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                // 生成安全走廊
                if (!sfc.generate(traj_gridpath)) {
                    ROS_ERROR("corridor failed, RELOCATE1");
                    return;
                }
                // 轨迹优化
                traj = optimaltraj_generate(initState, finState, sfc.corridor2mat(), v_max, a_max);
                visualize_relocate(traj);

                // 如果之前还没初始化过
                if (!init_flag) {
                    traj_msg = traj2msg(traj);
                    TrackTrajPub.publish(traj_msg);
                    publish_time = ros::Time::now().toSec();
                    last_traj = traj;
                    relocate_init = true;
                    init_flag = true;
                } else {
                    // 如果已经有上一条轨迹，需要判断时间预算
                    double now_time = ros::Time::now().toSec();
                    if (now_time - start_time >= BUDGET_TIME) {
                        ROS_WARN("this traj is not time feasible relocate");
                        return;  // 放弃这次重规划
                    } else {
                        // 等待直到到达BUDGET_TIME
                        while (1) {
                            now_time = ros::Time::now().toSec();
                            if (now_time - start_time >= BUDGET_TIME)
                                break;
                        }
                        relocate_init = true;
                        traj_msg = traj2msg(traj);
                        TrackTrajPub.publish(traj_msg);
                        publish_time = ros::Time::now().toSec();
                        last_traj = traj;
                    }
                }
                sfc.cubes.clear();
            } else {
                vector<Eigen::Vector3d> pos_list;
                for (double t = rt; t < last_traj.getTotalDuration(); t += 0.01) {
                    pos_list.push_back(last_traj.getPos(t));
                }
                if (rt >= last_traj.getTotalDuration())
                    return;
                vector<Eigen::Vector3d> traj_gridpath = kinosearch.poslist2freegrid(pos_list);  // 起点是FREE的
                if (traj_gridpath.size() <= 1)
                    return;
                finState << traj_gridpath[traj_gridpath.size() - 1], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                if (!sfc.generate(traj_gridpath)) {
                    ROS_ERROR("corridor failed, RELOCATE2");
                    return;
                }
                traj = optimaltraj_generate(initState, finState, sfc.corridor2mat(), v_max, a_max);
                visualize_relocate(traj);
                if (!init_flag) {
                    traj_msg = traj2msg(traj);
                    TrackTrajPub.publish(traj_msg);
                    publish_time = ros::Time::now().toSec();
                    last_traj = traj;
                    init_flag = true;
                } else {
                    double now_time;
                    now_time = ros::Time::now().toSec();
                    if (now_time - start_time >= BUDGET_TIME) {
                        ROS_WARN("this traj is not time feasible relocate");
                        return;
                    }  // abort this replan
                    else {
                        while (1) {
                            now_time = ros::Time::now().toSec();
                            if (now_time - start_time >= BUDGET_TIME)
                                break;
                        }
                        relocate_init = true;
                        traj_msg = traj2msg(traj);
                        TrackTrajPub.publish(traj_msg);
                        publish_time = ros::Time::now().toSec();
                        last_traj = traj;
                    }
                }
                sfc.cubes.clear();
            }
            return;
        }

        //======================
        // C) TRACKING
        //======================
        case TRACKING: {
            // 重置迁移标志
            relocate_init = false;
            // mytarget_h = 1.0;  // 初始目标高度设为 1.0 米

            // 新增：根据当前起点状态与目标位置的距离判断是否使用 Bezier 预测
            // 使用 start_pt 作为当前规划起点（在 fsm_timer_cb 前已根据 init_flag 得到）
            Eigen::Vector2d current_xy = start_pt.head(2);  // 当前 UAV 起点位置（只取 xy）
            Eigen::Vector2d target_xy;
            if (!target_detect_list.empty()) {
                // 取最新的目标检测数据的前两个分量作为目标 xy
                target_xy << target_detect_list.back()(0), target_detect_list.back()(1);  // 最新目标检测坐标
            } else {
                ROS_ERROR("No target detected!");
                return;
            }
            // 使用 Bezier 轨迹生成目标预测路径
            int bezier_flag = tgpredict.TrackingGeneration(5, 5, target_detect_list);  // 输入检测数据生成轨迹
            if (bezier_flag == 0) {
                // 预测状态（位置、速度等）和采样点用于后续可视化或路径搜索
                predict_state_list = tgpredict.getStateListFromBezier(1);
                Sample_list = tgpredict.SamplePoslist_bezier(_PREDICT_SEG);
            } else {
                ROS_WARN("bezier predict error");
            }
            if (predict_state_list.size() < 1) {
                ROS_ERROR("Bezier predict failed");
                return;
            }
            // 可视化预测点
            visualize_pre(Sample_list);
            // }

            // 2) 调用混合A*搜索器对预测轨迹做搜索
            int flag_pp = 0;
            Eigen::Vector3d begin_point = predict_state_list[0].head(3);
            flag_pp = kinosearch.search(start_pt, start_vel, predict_state_list, _TIME_INTERVAL);
            if (flag_pp == -2) {
                // 搜索失败
                flag_pp = 0;
            }
            // 如果起点与预测轨迹首点距离很近，就不搜索
            if ((start_pt - begin_point).norm() < 0.6) {
                flag_pp = 0;
            }

            // 3) 如果搜索成功，就提取路径并做走廊+优化
            if (flag_pp) {
                vector<Eigen::Vector3d> pos_xyz = kinosearch.getKinoTraj(0.01);
                vector<Eigen::Vector3d> gridpath = kinosearch.traj2grids(pos_xyz);

                finState << gridpath[gridpath.size() - 1], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                if (!sfc.generate(gridpath)) {
                    ROS_ERROR("corridor failed, TRACKING!");
                    return;
                }
                traj = optimaltraj_generate(initState, finState, sfc.corridor2mat(), v_max, a_max);
                traj_msg = traj2msg(traj);
                visualize_traj(traj);

                // ========== 新增：先做优先级排序 & 碰撞检测 ==========
                // 如果需要进一步做碰撞检测
                bool collision = checkCollision(pos_xyz);
                if (collision) {
                    // 只有 rank=0 的无人机才真正发布轨迹
                    rank = computePriorityRank();

                    if (drone_id_ == 0) {
                        rank = 0;
                    } else if (drone_id_ == 1) {
                        std::cout << "drone_id_: " << drone_id_ << std::endl;
                        rank = 1;
                    } else if (drone_id_ == 2) {
                        rank = 2;
                    } else {
                        rank = 3;
                    }

                    mytarget_h = 0.04 * rank + mytarget_h;
                    if (drone_id_ == 1) {
                        if (mytarget_h > 1.25) {
                            mytarget_h = 1.25;
                        }
                    } else if (drone_id_ == 2) {
                        if (mytarget_h > 1.4) {
                            mytarget_h = 1.4;
                        }
                    } else if (drone_id_ == 3) {
                        if (mytarget_h > 1.55) {
                            mytarget_h = 1.55;
                        }
                    }

                    // 添加发布代码
                    std_msgs::Float32 height_msg;
                    height_msg.data = mytarget_h;
                    height_pub_.publish(height_msg);
                } else {
                    mytarget_h = -0.04 + mytarget_h;
                    if (mytarget_h < 1.2) {
                        mytarget_h = 1.2;
                    }
                    // 添加发布代码
                    std_msgs::Float32 height_msg;
                    height_msg.data = mytarget_h;
                    height_pub_.publish(height_msg);
                }

                // 4) 发布轨迹
                if (!init_flag) {
                    publish_time = ros::Time::now().toSec();
                    traj_msg = traj2msg(traj);
                    // TrackTrajPub.publish(traj_msg);
                    last_traj = traj;
                    init_flag = true;
                    // 设置阈值
                    double threshold = 1;
                    bool ifornot_use_traj_now_msg = false;  // 如果为true则使用traj_now_msg。
                    // 新增：如果当前与目标距离小于阈值，则直接发布目标当前位置作为轨迹消息
                    if ((current_xy - target_xy).norm() < threshold && ifornot_use_traj_now_msg) {
                        // 构造一个保持目标状态的轨迹消息 traj_now_msg
                        quadrotor_msgs::PolynomialTrajectory traj_now_msg;
                        traj_now_msg.header.stamp = ros::Time::now();
                        traj_now_msg.header.frame_id = "world";
                        traj_now_msg.trajectory_id = traj_msg.trajectory_id;  // 保持一致或新编号
                        traj_now_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
                        traj_now_msg.num_segment = 1;
                        traj_now_msg.num_order = 0;        // 单段常数轨迹
                        traj_now_msg.time.push_back(1.0);  // 规划1秒
                        traj_now_msg.order.push_back(0);
                        traj_now_msg.coef_x.push_back(target_xy(0));
                        traj_now_msg.coef_y.push_back(target_xy(1));
                        traj_now_msg.coef_z.push_back(1);  // 固定高度1.0
                        traj_now_msg.mag_coeff = 1;
                        TrackTrajPub.publish(traj_now_msg);
                    } else {
                        TrackTrajPub.publish(traj_msg);
                    }
                } else {
                    double now_time = ros::Time::now().toSec();
                    if (now_time - start_time >= BUDGET_TIME) {
                        ROS_WARN("this traj is not time feasible");
                        return;
                    } else {
                        while (1) {
                            now_time = ros::Time::now().toSec();
                            if (now_time - start_time >= BUDGET_TIME) {
                                break;
                            }
                        }
                        publish_time = ros::Time::now().toSec();
                        traj_msg = traj2msg(traj);
                        TrackTrajPub.publish(traj_msg);
                        last_traj = traj;
                    }
                }
                sfc.cubes.clear();
            }
            kinosearch.reset();
            return;
        }
    }  // end switch
}
