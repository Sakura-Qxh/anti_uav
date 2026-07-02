/*
本程序主要实现了一个基于 ROS 和 PCL 的“随机森林感知”模块，其主要功能为：
生成随机障碍物地图（包括极坐标障碍物和圆形障碍物），存储在全局点云变量 cloudMap 中；
利用 PCL 的 KD-Tree 实现对障碍物点云的搜索；
订阅机器人（或传感器）的里程计消息，更新机器人当前状态；
根据当前状态，在一定范围内搜索局部障碍物点云，并发布局部和全局地图的 ROS 消息；
还支持通过点击回调（clickCallback）添加障碍物点，扩充地图。

输入：
参数服务器配置：地图尺寸、障碍物数量、分辨率、传感器范围等。
里程计话题 /odometry：获取机器人当前位置。
点击事件话题 /goal（已注释）：通过点击位置动态添加障碍物。

输出：
全局点云话题 /map_generator/global_cloud：完整随机地图的点云数据。
局部点云话题 /map_generator/local_cloud：机器人周围传感器范围内的点云数据。

程序主要包括以下几个部分：
全局头文件和命名空间设置
全局变量定义及随机数生成器配置
两个随机地图生成函数（RandomMapGenerate 和 RandomMapGenerateCylinder）
里程计回调函数（rcvOdometryCallbck）
发布感知到障碍物点云的函数（pubSensedPoints）
用户点击回调函数（clickCallback）
main 函数：参数获取、节点初始化、地图生成和循环发布点云
*/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <iostream>
#include <random>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;  // 定义了一个 KD-Tree，用于对 cloudMap 中的点云进行近邻搜索。
vector<int> pointIdxRadiusSearch;                // 用于存储 KD-Tree 搜索结果中的点索引和距离信息。
vector<float> pointRadiusSquaredDistance;

// 使用标准库中的 random_device 和 default_random_engine 来生成随机数。注意此处固定种子“6”使得每次运行产生相同的随机数（也可以使用 rd() 获取真实随机数）。
random_device rd;
default_random_engine eng(6);
// default_random_engine eng(rd());// 若需要真正的随机性，可使用此行

uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;  // 用于存储机器人的状态（例如位置、速度等），从 odometry 消息中更新

int _obs_num;  //_obs_num、_x_size、_y_size、_z_size 等参数：用于配置地图尺寸、障碍物数量、随机地图生成的参数等
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
double _min_dist;

bool _map_ok = false;  // 标志变量，分别表示地图是否生成成功和是否接收到里程计消息。
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

/*
RandomMapGenerate()：生成“极坐标型”障碍物和“圆形”障碍物。
RandomMapGenerateCylinder()：生成带有圆柱形特征的障碍物，且在生成前会检测障碍物间的最小距离，避免重叠
*/
void RandomMapGenerate() {
    pcl::PointXYZ pt_random;

    // 初始化用于生成障碍物的随机数分布（rand_x、rand_y、rand_w、rand_h、rand_radius_、rand_theta_、rand_z_）。
    rand_x = uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = uniform_real_distribution<double>(_h_l, _h_h);

    rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    /*
    循环 _obs_num 次，随机生成障碍物中心位置 (x, y) 和宽度 w。
    对 x、y 根据分辨率进行离散化（使用 floor 函数）。
    计算宽度对应的网格数（widNum）。
    对网格中的每个点（r, s）以及高度 t（从 -20 到 heiNum）生成障碍物点，加入 cloudMap。
    注意：对于 z 轴上若生成的点低于 0，则跳过（避免负高度）。
    */
    for (int i = 0; i < _obs_num; i++) {
        double x, y, w, h;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        int widNum = ceil(w / _resolution);

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(h / _resolution);
                for (int t = -20; t < heiNum; t++) {
                    pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                    pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                    pt_random.z = (t + 0.5) * _resolution + 1e-2;
                    if (pt_random.z < 0)
                        continue;  // hzc
                    cloudMap.points.push_back(pt_random);
                }
            }
    }

    // generate circle obs
    /*循环 circle_num_ 次，随机生成圆心位置 (x, y, z) 并离散化处理。
    生成一个旋转矩阵（rotate），随机角度 theta 用于旋转。
    随机生成半径 radius1 和 radius2。
    对于角度从 0 到 2π（6.282）按 _resolution/2 的步长计算圆上点，构造点 cpt，然后“膨胀”（inflate）周围邻域，计算旋转和平移后的点，将其加入 cloudMap。
    */
    for (int i = 0; i < circle_num_; ++i) {
        double x, y, z;
        x = rand_x(eng);
        y = rand_y(eng);
        z = rand_z_(eng);

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        Eigen::Vector3d translate(x, y, z);

        double theta = rand_theta_(eng);
        Eigen::Matrix3d rotate;
        rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
            1;

        double radius1 = rand_radius_(eng);
        double radius2 = rand_radius2_(eng);

        // draw a circle centered at (x,y,z)
        Eigen::Vector3d cpt;
        for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
            cpt(0) = 0.0;
            cpt(1) = radius1 * cos(angle);
            cpt(2) = radius2 * sin(angle);

            // inflate
            Eigen::Vector3d cpt_if;
            for (int ifx = -0; ifx <= 0; ++ifx)
                for (int ify = -0; ify <= 0; ++ify)
                    for (int ifz = -0; ifz <= 0; ++ifz) {
                        cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                                       ifz * _resolution);
                        cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                        pt_random.x = cpt_if(0);
                        pt_random.y = cpt_if(1);
                        pt_random.z = cpt_if(2);
                        if (pt_random.z < 0)
                            continue;  // hzc
                        cloudMap.push_back(pt_random);
                    }
        }
    }

    // 最后设置 cloudMap 的宽度、高度以及 is_dense 标志，并利用 cloudMap 构造 KD-Tree（kdtreeLocalMap），设置 _map_ok 标志为 true，表示地图生成完成。
    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished generate random map ");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

/*
首先生成一个 obs_position 数组，用于存储已生成障碍物的 (x, y) 坐标，确保新障碍物与已有障碍物之间的距离不小于 _min_dist；如果距离过近，则跳过该次生成。
之后再生成障碍物时，分别使用极坐标型和圆形障碍物生成算法（区分 i%2 的情况），以生成圆柱形障碍物的效果。
最后同样构造 cloudMap 并设置 KD-Tree，以及 _map_ok 标志。
*/
void RandomMapGenerateCylinder() {
    pcl::PointXYZ pt_random;

    vector<Eigen::Vector2d> obs_position;

    rand_x = uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = uniform_real_distribution<double>(_h_l, _h_h);
    rand_inf = uniform_real_distribution<double>(0.5, 1.5);

    rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    for (int i = 0; i < _obs_num && ros::ok(); i++) {
        double x, y, w, h, inf;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);
        inf = rand_inf(eng);

        bool flag_continue = false;
        for (auto p : obs_position)
            if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/) {
                i--;
                flag_continue = true;
                break;
            }
        if (flag_continue)
            continue;

        obs_position.push_back(Eigen::Vector2d(x, y));

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil((w * inf) / _resolution);
        double radius = (w * inf) / 2;

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(h / _resolution);
                for (int t = -20; t < heiNum; t++) {
                    double temp_x = x + (r + 0.5) * _resolution + 1e-2;
                    double temp_y = y + (s + 0.5) * _resolution + 1e-2;
                    double temp_z = (t + 0.5) * _resolution + 1e-2;
                    if (i % 2 == 0) {
                        pt_random.x = temp_x;
                        pt_random.y = temp_y;
                        pt_random.z = temp_z;
                        if (pt_random.z < 0)
                            continue;  // hzc
                        cloudMap.points.push_back(pt_random);
                    } else {
                        if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius) {
                            pt_random.x = temp_x;
                            pt_random.y = temp_y;
                            pt_random.z = temp_z;
                            if (pt_random.z < 0)
                                continue;  // hzc
                            cloudMap.points.push_back(pt_random);
                        }
                    }
                }
            }
    }

    // generate circle obs
    for (int i = 0; i < circle_num_; ++i) {
        double x, y, z;
        x = rand_x(eng);
        y = rand_y(eng);
        z = rand_z_(eng);

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        Eigen::Vector3d translate(x, y, z);

        double theta = rand_theta_(eng);
        Eigen::Matrix3d rotate;
        rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
            1;

        double radius1 = rand_radius_(eng);
        double radius2 = rand_radius2_(eng);

        // draw a circle centered at (x,y,z)
        Eigen::Vector3d cpt;
        for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
            cpt(0) = 0.0;
            cpt(1) = radius1 * cos(angle);
            cpt(2) = radius2 * sin(angle);

            // inflate
            Eigen::Vector3d cpt_if;
            for (int ifx = -0; ifx <= 0; ++ifx)
                for (int ify = -0; ify <= 0; ++ify)
                    for (int ifz = -0; ifz <= 0; ++ifz) {
                        cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                                       ifz * _resolution);
                        cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                        pt_random.x = cpt_if(0);
                        pt_random.y = cpt_if(1);
                        pt_random.z = cpt_if(2);
                        if (pt_random.z < 0)
                            continue;  // hzc
                        cloudMap.push_back(pt_random);
                    }
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    ROS_WARN("Finished generate random map ");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

/*
该函数用于订阅 odometry 消息，更新全局状态变量 _state 和 _has_odom 标志。
首先检查 odom.child_frame_id 是否为 "X" 或 "O"，若是则直接返回（可能过滤掉冗余数据）。
然后将 odometry 中的位置和速度信息依次存入 _state 向量中，更新 _has_odom 为 true。
注释中可指出：_state 数组中前 3 个元素是位置，后 3 个为线速度，最后 3 个暂时置为 0。
*/
void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
    if (odom.child_frame_id == "X" || odom.child_frame_id == "O")
        return;
    _has_odom = true;

    _state = {odom.pose.pose.position.x,
              odom.pose.pose.position.y,
              odom.pose.pose.position.z,
              odom.twist.twist.linear.x,
              odom.twist.twist.linear.y,
              odom.twist.twist.linear.z,
              0.0,
              0.0,
              0.0};
}

int i = 0;
/*
该函数用于发布障碍物点云消息，分为两种情况：

发布全局地图：
调用 pcl::toROSMsg 将 cloudMap 转换为 ROS 消息 globalMap_pcd，并发布到 _all_map_pub。
（注意：该部分当前未被注释掉，可能只在前几次循环中发布全局地图。）

发布局部地图：
当 _map_ok 与 _has_odom 均为 true 时，先从 _state 中取出当前机器人的位置，作为搜索点 searchPoint。
利用 KD-Tree 的 radiusSearch 在 _sensing_range 范围内搜索障碍物，并将搜索到的点加入局部点云 localMap。
若搜索结果为空，则报错；否则，将 localMap 转换为 ROS 消息 localMap_pcd 并发布到 _local_map_pub。
*/
void pubSensedPoints() {
    // if (i < 10) {
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    _all_map_pub.publish(globalMap_pcd);
    // }

    return;

    /* ---------- only publish points around current position ---------- */
    if (!_map_ok || !_has_odom)
        return;

    pcl::PointCloud<pcl::PointXYZ> localMap;

    pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();

    pcl::PointXYZ pt;

    if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
        return;

    if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                    pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            pt = cloudMap.points[pointIdxRadiusSearch[i]];
            localMap.points.push_back(pt);
        }
    } else {
        ROS_ERROR("[Map server] No obstacles .");
        return;
    }

    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    pcl::toROSMsg(localMap, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    _local_map_pub.publish(localMap_pcd);
}

/*
当接收到用户点击（PoseStamped 消息）时，利用点击的位置在对应区域生成障碍物点：

将点击位置 x、y 离散化（类似随机地图生成中的处理）。
依据宽度 w（随机生成）计算网格数 widNum，然后对每个网格内的点计算高度和位置，将生成的点同时加入 clicked_cloud_ 和 cloudMap。
更新 clicked_cloud_ 的宽度、高度和 is_dense 标志，并转换为 ROS 消息 localMap_pcd 发布到 click_map_pub_。
*/
void clickCallback(const geometry_msgs::PoseStamped& msg) {
    double x = msg.pose.position.x;
    double y = msg.pose.position.y;
    double w = rand_w(eng);
    double h;
    pcl::PointXYZ pt_random;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
            h = rand_h(eng);
            int heiNum = ceil(h / _resolution);
            for (int t = -1; t < heiNum; t++) {
                pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                pt_random.z = (t + 0.5) * _resolution + 1e-2;
                clicked_cloud_.points.push_back(pt_random);
                cloudMap.points.push_back(pt_random);
            }
        }
    clicked_cloud_.width = clicked_cloud_.points.size();
    clicked_cloud_.height = 1;
    clicked_cloud_.is_dense = true;

    pcl::toROSMsg(clicked_cloud_, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    click_map_pub_.publish(localMap_pcd);

    cloudMap.width = cloudMap.points.size();

    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_map_sensing");
    ros::NodeHandle n("~");

    _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
    _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

    _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

    click_map_pub_ =
        n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
    // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);
    ros::Subscriber click_sub = n.subscribe("/clicked_point", 10, clickCallback);

    n.param("init_state_x", _init_x, 0.0);
    n.param("init_state_y", _init_y, 0.0);

    n.param("map/x_size", _x_size, 50.0);
    n.param("map/y_size", _y_size, 50.0);
    n.param("map/z_size", _z_size, 5.0);
    n.param("map/obs_num", _obs_num, 30);
    n.param("map/resolution", _resolution, 0.1);
    n.param("map/circle_num", circle_num_, 30);

    n.param("ObstacleShape/lower_rad", _w_l, 0.3);
    n.param("ObstacleShape/upper_rad", _w_h, 0.8);
    n.param("ObstacleShape/lower_hei", _h_l, 3.0);
    n.param("ObstacleShape/upper_hei", _h_h, 7.0);

    n.param("ObstacleShape/radius_l", radius_l_, 7.0);
    n.param("ObstacleShape/radius_h", radius_h_, 7.0);
    n.param("ObstacleShape/z_l", z_l_, 7.0);
    n.param("ObstacleShape/z_h", z_h_, 7.0);
    n.param("ObstacleShape/theta", theta_, 7.0);

    n.param("sensing/radius", _sensing_range, 10.0);
    n.param("sensing/rate", _sense_rate, 10.0);

    n.param("min_distance", _min_dist, 1.0);

    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;

    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;

    _obs_num = min(_obs_num, (int)_x_size * 10);
    _z_limit = _z_size;

    ros::Duration(0.5).sleep();

    // RandomMapGenerate();
    RandomMapGenerateCylinder();

    ros::Rate loop_rate(_sense_rate);

    while (ros::ok()) {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
}