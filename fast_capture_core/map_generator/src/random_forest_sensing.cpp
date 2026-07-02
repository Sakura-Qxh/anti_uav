// random_forest_sensing.cpp
// 修改版：在 (-10,10) 生成半径 5、高度 5 的圆柱形障碍物；
// 在 (-10,-10) 生成边长 6、高度 5 的正方形障碍物

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

using namespace std;

ros::Publisher _all_map_pub;
ros::Subscriber _odom_sub;

bool _map_ok = false;
double _resolution = 0.1;

pcl::PointCloud<pcl::PointXYZ> cloudMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
sensor_msgs::PointCloud2 globalMap_pcd;

// 生成固定障碍物：
// 1) 圆形障碍物：中心 (-10,10)，半径=5，高度=5
// 2) 正方形障碍物：中心 (-10,-10)，边长=6，高度=5
void FixedMapGenerate() {
    cloudMap.points.clear();
    pcl::PointXYZ pt;

    // 圆柱形障碍物
    const double cx1 = -1.5, cy1 = 2, radius = 0.3, height_cyl = 3.0;
    for (double x = cx1 - radius; x <= cx1 + radius; x += _resolution) {
        for (double y = cy1 - radius; y <= cy1 + radius; y += _resolution) {
            double dx = x - cx1, dy = y - cy1;
            if (dx * dx + dy * dy <= radius * radius) {
                for (double z = 0.0; z <= height_cyl; z += _resolution) {
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    cloudMap.points.push_back(pt);
                }
            }
        }
    }

    // 立方体障碍物（正方形柱）
    const double cx2 = -1.5, cy2 = -2, half_side = 0.3, height_box = 3.0;
    for (double x = cx2 - half_side; x <= cx2 + half_side; x += _resolution) {
        for (double y = cy2 - half_side; y <= cy2 + half_side; y += _resolution) {
            for (double z = 0.0; z <= height_box; z += _resolution) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                cloudMap.points.push_back(pt);
            }
        }
    }

    // 圆柱形障碍物
    const double cx3 = 1.5, cy3 = 0, radius3 = 0.3, height_cyl3 = 3.0;
    for (double x = cx3 - radius3; x <= cx3 + radius3; x += _resolution) {
        for (double y = cy3 - radius3; y <= cy3 + radius3; y += _resolution) {
            double dx = x - cx3, dy = y - cy3;
            if (dx * dx + dy * dy <= radius3 * radius3) {
                for (double z = 0.0; z <= height_cyl3; z += _resolution) {
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    cloudMap.points.push_back(pt);
                }
            }
        }
    }

    // 立方体障碍物（正方形柱）
    const double cx4 = 3.5, cy4 = -2.5, half_side4 = 0.3, height_box4 = 3.0;
    for (double x = cx4 - half_side4; x <= cx4 + half_side4; x += _resolution) {
        for (double y = cy4 - half_side4; y <= cy4 + half_side4; y += _resolution) {
            for (double z = 0.0; z <= height_box4; z += _resolution) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                cloudMap.points.push_back(pt);
            }
        }
    }

    // 立方体障碍物（正方形柱）
    const double cx5 = 3.5, cy5 = 1.5, half_side5 = 0.3, height_box5 = 4.0;
    for (double x = cx5 - half_side5; x <= cx5 + half_side5; x += _resolution) {
        for (double y = cy5 - half_side5; y <= cy5 + half_side5; y += _resolution) {
            for (double z = 0.0; z <= height_box5; z += _resolution) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                cloudMap.points.push_back(pt);
            }
        }
    }

    // --- 矩形围栏边界: 左下(0,10.5) 到 右上(24,-10.5), 围栏高度=5 ---
    {
        const double x_min = -4.5, x_max = 5.5;
        const double y_max = 4, y_min = -5;
        const double fence_h = 5.0;

        // 上边 (x 从 x_min 到 x_max, y = y_max)
        for (double x = x_min; x <= x_max; x += _resolution) {
            double y = y_max;
            for (double z = 0.0; z <= fence_h; z += _resolution) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                cloudMap.points.push_back(pt);
            }
        }
        // 下边 (x 从 x_min 到 x_max, y = y_min)
        for (double x = x_min; x <= x_max; x += _resolution) {
            double y = y_min;
            for (double z = 0.0; z <= fence_h; z += _resolution) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                cloudMap.points.push_back(pt);
            }
        }
        // 左边 (x = x_min, y 从 y_min 到 y_max)
        for (double y = y_min; y <= y_max; y += _resolution) {
            double x = x_min;
            for (double z = 0.0; z <= fence_h; z += _resolution) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                cloudMap.points.push_back(pt);
            }
        }
        // 右边 (x = x_max, y 从 y_min 到 y_max)
        for (double y = y_min; y <= y_max; y += _resolution) {
            double x = x_max;
            for (double z = 0.0; z <= fence_h; z += _resolution) {
                pt.x = x;
                pt.y = y;
                pt.z = z;
                cloudMap.points.push_back(pt);
            }
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    // 构造 KD-Tree
    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
    _map_ok = true;
    ROS_WARN("Fixed map generated: circle at (-10,10,r=5,h=5) and square at (-10,-10,side=6,h=5)");
}

void rcvOdometryCallbck(const nav_msgs::Odometry& /*odom*/) {
    // this demo only publishes the fixed global map
}

void pubSensedPoints() {
    if (!_map_ok)
        return;
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    _all_map_pub.publish(globalMap_pcd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fixed_map_sensing");
    ros::NodeHandle n("~");

    n.param("map/resolution", _resolution, 0.1);

    _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
    _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

    FixedMapGenerate();

    ros::Rate loop_rate(1.0);  // 1 Hz 发布全局地图
    while (ros::ok()) {
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
