#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include "laser_geometry/laser_geometry.h"


using namespace std;
// using namespace std::chrono;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
laser_geometry::LaserProjection projector;

void hokuyo_callbacks(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud cloud;
    // PointCloud::Ptr cloud (new PointCloud);
    // PointCloud cloud;
    projector.projectLaser(*scan_in, cloud);

    // Publish the new point cloud.
    // cloud->header.frame_id = "/laser";
    // cloud->header.stamp = ros::Time::now().toNSec() / 1e3;
    // pcl_from_scan.publish(cloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserScan_to_pointcloud");
    ros::NodeHandle nh;

    ros::Subscriber hokuyo_sub;
    hokuyo_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 2, hokuyo_callbacks);

    ros::Publisher pcl_from_scan;
    pcl_from_scan = nh.advertise<PointCloud>("hokuyo_points", 1);

    nh.shutdown();          
    return 0;
}