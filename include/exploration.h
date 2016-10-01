#ifndef JACKALEXPLORATION_EXPLORATION_H_
#define JACKALEXPLORATION_EXPLORATION_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <octomap/octomap.h>
// #include <geometry_msgs/Pose.h>


using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double octo_reso = 0.1;

octomap::OcTree* cur_tree;
octomap::OcTree* cur_tree_2d;

tf::TransformListener *tf_listener; 

point3d position, laser_orig, velo_orig;

ofstream explo_log_file;
std::string octomap_name_2d, octomap_name_3d;


struct sensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    // vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    sensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                InitialVector = point3d(1.0, 0.0, 0.0);
                InitialVector.rotate_IP(0.0, j * angle_inc_vel, i * angle_inc_hor);
                SensorRays.push_back(InitialVector);
        }
    }
}; 
sensorModel velodynePuck(360, 16, 2*PI, 0.5236, 30.0);


double getFreeVolume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}


octomap::Pointcloud castSensorRays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &sensor_orientation) {
    octomap::Pointcloud hits;

    octomap::Pointcloud RaysToCast;
    RaysToCast.push_back(velodynePuck.SensorRays);
    RaysToCast.rotate(sensor_orientation.x(),sensor_orientation.y(),sensor_orientation.z());
    point3d end;
    // Cast Rays to 3d OctoTree and get hit points
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, velodynePuck.max_range)) {
            hits.push_back(end);
        } else {
            end = RaysToCast.getPoint(i) * velodynePuck.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}

vector<pair<point3d, point3d>> generate_candidates(point3d sensor_orig, double initial_yaw) {
    double R = 0.5;   // Robot step, in meters.
    double n = 3;
    octomap::OcTreeNode *n_cur;

    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();                // fixed 
    double x, y;

    for (double radius = R; radius <= R + 0.5; radius += 0.49)
        for(double yaw = initial_yaw-PI/2; yaw < initial_yaw+PI/2; yaw += PI / (2*n) ) {
            x = sensor_orig.x() + radius * cos(yaw);
            y = sensor_orig.y() + radius * sin(yaw);

            // for every candidate goal, check surroundings
            bool candidate_valid = true;
            for (double x_buf = x - 0.1; x_buf < x + 0.1; x_buf += octo_reso/2) 
                for (double y_buf = y - 0.1; y_buf < y + 0.1; y_buf += octo_reso/2)
                    for (double z_buf = z - 0.1; z_buf < z + 0.1; z_buf += octo_reso/2)
            {
                n_cur = cur_tree_2d->search(point3d(x_buf, y_buf, z_buf));
                if(!n_cur) {
                    // ROS_WARN("Part of (%f, %f, %f) unknown", x_buf, y_buf, z_buf);
                    // candidate_valid = false;
                    continue;
                }                                 
                if(cur_tree_2d->isNodeOccupied(n_cur)) {
                    // ROS_WARN("Part of (%f, %f, %f) occupied", x_buf, y_buf, z_buf);
                    candidate_valid = false;
                }  
            }
            if (candidate_valid)
            {
                candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
            }
            else
                ROS_WARN("Part of Candidtae(%f, %f, %f) occupied", x, y, z);
        }
        
    return candidates;
}

double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    octree_copy->insertPointCloud(hits, sensor_orig, velodynePuck.max_range, true, true);
    double after = getFreeVolume(octree_copy);
    delete octree_copy;
    return after - before;
}


void velodyne_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);
    octomap::Pointcloud hits;

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
    }
    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        // if(isnan(cloud->at(j).x)) continue;
        if(cloud->at(j).z < -1.0)    continue;  
        hits.push_back(point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z));
        // cur_tree->insertRay(point3d( velo_orig.x(),velo_orig.y(),velo_orig.z()), 
        //     point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), velodynePuck.max_range);
    }

    cur_tree->insertPointCloud(hits, velo_orig, velodynePuck.max_range);
    // cur_tree->updateInnerOccupancy();
    ROS_INFO("Entropy(3d map) : %f", getFreeVolume(cur_tree));

    cur_tree->write(octomap_name_3d);
    delete cloud;
    delete cloud_local;
}

void hokuyo_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg )
{
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);
    octomap::Pointcloud hits;

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
    }

    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        // if(isnan(cloud->at(j).x)) continue;
        hits.push_back(point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z));
        // cur_tree_2d->insertRay(point3d( laser_orig.x(),laser_orig.y(),laser_orig.z()), 
        //     point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), 30.0);
    }
    cur_tree_2d->insertPointCloud(hits, laser_orig, velodynePuck.max_range);
    // cur_tree_2d->updateInnerOccupancy();
    ROS_INFO("Entropy(2d map) : %f", getFreeVolume(cur_tree_2d));
    cur_tree_2d->write(octomap_name_2d);
    delete cloud;
    delete cloud_local;

}


class InfoTheoreticExploration {
public:
    typedef octomap::point3d point3d;
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    
    InfoTheoreticExploration();
    InfoTheoreticExploration(double octo_reso);
    
    double calculateMutualInformation(void);

private:
    double getFreeVolume(void);
    octomap::OcTree* cur_tree;
    double octo_reso;
};

#endif