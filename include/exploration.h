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
#include <geometry_msgs/Pose.h>


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


double countFreeVolume(const octomap::OcTree *octree) {
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

// extract 2d frontier points
vector<vector<point3d>> extractFrontierPoints(const octomap::OcTree *octree) {

    vector<vector<point3d>> frontier_groups;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true;         // whether or not a frontier point
    bool belong_old;            //whether or not belong to old group
    double distance;
    double R1 = 0.5;            //group size
    double x_cur, y_cur, z_cur;


    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

      if(!octree->isNodeOccupied(*n))
        {
         x_cur = n.getX();
         y_cur = n.getY();
         z_cur = n.getZ();

         if(z_cur < 0.2)    continue;
         if(z_cur > 0.4)    continue;
         //if there are unknown around the cube, the cube is frontier
         for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
             for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
            {
                n_cur_frontier = octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                if(!n_cur_frontier)
                {
                    frontier_true = true;
                    continue;            
                }

            }
            if(frontier_true)// && num_free >5 )
            {
                // divede frontier points into groups
                if(frontier_groups.size() < 1)
                {
                    frontier_points.resize(1);
                    frontier_points[0] = point3d(x_cur,y_cur,z_cur);
                    frontier_groups.push_back(frontier_points);
                    frontier_points.clear();
                }
                else
                {
                    bool belong_old = false;            

                    for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                            distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)) ;
                            if(distance < R1){
                               frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
                               belong_old = true;
                               break;
                            }
                    }
                    if(!belong_old){
                               frontier_points.resize(1);
                               frontier_points[0] = point3d(x_cur, y_cur, z_cur);
                               frontier_groups.push_back(frontier_points);
                               frontier_points.clear();
                    }                              
                }

            } 
        }
        
    }
    return frontier_groups;
}

//generate candidates for moving. Input sensor_orig and initial_yaw, Output candidates
//senor_orig: locationg of sensor.   initial_yaw: yaw direction of sensor
vector<pair<point3d, point3d>> extractCandidateViewPoints(vector<vector<point3d>> frontier_groups, point3d sensor_orig ) {
    double R2 = 1.0;        // Robot step, in meters.
    double R3 = 0.4;       // to other frontiers
    double n = 12;
    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();
    double x, y;
    double yaw;
    double distance_can;

        for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++) {
            for(double yaw = 0; yaw < 2*PI; yaw += PI*2 / n){ 
                x = frontier_groups[u][0].x() - R2 * cos(yaw);
                y = frontier_groups[u][0].y() - R2 * sin(yaw);

                bool candidate_valid = true;
                n_cur_3d = cur_tree->search(point3d(x, y, z));


                if (!n_cur_3d) {
                    candidate_valid = false;
                    continue;
                }

                if(sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2)) < 0.25){
                  candidate_valid = false;// delete candidates close to sensor_orig
                  continue;
                }

                else{

                    // check candidate to other frontiers;
                    for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++)
                        for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
                            distance_can = sqrt(pow(x - frontier_groups[n][m].x(),2) + pow(y - frontier_groups[n][m].y(),2));
                            if(distance_can < R3){
                                candidate_valid = false;        //delete candidates close to frontier
                                continue;
                            }
                    }
                
                    // volumn check
                    for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                        for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                            for (double z_buf = sensor_orig.z()-0.2; z_buf <sensor_orig.z()+0.2; z_buf += octo_reso)
                            {
                                n_cur_3d = cur_tree->search(point3d(x_buf, y_buf, z_buf));
                                if(!n_cur_3d)       continue;
                                else if (cur_tree->isNodeOccupied(n_cur_3d)){
                                candidate_valid = false;//delete candidates which have ccupied cubes around in 3D area
                                continue;
                                }  
                            }

                }

                if (candidate_valid)
                {
                    candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                }
            }
        }
    return candidates;
}

// vector<pair<point3d, point3d>> generate_candidates(point3d sensor_orig, double initial_yaw) {
//     double R = 0.5;   // Robot step, in meters.
//     double n = 3;
//     octomap::OcTreeNode *n_cur;

//     vector<pair<point3d, point3d>> candidates;
//     double z = sensor_orig.z();                // fixed 
//     double x, y;

//     for (double radius = R; radius <= R + 0.5; radius += 0.49)
//         for(double yaw = initial_yaw-PI/2; yaw < initial_yaw+PI/2; yaw += PI / (2*n) ) {
//             x = sensor_orig.x() + radius * cos(yaw);
//             y = sensor_orig.y() + radius * sin(yaw);

//             // for every candidate goal, check surroundings
//             bool candidate_valid = true;
//             for (double x_buf = x - 0.1; x_buf < x + 0.1; x_buf += octo_reso/2) 
//                 for (double y_buf = y - 0.1; y_buf < y + 0.1; y_buf += octo_reso/2)
//                     for (double z_buf = z - 0.1; z_buf < z + 0.1; z_buf += octo_reso/2)
//             {
//                 n_cur = cur_tree_2d->search(point3d(x_buf, y_buf, z_buf));
//                 if(!n_cur) {
//                     // ROS_WARN("Part of (%f, %f, %f) unknown", x_buf, y_buf, z_buf);
//                     // candidate_valid = false;
//                     continue;
//                 }                                 
//                 if(cur_tree_2d->isNodeOccupied(n_cur)) {
//                     // ROS_WARN("Part of (%f, %f, %f) occupied", x_buf, y_buf, z_buf);
//                     candidate_valid = false;
//                 }  
//             }
//             if (candidate_valid)
//             {
//                 candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
//             }
//             else
//                 ROS_WARN("Part of Candidtae(%f, %f, %f) occupied", x, y, z);
//         }
        
//     return candidates;
// }

double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    octree_copy->insertPointCloud(hits, sensor_orig, velodynePuck.max_range, true, true);
    double after = countFreeVolume(octree_copy);
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
    ROS_INFO("Entropy(3d map) : %f", countFreeVolume(cur_tree));

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
    ROS_INFO("Entropy(2d map) : %f", countFreeVolume(cur_tree_2d));
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
    
    void calculateMutualInformation(void);
    void generateCandidateViewPoints(void);



private:
    double countFreeVolume(void);
    octomap::OcTree* cur_tree;
    double octo_reso;
    vector<pair<point3d, point3d>> candidate_view_points;
    octomap::Pointcloud hits;
    
};

#endif