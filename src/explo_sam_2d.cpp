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

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
// #include <tf/transform_broadcaster.h>
#include "navigation_utils.h"
// #include "laser_geometry/laser_geometry.h"
#include <ros/callback_queue.h>


using namespace std;
using namespace std::chrono;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double free_prob = 0.3;
const double octo_reso = 0.1;

octomap::OcTree* cur_tree;
octomap_msgs::Octomap cur_tree_msg;
bool octomap_flag = 0; // 0 : msg not received
bool kinect_flag = 0; // 0 : msg not received
tf::TransformListener *tf_listener; 
int octomap_seq = 0;


// laser_geometry::LaserProjection projector;
   
point3d position, laser_orig, velo_orig;
// point3d orientation;

ros::Publisher Map_pcl_pub;
ros::Publisher Free_pcl_pub;
ros::Publisher VScan_pcl_pub;
ros::Publisher octomap_pub;

PointCloud::Ptr map_pcl (new PointCloud);
PointCloud::Ptr free_pcl (new PointCloud);
PointCloud::Ptr vsn_pcl (new PointCloud);

struct SensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;

    SensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        // for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                // pitch_yaws.push_back(make_pair(j * angle_inc_vel, i * angle_inc_hor));
                pitch_yaws.push_back(make_pair(0, i * angle_inc_hor));
        }
    }
}; 

// SensorModel Velodyne_puck(3600, 16, 2*PI, PI/6, 100.0 );
SensorModel Velodyne_puck(360, 1, 2*PI, 0.00523, 100.0 );


double get_free_volume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}

// void get_free_points(const octomap::OcTree *octree, PointCloud::Ptr pclPtr) {
    
//     for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
//         if(!octree->isNodeOccupied(*n))
//         {
//             pclPtr->points.push_back(pcl::PointXYZ(n.getX()/5.0, n.getY()/5.0, n.getZ()/5.0));
//             pclPtr->width++;
//         }
//     }
//     return;
// }


vector<point3d> cast_sensor_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    vector<point3d> hits;
    // octomap::OcTreeNode *n;

    // #pragma omp parallel for
    for(auto p_y : Velodyne_puck.pitch_yaws) {
        double pitch = p_y.first;
        double yaw = p_y.second;
        point3d direction_copy(direction.normalized());
        point3d end;
        direction_copy.rotate_IP(0.0, pitch, yaw);
        if(octree->castRay(position, direction_copy, end, true, Velodyne_puck.max_range)) {
            hits.push_back(end);
        } else {
            direction_copy *= Velodyne_puck.max_range;
            direction_copy += position;
            // n = octree->search(direction_copy);
            // if (!n)                                 continue;
            // if (n->getOccupancy() < free_prob )     continue;
            hits.push_back(direction_copy);
        }
    }
    return hits;
}

vector<pair<point3d, point3d>> generate_candidates(point3d sensor_orig) {
    double R = 0.5;   // Robot step, in meters.
    double n = 5;
    int counter = 0;
    octomap::OcTreeNode *n_cur;

    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();                // fixed 
    double pitch = 0;                   // fixed
    double x, y;

    // for(z = sensor_orig.z() - 1; z <= sensor_orig.z() + 1; z += 1)
        for(double yaw = 0; yaw < 2 * PI; yaw += PI / n) {
            x = sensor_orig.x() + R * cos(yaw);
            y = sensor_orig.y() + R * sin(yaw);
            n_cur = cur_tree->search(point3d(x,y,z));
            if(!n_cur)                                  continue;
            if(n_cur->getOccupancy() > free_prob)       continue;
            candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0, pitch, yaw)));
            counter++;
        }

    ROS_INFO("Candidates : %d", counter);
    return candidates;
}

double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const vector<point3d> &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    // #pragma omp parallel for
    for(const auto h : hits) {
        octree_copy->insertRay(sensor_orig, h, Velodyne_puck.max_range);
    }
    octree_copy->updateInnerOccupancy();
    double after = get_free_volume(octree_copy);
    delete octree_copy;
    return after - before;
}

void RPY2Quaternion(double roll, double pitch, double yaw, double *x, double *y, double *z, double *w) {
    double cr2, cp2, cy2, sr2, sp2, sy2;
    cr2 = cos(roll*0.5);
    cp2 = cos(pitch*0.5);
    cy2 = cos(yaw*0.5);

    sr2 = -sin(roll*0.5);
    sp2 = -sin(pitch*0.5);
    sy2 = sin(yaw*0.5);

    *w = cr2*cp2*cy2 + sr2*sp2*sy2;
    *x = sr2*cp2*cy2 - cr2*sp2*sy2;
    *y = cr2*sp2*cy2 + sr2*cp2*sy2;
    *z = cr2*cp2*sy2 - sr2*sp2*cy2;
}


void velodyne_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);

    pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener);
    // map_pcl->clear();
    // map_pcl->header.frame_id = "/explo_points";

    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        if(isnan(cloud->at(j).x)) continue;
        cur_tree->insertRay(point3d( velo_orig.x(),velo_orig.y(),velo_orig.z()), 
            point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), Velodyne_puck.max_range);
        // add the point into map
        // map_pcl->points.push_back(pcl::PointXYZ(cloud_local->at(j).x, cloud_local->at(j).y, cloud_local->at(j).z));
        // map_pcl->width++;
    }

    // Insert point cloud into octomap.
    // cur_tree->insertPointCloud(*cloud,velo_orig);

    bool res = octomap_msgs::fullMapToMsg(*cur_tree, cur_tree_msg);
    cur_tree_msg.header.seq = octomap_seq++;
    cur_tree_msg.header.frame_id = "/map";
    cur_tree_msg.header.stamp = ros::Time::now();
    cur_tree_msg.resolution = octo_reso;
    cur_tree_msg.binary = true;
    cur_tree_msg.id = "OcTree";
    // map_pcl->height = 1;
    
    ROS_INFO("Entropy(Velo) : %f", get_free_volume(cur_tree));
    // map_pcl->header.stamp = ros::Time::now().toNSec() / 1e3;
    // Map_pcl_pub.publish(map_pcl);

    // cur_tree->writeBinary("Octomap_DA.bt");
    cur_tree->write("Octomap_DA.ot");
    // cout << " " << endl;
    octomap_pub.publish(cur_tree_msg);
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

    pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener);

    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        if(isnan(cloud->at(j).x)) continue;
        cur_tree->insertRay(point3d( laser_orig.x(),laser_orig.y(),laser_orig.z()), 
            point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), Velodyne_puck.max_range);
    }

    // cout << "Entropy(scan) : " << get_free_volume(cur_tree) << endl;
    ROS_INFO("Entropy(scan) : %f", get_free_volume(cur_tree));
    delete cloud;
    delete cloud_local;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "explo_octomap_da_2d");
    ros::NodeHandle nh;
    // ros::Subscriber octomap_sub;
    // octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, octomap_callback);

    ros::Subscriber velodyne_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, velodyne_callbacks);
    
    ros::Subscriber hokuyo_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hokuyo_points", 1, hokuyo_callbacks);

    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Goal_Marker", 1 );
    ros::Publisher JackalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Jackal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("Candidate_MIs", 1);

    octomap_pub = nh.advertise<octomap_msgs::Octomap>( "Octomap_realtime", 1);

    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;

    visualization_msgs::MarkerArray marker_array_msg;

    Map_pcl_pub = nh.advertise<PointCloud>("Current_Map", 1);
    VScan_pcl_pub = nh.advertise<PointCloud>("virtual_Scans", 1);
    Free_pcl_pub = nh.advertise<PointCloud>("Free_points", 1);

    map_pcl->header.frame_id = "/velodyne";
    map_pcl->height = 1;
    map_pcl->width = 0;

    free_pcl->header.frame_id = "/map";
    free_pcl->height = 1;
    free_pcl->width = 0;

    vsn_pcl->header.frame_id = "/map";
    vsn_pcl->height = 1;
    vsn_pcl->width = 0;
   
    double qx, qy, qz, qw;
    RPY2Quaternion(0, 0, 1, &qx, &qy, &qz, &qw);


    // Initialize parameters 
    // ros::Rate r(10); // 1 hz
    int max_idx = 0;

    position = point3d(0, 0, 0.0);
    // point3d dif_pos = point3d(0,0,0);
    point3d eu2dr(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    cur_tree = &new_tree;


    // Update the initial location of the robot
   while(!tf_listener->waitForTransform("/map", "/velodyne", ros::Time(0), ros::Duration(1.0)));
   while(!tf_listener->waitForTransform("/map", "/laser", ros::Time(0), ros::Duration(1.0)));
   try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    ROS_INFO("Initial  Position : %3.2f, %3.2f, %3.2f - Yaw : %3.1f ", laser_orig.x(), laser_orig.y(), laser_orig.z(), transform.getRotation().getAngle()*PI/180);

    point3d next_vp(laser_orig.x(), laser_orig.y(),laser_orig.z());
    RPY2Quaternion(0, 0, 0.5, &qx, &qy, &qz, &qw);
    bool arrived = goToDest(next_vp, qx, qy, qz, qw);

    // Update the initial location of the robot
   while(!tf_listener->waitForTransform("/map", "/velodyne", ros::Time(0), ros::Duration(1.0)));
   while(!tf_listener->waitForTransform("/map", "/laser", ros::Time(0), ros::Duration(1.0)));
   try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    // Take a Second Scan
    ros::spinOnce();

    RPY2Quaternion(0, 0, 1.0, &qx, &qy, &qz, &qw);
    arrived = goToDest(next_vp, qx, qy, qz, qw);
    // Update the initial location of the robot
   while(!tf_listener->waitForTransform("/map", "/velodyne", ros::Time(0), ros::Duration(1.0)));
   while(!tf_listener->waitForTransform("/map", "/laser", ros::Time(0), ros::Duration(1.0)));
   try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    // Take a Third Scan
    ros::spinOnce();

    // Visualize current position
    RPY2Quaternion(0, 0, transform.getRotation().getAngle(), &qx, &qy, &qz, &qw);

    // Publish the jackal_frame as a Marker in rviz
    visualization_msgs::Marker jackal_frame;
    jackal_frame.header.frame_id = "map";
    jackal_frame.header.stamp = ros::Time();
    jackal_frame.ns = "robot_frame";
    jackal_frame.id = 0;
    jackal_frame.type = visualization_msgs::Marker::ARROW;
    jackal_frame.action = visualization_msgs::Marker::ADD;
    jackal_frame.pose.position.x = position.x();
    jackal_frame.pose.position.y = position.y();
    jackal_frame.pose.position.z = position.z();
    jackal_frame.pose.orientation.x = qx;
    jackal_frame.pose.orientation.y = qy;
    jackal_frame.pose.orientation.z = qz;
    jackal_frame.pose.orientation.w = qw;
    jackal_frame.scale.x = 0.5;
    jackal_frame.scale.y = 0.1;
    jackal_frame.scale.z = 0.1;
    jackal_frame.color.a = 1.0; // Don't forget to set the alpha!
    jackal_frame.color.r = 1.0;
    jackal_frame.color.g = 0.0;
    jackal_frame.color.b = 0.0;
    JackalMarker_pub.publish( jackal_frame );

    // Initial Scan
    // ros::spinOnce();

    while (ros::ok())
    {
        // Generate Candidates
        vector<pair<point3d, point3d>> candidates = generate_candidates(laser_orig);
        vector<double> MIs(candidates.size());
        double before = get_free_volume(cur_tree);
        max_idx = 0;

        // for every candidate...
        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            // Evaluate Mutual Information
            eu2dr.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            vector<point3d> hits = cast_sensor_rays(cur_tree, c.first, eu2dr);
            MIs[i] = calc_MI(cur_tree, c.first, hits, before);

            // Pick the Best Candidate
            if (MIs[i] > MIs[max_idx])
            {
                max_idx = i;
            }
        }
        next_vp = point3d(candidates[max_idx].first.x(),candidates[max_idx].first.y(),candidates[max_idx].first.z());
        ROS_INFO("Max MI : %f , @ location: %3.2f  %3.2f  %3.2f", MIs[max_idx], next_vp.x(), next_vp.y(), next_vp.z() );

        RPY2Quaternion(0, 0, candidates[max_idx].second.yaw(), &qx, &qy, &qz, &qw);

        // Publish the candidates as marker array in rviz
        double tmp_qx, tmp_qy, tmp_qz, tmp_qw;
        RPY2Quaternion(0, PI/2, 0, &tmp_qx, &tmp_qy, &tmp_qz, &tmp_qw);
        
        marker_array_msg.markers.resize(candidates.size());
        for (int i = 0; i < candidates.size(); i++)
        {
            marker_array_msg.markers[i].header.frame_id = "map";
            marker_array_msg.markers[i].header.stamp = ros::Time::now();
            marker_array_msg.markers[i].ns = "candidates";
            marker_array_msg.markers[i].id = i;
            marker_array_msg.markers[i].type = visualization_msgs::Marker::ARROW;
            marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
            marker_array_msg.markers[i].pose.position.x = candidates[i].first.x();
            marker_array_msg.markers[i].pose.position.y = candidates[i].first.y();
            marker_array_msg.markers[i].pose.position.z = candidates[i].first.z();
            marker_array_msg.markers[i].pose.orientation.x = tmp_qx;
            marker_array_msg.markers[i].pose.orientation.y = tmp_qy;
            marker_array_msg.markers[i].pose.orientation.z = tmp_qz;
            marker_array_msg.markers[i].pose.orientation.w = tmp_qw;
            marker_array_msg.markers[i].scale.x = MIs[i]/MIs[max_idx] + 0.01;
            marker_array_msg.markers[i].scale.y = 0.05;
            marker_array_msg.markers[i].scale.z = 0.05;
            marker_array_msg.markers[i].color.a = 1.0;
            marker_array_msg.markers[i].color.r = 0.0;
            marker_array_msg.markers[i].color.g = 1.0;
            marker_array_msg.markers[i].color.b = 0.0;
        }
        Candidates_pub.publish(marker_array_msg);

        // Publish the goal as a Marker in rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = next_vp.x();
        marker.pose.position.y = next_vp.y();
        marker.pose.position.z = next_vp.z();
        marker.pose.orientation.x = qx;
        marker.pose.orientation.y = qy;
        marker.pose.orientation.z = qz;
        marker.pose.orientation.w = qw;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        GoalMarker_pub.publish( marker );

        // Send the Robot 
        arrived = goToDest(next_vp, qx, qy, qz, qw);

        if(arrived)
        {
            // Update the initial location of the robot
           while(!tf_listener->waitForTransform("/map", "/velodyne", ros::Time(0), ros::Duration(1.0)));
           while(!tf_listener->waitForTransform("/map", "/laser", ros::Time(0), ros::Duration(1.0)));
           try{
                tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
                velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            }
                catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
            // Update Octomap
            ros::spinOnce();
            ROS_INFO("Succeed, new Map Free Volume: %f", get_free_volume(cur_tree));
        }
        else
        {
            ROS_ERROR("Failed to navigate to goal");
        }
        // r.sleep();
    }
    nh.shutdown();          
    return 0;
}
