#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>
#include <ctime>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <tf/transform_listener.h>
#include <octomap/octomap.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include "navigation_utils.h"


using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
// const double free_prob = 0.3;
const double octo_reso = 0.1;

octomap::OcTree* cur_tree;
octomap::OcTree* cur_tree_2d;

tf::TransformListener *tf_listener; 
   
point3d position, laser_orig, velo_orig;

ofstream explo_log_file;
std::string octomap_name_2d, octomap_name_3d;


struct SensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    SensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
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
SensorModel Velodyne_puck(360, 16, 2*PI, 0.5236, 30.0);


double get_free_volume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}


octomap::Pointcloud cast_sensor_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    octomap::Pointcloud hits;

    octomap::Pointcloud SensorRays_copy;
    SensorRays_copy.push_back(Velodyne_puck.SensorRays);
    SensorRays_copy.rotate(0.0,0.0,direction.z());
    point3d end;
    // #pragma omp parallel for
    for(int i = 0; i < SensorRays_copy.size(); i++) {
        if(octree->castRay(position, SensorRays_copy.getPoint(i), end, true, Velodyne_puck.max_range)) {
            // hits.push_back(end);
        } else {
            end = SensorRays_copy.getPoint(i) * Velodyne_puck.max_range;
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

    octree_copy->insertPointCloud(hits, sensor_orig, Velodyne_puck.max_range, true, true);
    double after = get_free_volume(octree_copy);
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
    // hits.push_back(point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z));
    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        // if(isnan(cloud->at(j).x)) continue;
        if(cloud->at(j).z < -1.0)    continue;  
        hits.push_back(point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z));
        // cur_tree->insertRay(point3d( velo_orig.x(),velo_orig.y(),velo_orig.z()), 
        //     point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), Velodyne_puck.max_range);
    }

    cur_tree->insertPointCloud(hits, velo_orig, Velodyne_puck.max_range);
    // cur_tree->updateInnerOccupancy();
    ROS_INFO("Entropy(3d map) : %f", get_free_volume(cur_tree));

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
    cur_tree_2d->insertPointCloud(hits, laser_orig, Velodyne_puck.max_range);
    // cur_tree_2d->updateInnerOccupancy();
    ROS_INFO("Entropy(2d map) : %f", get_free_volume(cur_tree_2d));
    cur_tree_2d->write(octomap_name_2d);
    delete cloud;
    delete cloud_local;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "jackal_exploration_da");
    ros::NodeHandle nh;
    // ros::Subscriber octomap_sub;
    // octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, octomap_callback);

    // Initialize time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,80,"Trajectory_%R:%S_%m%d_DA.txt",timeinfo);
    std::string logfilename(buffer);
    std::cout << logfilename << endl;
    strftime(buffer,80,"octomap_2d_%R:%S_%m%d_DA.ot",timeinfo);
    octomap_name_2d = buffer;
    strftime(buffer,80,"octomap_3d_%R:%S_%m%d_DA.ot",timeinfo);
    octomap_name_3d = buffer;


    ros::Subscriber velodyne_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, velodyne_callbacks);
    ros::Subscriber hokuyo_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hokuyo_points", 1, hokuyo_callbacks);
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Goal_Marker", 1 );
    ros::Publisher JackalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Jackal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("Candidate_MIs", 1);
    ros::Publisher Octomap_marker_pub = nh.advertise<visualization_msgs::Marker>("Occupied_MarkerArray", 1);


    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading;

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker OctomapOccupied_cubelist;
    
    ros::Time now_marker = ros::Time::now();
   
    double R_velo, P_velo, Y_velo;

    // Initialize parameters 
    // ros::Rate r(10); // 1 hz
    int max_idx = 0;

    position = point3d(0, 0, 0.0);
    point3d Sensor_PrincipalAxis(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    octomap::OcTree new_tree_2d(octo_reso);
    cur_tree = &new_tree;
    cur_tree_2d = &new_tree_2d;

    bool got_tf = false;

    // Update the pose of velodyne from predefined tf.
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/base_link", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf::Matrix3x3(transform.getRotation()).getRPY(R_velo, P_velo, Y_velo);
        Sensor_PrincipalAxis.rotate_IP(R_velo, P_velo, Y_velo);
        ROS_INFO("Current Velodyne heading: vector(%2.2f, %2.2f, %2.2f) -  RPY(%3.1f, %3.1f, %3.1f).", Sensor_PrincipalAxis.x(), Sensor_PrincipalAxis.y(), Sensor_PrincipalAxis.z(), R_velo/PI*180.0, P_velo/PI*180.0, Y_velo/PI*180.0);
        got_tf = true;
        }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: initial pose of Velodyne"); 
        ros::Duration(0.05).sleep();
        } 
    }   
    
    // Rotate Sensor Model based on Velodyn Pose
    Velodyne_puck.SensorRays.rotate(R_velo, P_velo, Y_velo);
    
    // Update the initial location of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }
    ROS_INFO("Initial  Position : %3.2f, %3.2f, %3.2f - Yaw : %3.1f ", laser_orig.x(), laser_orig.y(), laser_orig.z(), transform.getRotation().getAngle()*PI/180);
    // Take a Initial Scan
    ros::spinOnce();

    // Rotate ~60 degrees 
    point3d next_vp(laser_orig.x(), laser_orig.y(),laser_orig.z());
    Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle()+0.5233);
    Goal_heading.normalize();
    bool arrived = goToDest(laser_orig, Goal_heading);

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    // Take a Second Scan
    ros::spinOnce();

    // Rotate another 60 degrees
    Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle()+0.5233);
    arrived = goToDest(laser_orig, Goal_heading);

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    // Take a Third Scan
    ros::spinOnce();

    // Rotate another 60 degrees
    Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle()+0.5233);
    arrived = goToDest(laser_orig, Goal_heading);

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }
    // Take a Third Scan
    ros::spinOnce();

    // steps robot taken, counter
    int robot_step_counter = 0;

    while (ros::ok())
    {
        // Generate Candidates
        vector<pair<point3d, point3d>> candidates = generate_candidates(laser_orig, transform.getRotation().getAngle());
        while(candidates.size() <= 1)
        {
            // Get the current heading
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: laser to map"); 
            } 
            ros::Duration(0.05).sleep();
            }
            // Rotate negative along Yaw for 30 deg to look for more open areas
            Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle() - PI/6);
            Goal_heading.normalize();
            arrived = goToDest(laser_orig, Goal_heading);
            vector<pair<point3d, point3d>> candidates = generate_candidates(laser_orig, transform.getRotation().getAngle());
        }
        
        ROS_INFO("%lu candidates generated.", candidates.size());
        vector<double> MIs(candidates.size());
        double before = get_free_volume(cur_tree);
        max_idx = 0;

        // for every candidate...
        double Secs_CastRay, Secs_InsertRay, Secs_tmp;
        Secs_InsertRay = 0;
        Secs_CastRay = 0;

        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            // Evaluate Mutual Information
            Secs_tmp = ros::Time::now().toSec();
            Sensor_PrincipalAxis.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            octomap::Pointcloud hits = cast_sensor_rays(cur_tree, c.first, Sensor_PrincipalAxis);
            Secs_CastRay += ros::Time::now().toSec() - Secs_tmp;
            Secs_tmp = ros::Time::now().toSec();
            MIs[i] = calc_MI(cur_tree, c.first, hits, before);
            Secs_InsertRay += ros::Time::now().toSec() - Secs_tmp;
            // Pick the Best Candidate
            if (MIs[i] > MIs[max_idx])
            {
                max_idx = i;
            }
        }

        next_vp = point3d(candidates[max_idx].first.x(),candidates[max_idx].first.y(),candidates[max_idx].first.z());
        Goal_heading.setRPY(0.0, 0.0, candidates[max_idx].second.yaw());
        Goal_heading.normalize();
        ROS_INFO("Max MI : %f , @ location: %3.2f  %3.2f  %3.2f", MIs[max_idx], next_vp.x(), next_vp.y(), next_vp.z() );
        ROS_INFO("CastRay Time: %2.3f Secs. InsertRay Time: %2.3f Secs.", Secs_CastRay, Secs_InsertRay);

        // Publish the candidates as marker array in rviz
        tf::Quaternion MI_heading;
        MI_heading.setRPY(0.0, -PI/2, 0.0);
        MI_heading.normalize();
        
        CandidatesMarker_array.markers.resize(candidates.size());
        for (int i = 0; i < candidates.size(); i++)
        {
            CandidatesMarker_array.markers[i].header.frame_id = "map";
            CandidatesMarker_array.markers[i].header.stamp = ros::Time::now();
            CandidatesMarker_array.markers[i].ns = "candidates";
            CandidatesMarker_array.markers[i].id = i;
            CandidatesMarker_array.markers[i].type = visualization_msgs::Marker::ARROW;
            CandidatesMarker_array.markers[i].action = visualization_msgs::Marker::ADD;
            CandidatesMarker_array.markers[i].pose.position.x = candidates[i].first.x();
            CandidatesMarker_array.markers[i].pose.position.y = candidates[i].first.y();
            CandidatesMarker_array.markers[i].pose.position.z = candidates[i].first.z();
            CandidatesMarker_array.markers[i].pose.orientation.x = MI_heading.x();
            CandidatesMarker_array.markers[i].pose.orientation.y = MI_heading.y();
            CandidatesMarker_array.markers[i].pose.orientation.z = MI_heading.z();
            CandidatesMarker_array.markers[i].pose.orientation.w = MI_heading.w();
            CandidatesMarker_array.markers[i].scale.x = (double)MIs[i]/MIs[max_idx];
            CandidatesMarker_array.markers[i].scale.y = 0.05;
            CandidatesMarker_array.markers[i].scale.z = 0.05;
            CandidatesMarker_array.markers[i].color.a = (double)MIs[i]/MIs[max_idx];
            CandidatesMarker_array.markers[i].color.r = 0.0;
            CandidatesMarker_array.markers[i].color.g = 1.0;
            CandidatesMarker_array.markers[i].color.b = 0.0;
        }
        Candidates_pub.publish(CandidatesMarker_array);
        CandidatesMarker_array.markers.clear();
        candidates.clear();

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
        marker.pose.position.z = 1.0;
        marker.pose.orientation.x = Goal_heading.x();
        marker.pose.orientation.y = Goal_heading.y();
        marker.pose.orientation.z = Goal_heading.z();
        marker.pose.orientation.w = Goal_heading.w();
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        GoalMarker_pub.publish( marker );

        // Send the Robot 
        Goal_heading.setRPY(0.0, 0.0, candidates[max_idx].second.yaw());
        arrived = goToDest(next_vp, Goal_heading);

        if(arrived)
        {
            // Update the initial location of the robot
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
                velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: velodyne to map"); 
            } 
            ros::Duration(0.05).sleep();
            }

            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: laser to map"); 
            } 
            ros::Duration(0.05).sleep();
            }

            // Update Octomap
            ros::spinOnce();
            ROS_INFO("Succeed, new Map Free Volume: %f", get_free_volume(cur_tree));
            robot_step_counter++;

            // Prepare the header for occupied array
            now_marker = ros::Time::now();
            OctomapOccupied_cubelist.header.frame_id = "map";
            OctomapOccupied_cubelist.header.stamp = now_marker;
            OctomapOccupied_cubelist.ns = "octomap_occupied_array";
            OctomapOccupied_cubelist.id = 0;
            OctomapOccupied_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            OctomapOccupied_cubelist.action = visualization_msgs::Marker::ADD;
            OctomapOccupied_cubelist.scale.x = octo_reso;
            OctomapOccupied_cubelist.scale.y = octo_reso;
            OctomapOccupied_cubelist.scale.z = octo_reso;
            OctomapOccupied_cubelist.color.a = 0.5;
            OctomapOccupied_cubelist.color.r = (double)19/255;
            OctomapOccupied_cubelist.color.g = (double)121/255;
            OctomapOccupied_cubelist.color.b = (double)156/255;

            unsigned long int j = 0;
            geometry_msgs::Point p;
            for(octomap::OcTree::leaf_iterator n = cur_tree->begin_leafs(cur_tree->getTreeDepth()); n != cur_tree->end_leafs(); ++n) {
                if(!cur_tree->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                OctomapOccupied_cubelist.points.push_back(p); 
                j++;
            }
            ROS_INFO("Publishing %ld occupied cells", j);
            Octomap_marker_pub.publish(OctomapOccupied_cubelist);
            OctomapOccupied_cubelist.points.clear();

            // Send out results to file.
            explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
            explo_log_file << "DA Step: " << robot_step_counter << "  | Current Entropy: " << get_free_volume(cur_tree) << endl;
            explo_log_file.close();

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
