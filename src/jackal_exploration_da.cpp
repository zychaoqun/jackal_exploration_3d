// Related headers:
#include "exploration.h"
#include "navigation_utils.h"

//C library headers:
#include <iostream>
#include <fstream>
// #include <chrono>
// #include <iterator>
// #include <ctime>

//C++ library headers:  NONE
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//other library headers:  NONE


using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "jackal_exploration_da");
    ros::NodeHandle nh;

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
    // ros::Subscriber hokuyo_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hokuyo_points", 1, hokuyo_callbacks);
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Goal_Marker", 1 );
    ros::Publisher JackalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Jackal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("Candidate_MIs", 1);
    ros::Publisher Octomap_marker_pub = nh.advertise<visualization_msgs::Marker>("Occupied_MarkerArray", 1);
    ros::Publisher Frontier_points_pub = nh.advertise<visualization_msgs::Marker>("Frontier_points", 1);
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher Octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_3d",1);


    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading;

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker OctomapOccupied_cubelist;
    visualization_msgs::Marker Frontier_points_cubelist;

    geometry_msgs::Twist twist_cmd;
    
    ros::Time now_marker = ros::Time::now();
   
    double R_velo, P_velo, Y_velo;

    // Initialize parameters 
    position = point3d(0, 0, 0.0);
    point3d Sensor_PrincipalAxis(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    octomap::OcTree new_tree_2d(octo_reso);
    cur_tree = &new_tree;
    cur_tree_2d = &new_tree_2d;

    bool got_tf = false;
    bool arrived;
    point3d next_vp;

    // Update the pose of velodyne from predefined tf.
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/base_link", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf::Matrix3x3(transform.getRotation()).getRPY(R_velo, P_velo, Y_velo);
        ROS_INFO("Current Velodyne heading: vector(%2.2f, %2.2f, %2.2f) -  RPY(%3.1f, %3.1f, %3.1f).", 
            velo_orig.x(), velo_orig.y(), velo_orig.z(), R_velo/PI*180.0, P_velo/PI*180.0, Y_velo/PI*180.0);
        got_tf = true;
        }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: initial pose of Velodyne"); 
        ros::Duration(0.05).sleep();
        } 
    }   
    
    // Rotate Sensor Model based on Velodyn Pose
    velodynePuck.SensorRays.rotate(R_velo, P_velo, Y_velo);
    
    // Initialize the map
    for(int o =0; o < 1; o++){
        // Update the pose of the robot
        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);// need to change tf of kinect###############
            velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: velodyne frame"); 
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
            ROS_WARN("Wait for tf: LaserScan frame"); 
        } 
        ros::Duration(0.05).sleep();
        }

        // Take a Scan
        ros::spinOnce();

        // Rotate another 60 degrees
        twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
        ros::Time start_turn = ros::Time::now();

        ROS_WARN("Rotate 60 degrees");
        while (ros::Time::now() - start_turn < ros::Duration(2.6)){ // turning duration - second
        twist_cmd.angular.z = 0.6; // turning speed
        // turning angle = turning speed * turning duration / 3.14 * 180
        pub_twist.publish(twist_cmd);
        ros::Duration(0.05).sleep();
        }
        // stop
        twist_cmd.angular.z = 0;
        pub_twist.publish(twist_cmd);
    }


    // steps robot taken, counter
    int robot_step_counter = 0;
    int idx_ptr = 0;
    int num_of_samples = 12;

    while (ros::ok())
    {
        // Extract Frontier points
        vector<vector<point3d>> frontier_groups=extractFrontierPoints( cur_tree );
        
        // Visualize frontier points;
        unsigned long int o = 0;
        for(vector<vector<point3d>>::size_type e = 0; e < frontier_groups.size(); e++) {
            o = o+frontier_groups[e].size();
        }

        Frontier_points_cubelist.points.resize(o);
        ROS_INFO("frontier points %ld", o);
        now_marker = ros::Time::now();
        Frontier_points_cubelist.header.frame_id = "map";
        Frontier_points_cubelist.header.stamp = now_marker;
        Frontier_points_cubelist.ns = "frontier_points_array";
        Frontier_points_cubelist.id = 0;
        Frontier_points_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
        Frontier_points_cubelist.action = visualization_msgs::Marker::ADD;
        Frontier_points_cubelist.scale.x = octo_reso;
        Frontier_points_cubelist.scale.y = octo_reso;
        Frontier_points_cubelist.scale.z = octo_reso;
        Frontier_points_cubelist.color.a = 1.0;
        Frontier_points_cubelist.color.r = (double)255/255;
        Frontier_points_cubelist.color.g = 0;
        Frontier_points_cubelist.color.b = (double)0/255;
        Frontier_points_cubelist.lifetime = ros::Duration();

        unsigned long int t = 0;
        int l = 0;
        geometry_msgs::Point q;
        for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++) { 
            for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
               q.x = frontier_groups[n][m].x();
               q.y = frontier_groups[n][m].y();
               q.z = frontier_groups[n][m].z()+octo_reso;
               Frontier_points_cubelist.points.push_back(q); 
               
            }
            t++;
        }
        ROS_INFO("Publishing %ld frontier_groups", t);
        
        // publish frontier points for rviz 
        Frontier_points_pub.publish(Frontier_points_cubelist); 
        Frontier_points_cubelist.points.clear();    

        // Generate Candidates based on cue from frontiers
        vector<pair<point3d, point3d>> candidates = extractCandidateViewPoints(frontier_groups, laser_orig, num_of_samples);         
        ROS_INFO("%lu candidates generated.", candidates.size());
        frontier_groups.clear();
        
        // Evaluate MI for every candidate view points
        vector<double>  MIs(candidates.size());
        double before = countFreeVolume(cur_tree);
        // int max_idx = 0;
        double begin_mi_eva_secs, end_mi_eva_secs;
        begin_mi_eva_secs = ros::Time::now().toSec();

        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            // Evaluate Mutual Information
            Sensor_PrincipalAxis = point3d(1.0, 0.0, 0.0);
            Sensor_PrincipalAxis.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            octomap::Pointcloud hits = castSensorRays(cur_tree, c.first, Sensor_PrincipalAxis);
            
            // Considering pure MI for decision making
            // MIs[i] = calc_MI(cur_tree, c.first, hits, before);
            
            // Normalize the MI with distance
            MIs[i] = calc_MI(cur_tree, c.first, hits, before) / 
                sqrt(pow(c.first.x()-laser_orig.x(),2) + pow(c.first.y()-laser_orig.y(),2));

            // Pick the Candidate view point with max MI
            // if (MIs[i] > MIs[max_idx])
            // {
            //     max_idx = i;
            // }
        }

        // sort vector MIs, with idx_MI, descending
        vector<int> idx_MI = sort_MIs(MIs);

        end_mi_eva_secs = ros::Time::now().toSec();
        ROS_INFO("Mutual Infomation Eva took:  %3.3f Secs.", end_mi_eva_secs - begin_mi_eva_secs);

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
            CandidatesMarker_array.markers[i].scale.x = (double)2.0*MIs[i]/MIs[idx_MI[0]];
            CandidatesMarker_array.markers[i].scale.y = 0.2;
            CandidatesMarker_array.markers[i].scale.z = 0.2;
            CandidatesMarker_array.markers[i].color.a = (double)MIs[i]/MIs[idx_MI[0]];
            CandidatesMarker_array.markers[i].color.r = 0.0;
            CandidatesMarker_array.markers[i].color.g = 1.0;
            CandidatesMarker_array.markers[i].color.b = 0.0;
        }
        Candidates_pub.publish(CandidatesMarker_array);
        CandidatesMarker_array.markers.clear();
        candidates.clear();

        // loop in the idx_MI, if the candidate with max MI cannot be achieved, 
        // switch to a sub-optimal MI.
        arrived = false;
        int idx_ptr = 0;
        while (!arrived) {
            // Setup the Goal
            next_vp = point3d(candidates[idx_MI[idx_ptr]].first.x(),candidates[idx_MI[idx_ptr]].first.y(),candidates[idx_MI[idx_ptr]].first.z());
            Goal_heading.setRPY(0.0, 0.0, candidates[idx_MI[idx_ptr]].second.yaw());
            Goal_heading.normalize();
            ROS_INFO("Max MI : %f , @ location: %3.2f  %3.2f  %3.2f", MIs[idx_MI[idx_ptr]], next_vp.x(), next_vp.y(), next_vp.z() );
            
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
                ROS_INFO("Succeed, new Map Free Volume: %f", countFreeVolume(cur_tree));
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
                ROS_INFO("Publishing %ld occupied cells in RVIZ", j);
                Octomap_marker_pub.publish(OctomapOccupied_cubelist);
                OctomapOccupied_cubelist.points.clear();

                // prepare octomap msg
                octomap_msgs::binaryMapToMsg(*cur_tree, msg_octomap);
                msg_octomap.binary = 1;
                msg_octomap.id = 1;
                msg_octomap.resolution = octo_reso;
                msg_octomap.header.frame_id = "/map";
                msg_octomap.header.stamp = ros::Time::now();
                Octomap_pub.publish(msg_octomap);

                // Send out results to file.
                explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
                explo_log_file << "DA Step ," << robot_step_counter << ", Current Entropy ," << countFreeVolume(cur_tree) << ", time, " << ros::Time::now().toSec() << endl;
                explo_log_file.close();

            }
            else
            {
                ROS_WARN("Failed to drive to the %d th goal, switch to the sub-optimal..", idx_ptr);
                idx_ptr++;
                if(idx_ptr > MIs.size()) {
                    ROS_ERROR("None of the goal is valid for path planning, shuting down the node");
                    nh.shutdown();
                }
            }

        }

        
        // r.sleep();
    }
    nh.shutdown();          
    return 0;
}
