#ifndef POSE_PUB_H_
#define POSE_PUB_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h> 
#include <visualization_msgs/MarkerArray.h>  

// Input: vector<vector<double>> pose;
// Ex/: vector<vector<double>> path{{0,0,0,0,0,0,1},{2,2,2,0,0,0,1},{4,4,4,0,0,0,1},{6,6,6,0,0,0,1},{8,8,8,0,0,0,1}};
// px, py, pz, qx, qy, qz, qw
// Function: control the UAV to track the path by send PoseStamped msg to /drone1/command/pose

using namespace std;

class PosePub
{
private:
    string fileName_;
    string delimeter_; 
	vector<vector<double>> path;
	vector<double> currentPose;
	vector<double> goalPose;
	double reachedCheck = 0.3;
	int path_index = 0;
	bool reached = true;
    ros::NodeHandle nh_;
    ros::Subscriber currentpos_sub_;
    ros::Subscriber stop_sub_;
    ros::Publisher goalpos_pub_;
    ros::Publisher controlinput_pub_;
    geometry_msgs::Twist controlinput;
    // geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped goal_pose;
    visualization_msgs::MarkerArray path_marker_array;
    visualization_msgs::Marker path_marker;
    ros::Publisher markerarray_pub_;

public:
    PosePub(ros::NodeHandle* nodehandle, string fileName, string delimeter);
    void getPath();
    void initSub(); 
    void initPub();  
    void initVec();
    void setgoalpos();  
    void eventCallback(const geometry_msgs::PoseStamped& odom1); 
    ~PosePub();
};


#endif
