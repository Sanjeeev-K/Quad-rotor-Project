#include "../include/quadrotor_simulation/pose_pub.h"

PosePub::PosePub(ros::NodeHandle* nodehandle, string fileName, string delimeter):nh_(*nodehandle), 
        fileName_(fileName), delimeter_(delimeter) { 
// constructor
    ROS_INFO("In class constructor of PosePub");
    getPath();
    initVec();
    initSub();
    initPub();
}

void PosePub::getPath() {
    ifstream file(fileName_);
    string line = "";
    vector<vector<string>> dataList;
    // Iterate through each line and split the content using delimeter
    while (getline(file, line))  {
        vector<string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter_));
        dataList.push_back(vec);

    }
    // Close the File
    file.close();
    for (auto rows : dataList) {
        vector<double> pathEle;
        for (auto ele : rows) {
            pathEle.push_back(stod(ele));
        }
        path.push_back(pathEle);
    }
}

void PosePub::initSub() {
    // ROS_INFO("Initializing Subscribers");  
    stop_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &PosePub::eventCallback,this);
    // currentpos_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &PosePub::currentposCallback,this);
}

void PosePub::initVec() {
    currentPose = vector<double> (vector<double> (7, 0));
    goalPose = vector<double> (vector<double> (7, 0));

}

void PosePub::initPub() {
    // ROS_INFO("Initializing Publishers");
    goalpos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/drone1/command/pose", 1, true); 
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    markerarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_markerarray", 10);
}

void PosePub::eventCallback(const geometry_msgs::PoseStamped& odom1) {
    
    currentPose[0] = odom1.pose.position.x;
    currentPose[1] = odom1.pose.position.y;
    currentPose[2] = odom1.pose.position.z;
    // currentPose[3] = odom3.pose.orientation.x; 
    // currentPose[4] = odom3.pose.orientation.y;
    // currentPose[5] = odom3.pose.orientation.z;
    // currentPose[6] = odom3.pose.orientation.w;
    goalPose[0] = path[path_index][0];
    goalPose[1] = path[path_index][1];
    goalPose[2] = path[path_index][2];
    // cout << "enter" << endl;

    path_marker.type = visualization_msgs::Marker::CUBE_LIST;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.ns = "paths";
    path_marker.scale.x = 0.1;
    path_marker.scale.y = 0.1;
    path_marker.scale.z = 0.1;  
    path_marker.header.frame_id = "/world";
    path_marker.color.a = 1.0; 
    path_marker.id = 0;
    path_marker.lifetime = ros::Duration();
    geometry_msgs::Point obj;
    obj.x = odom1.pose.position.x;
    obj.y = odom1.pose.position.y;
    // std::cout << "pos: " << obj.x << " " << obj.y << std::endl;
    obj.z = odom1.pose.position.z;
    path_marker.points.push_back(obj);
    path_marker.color.g = 0.0;
    path_marker.color.r = 1.0;

    path_marker.colors.push_back(path_marker.color);
    path_marker.header.stamp = ros::Time::now();
    path_marker_array.markers.push_back(path_marker);
    markerarray_pub_.publish(path_marker_array);
    path_marker_array.markers.pop_back();

    int pathsize = path.size();
    // cout << "Xinit pose: " << currentPose[0] << " Yinit pose: " << currentPose[1] << " Zinit pose: " << currentPose[2] <<endl;
    // cout << "Xtarget pose: " << goalPose[0] << " Ytarget pose: " << goalPose[1] << " Ztarget pose: " << goalPose[2] <<endl;
    if ((abs(currentPose[0] - goalPose[0])<reachedCheck) && (abs(currentPose[1] - goalPose[1])<reachedCheck ) 
        && (abs(currentPose[2] - goalPose[2])<reachedCheck)) {   
        
        path_index++;

        goalPose[0] = path[path_index][0];
        goalPose[1] = path[path_index][1];
        goalPose[2] = path[path_index][2];
        // goalPose[3] = path[path_index-1][3];
        // goalPose[4] = path[path_index-1][4];
        // goalPose[5] = path[path_index-1][5];
        // goalPose[6] = path[path_index-1][6];
        goal_pose.header.frame_id="world";
        // goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose.position.x = goalPose[0];
        goal_pose.pose.position.y = goalPose[1];
        goal_pose.pose.position.z = goalPose[2];
        goal_pose.pose.orientation.x = goalPose[3];
        goal_pose.pose.orientation.y = goalPose[4];
        goal_pose.pose.orientation.z = goalPose[5];
        goal_pose.pose.orientation.w = goalPose[6];
        goalpos_pub_.publish(goal_pose);
    }
}

PosePub::~PosePub()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_pub"); //node name
    string fileName = "/home/han/quadrotor_ws/src/quadrotor_demo/path.csv";
    string delimeter = ",";
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    PosePub posepub(&nh, fileName, delimeter);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing posepub...");
    ros::spin();
    return 0;
}



