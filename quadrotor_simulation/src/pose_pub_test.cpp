#include "../include/quadrotor_simulation/pose_pub_test.h"

PosePubTest::PosePubTest(ros::NodeHandle* nodehandle, vector<vector<double>> input_path):nh_(*nodehandle), path(input_path) { 
// constructor
    ROS_INFO("In class constructor of PosePubTest");
    initVec();
    initSub();
    initPub();
}

void PosePubTest::initSub() {
    // ROS_INFO("Initializing Subscribers");  
    stop_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &PosePubTest::eventCallback,this);
    // currentpos_sub_ = nh_.subscribe("/drone1/ground_truth_to_tf/pose", 1, &PosePubTest::currentposCallback,this);
}

void PosePubTest::initVec() {
    currentPose = vector<double> (vector<double> (7, 0));
    goalPose = vector<double> (vector<double> (7, 0));

}

void PosePubTest::initPub() {
    // ROS_INFO("Initializing Publishers");
    goalpos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/drone1/command/pose", 1, true); 
    controlinput_pub_ = nh_.advertise<geometry_msgs::Twist>("/drone1/cmd_vel", 1, true); 
    markerarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_markerarray", 10);
}

void PosePubTest::eventCallback(const geometry_msgs::PoseStamped& odom1) {
    
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

PosePubTest::~PosePubTest()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_pub"); //node name
    //xy
    // vector<vector<double>> path{
    //     {0,0,0,0,0,0,1},
    //     {2.0,1.5,0.5,0,0,0,1},
    //     {2.5,3.0,0.8,0,0,0,1},
    //     {4.0,4.5,0.7,0,0,0,1},
    //     {4.5,6.0,0.8,0,0,0,1},
    //     {3.0,7.5,0.9,0,0,0,1},
    //     {2.5,9.0,0.8,0,0,0,1},
    //     {4.0,10.5,0.9,0,0,0,1},
    //     {4.5,12.0,0.8,0,0,0,1},
    //     {4.5,13.0,0.5,0,0,0,1},
    //     {4.5,14.0,0.2,0,0,0,1}};

    //xyz
    vector<vector<double>> path{
        {0,0,0,0,0,0,1},
        {2.0,1.5,0.5,0,0,0,1},
        {2.5,3.0,0.8,0,0,0,1},
        {4.0,4.5,1.0,0,0,0,1},
        {4.5,6.0,1.3,0,0,0,1},
        {3.0,7.5,0.9,0,0,0,1},
        {2.5,9.0,0.8,0,0,0,1},
        {4.0,10.5,0.9,0,0,0,1},
        {4.5,12.0,1.3,0,0,0,1},
        {4.5,14.0,0.5,0,0,0,1},
        {4.5,15.0,0.2,0,0,0,1}};

    // corridor
    // vector<vector<double>> path{
    //     {0,0,0,0,0,0,1},
    //     {4.0,-0.3,1.1,0,0,0,1},
    //     {6.0,0.2,1.0,0,0,0,1},
    //     {8.0,1.8,0.9,0,0,0,1},
    //     {5.2,4.0,1.3,0,0,0,1},
    //     {1.3,3.8,1.0,0,0,0,1},
    //     {0.2,5.8,1.2,0,0,0,1},
    //     {1.2,8.0,1.1,0,0,0,1},
    //     {4.0,7.9,0.7,0,0,0,1},
    //     {6.0,8.0,0.5,0,0,0,1},
    //     {8.0,8.5,0.2,0,0,0,1}};


    //circle
    // vector<vector<double>> path{
    //     {0,0,0,0,0,0,1},
    //     {2.0,0.7,0.6,0,0,0,1},
    //     {2.5,2.0,0.8,0,0,0,1},
    //     {4.0,3.5,0.7,0,0,0,1},
    //     {4.5,5.0,0.8,0,0,0,1},
    //     {5.0,8.0,1.0,0,0,0,1},
    //     {1.0,10.5,0.8,0,0,0,1},
    //     {-3.0,9.0,0.9,0,0,0,1},
    //     {-4.0,7.0,0.8,0,0,0,1},
    //     {-5.0,5.0,0.7,0,0,0,1},
    //     {-5.5,3.0,0.8,0,0,0,1},
    //     {-3.0,1.5,0.6,0,0,0,1},
    //     {0,0,0.2,0,0,0,1}};
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    PosePubTest posepub(&nh, path);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("Initializing posepub...");
    ros::spin();
    return 0;
}



