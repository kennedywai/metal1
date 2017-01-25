/**
Random Wandering
**/
#include <ros/ros.h>
#include <angles/angles.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/GetMap.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>

#include <vector>
#include <list>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define MIN_SCAN_ANGLE_RAD -30.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD +30.0/180*M_PI
#define MIN_ANGLE_RAD -M_PI
#define MAX_ANGLE_RAD M_PI
#define MIN_RADIUS 0.4
#define MAX_RADIUS 0.8
#define threshold_for_costmap_ 23
#define RANDOM_NUMBER_SET 40

// Grid map definition
int rows;
int cols;
vector<vector<bool> > grid;
//vector<vector<int> > cost_values;
double roll, pitch, yaw;

class RandomWalk{
	public:
	RandomWalk();
	
	private:
	// Reading Map Data
	nav_msgs::OccupancyGrid map_;
	// Robot pose on the global frame referential
	geometry_msgs::PoseStamped robot_pose_;
	geometry_msgs::PoseWithCovarianceStamped robot_pose_amcl_;
	// Goal 
	move_base_msgs::MoveBaseGoal goal;
	// Node handler
	ros::NodeHandle n_;

	// Map subscriber.
	ros::Subscriber map_sub_;
	// Robot Pose subscriber
	ros::Subscriber robot_pose_sub_;

	// Odometry topic subscriber.
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;
	
	// Data structure that holds the visited cells
	nav_msgs::GridCells cells_;
	
	// Map data for SLAM map, global costmap and local costmap
	// Number of known cells on the map
	int map_known_cells_;
	// Number of uknow cells on the map
	int map_unknown_cells_;
	// Number of occupied cells on the map
	int map_occupied_cells_;
	// Number of free cells on the map
	int map_free_cells_;
	// Map X-Y Coordinates
	float min_x, max_x, min_y, max_y;
	// Goal set 
	bool goal_set = false;
	
	// Move Base Action Server.
	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac_;
	
	// Inflation radius from the nav stack
	double inflation_radius_;
	
	ros::Time start_time_;
	
	void setGoal();
	bool checkCell(int goal_cell_x, int goal_cell_y);
	void readMap();
	void printGridToFile();
	void circularDistribution();
	double random_double(double Min, double Max);
	
	void odomCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg);
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void robotposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	
	void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
	void goalActiveCallback();
	void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

};


RandomWalk::RandomWalk() : ac_("move_base", true){
	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac_.waitForServer(ros::Duration(60));
	ROS_INFO("RandomWalk -- Found the GOD DAMN move base!");
	
	// Subscribe to topics
	odom_sub_.subscribe(n_, "andbot/odom_diffdrive", 10);
	tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tf_, "map", 10);
    tf_filter_->registerCallback( boost::bind(&RandomWalk::odomCallback, this, _1) );
	robot_pose_sub_ = n_.subscribe("amcl_pose", 1, &RandomWalk::robotposeCallback, this);
	map_sub_ = n_.subscribe("move_base/global_costmap/costmap", 1, &RandomWalk::mapCallback, this);

	n_.param("move_base/local_costmap/inflation_radius", inflation_radius_, 0.60);
	ROS_INFO("inflation_radius_= %lf", inflation_radius_);

	map_known_cells_ = 0;
	map_unknown_cells_ = 0;
	map_occupied_cells_ = 0;
	map_free_cells_ = 0;
	
	min_x = 0.0;
	max_x = 18.0;
	min_y = 0.0;
	max_y = 33.6;

	start_time_ = ros::Time::now();
}

void RandomWalk::setGoal(){
	int x, y;

	do{
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
	
		x = rand() % map_.info.width;// pixel values in range [0,735]
		y = rand() % map_.info.height;// pixel values in range [0,671]

		goal.target_pose.pose.position.x = x*map_.info.resolution;
		goal.target_pose.pose.position.y = y*map_.info.resolution;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(random_double(MIN_ANGLE_RAD, MAX_ANGLE_RAD));
		//ROS_INFO("YAW:%.3f", random_double(MIN_ANGLE_RAD, MAX_ANGLE_RAD));
		//ROS_INFO("RandomWalk - %s - Testing %lf %lf...", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		
		if(!n_.ok()){
			ROS_INFO("RandomWalk - %s - Exiting...", __FUNCTION__);
			return;
		}
	}
	while(!checkCell(x, y) || (goal.target_pose.pose.position.x <= min_x) || (goal.target_pose.pose.position.x >= max_x) || (goal.target_pose.pose.position.y <= min_y) || (goal.target_pose.pose.position.y >= max_y) );
	ROS_INFO("RandomWalk - %s - will send the robot to X:%lf Y:%lf W:%.3f", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation);
	// Sending the goal
	ac_.sendGoal(goal, boost::bind(&RandomWalk::goalDoneCallback, this,  _1, _2), boost::bind(&RandomWalk::goalActiveCallback, this), boost::bind(&RandomWalk::goalFeedbackCallback, this, _1));
}

// Checking the cell to see if its a free cell inside the static map and the global/local costmap(required)
bool RandomWalk::checkCell(int goal_cell_x, int goal_cell_y){
			
	int cir = ceil(inflation_radius_/map_.info.resolution);	
    rows = map_.info.height;
    cols = map_.info.width;
    int currCell = 0;

	// Dynamically resize the grid
    grid.resize(rows);
    for (int i = 0; i < rows; i++){
        grid[i].resize(cols);
    }

    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            if (map_.data[currCell] == 0 && map_.data[currCell] <= threshold_for_costmap_) //absolutely free and unoccupied cells
                grid[i][j] = false;
            else
                grid[i][j] = true; // occupied (100) or unknown cell (-1)
            currCell++;
        }
    }

	if(grid[goal_cell_y][goal_cell_x]){
		return false;
	}
	else {
		return true;
	}
}

void RandomWalk::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	ROS_INFO("RandomWalk - %s - Got the map!", __FUNCTION__);
	map_ = *msg;
	//ROS_INFO("RandomWalk - %s - map_known_cells_:%d!", __FUNCTION__, map_known_cells_ );
	if(map_known_cells_ == 0){
		// width = 736
		for(int i=0 ; i<map_.info.width; i++){
			// height = 672
			for(int j=0 ; j<map_.info.height; j++){
				//ROS_INFO("MAP CELL OF [%d] = %d", j*map_.info.width+i, map_.data[j*map_.info.width+i]);
				if(map_.data[j*map_.info.width+i] >= 0 && map_.data[j*map_.info.width+i] <= 20) 
					map_known_cells_++;
				//ROS_INFO("i:%d j:%d", i, j);
				else if(map_.data[j*map_.info.width+i] == -1)
					map_unknown_cells_++;
				else if(map_.data[j*map_.info.width+i] >= 70 && map_.data[j*map_.info.width+i] <=100)
					map_occupied_cells_++;
				else 
					map_free_cells_++;
			}
		}
	}
	/*
	ROS_INFO("RandomWalk - %s - map_known_cells_:%d!", __FUNCTION__, map_known_cells_);
	ROS_INFO("RandomWalk - %s - map_unknown_cells_:%d!", __FUNCTION__, map_unknown_cells_);
	ROS_INFO("RandomWalk - %s - map_occupied_cells_:%d!", __FUNCTION__, map_occupied_cells_);
	ROS_INFO("RandomWalk - %s - map_free_cells_:%d!", __FUNCTION__, map_free_cells_);
	*/
	//readMap();
	setGoal();
}

void RandomWalk::odomCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg){
	ROS_DEBUG("RandomWalk - %s - Got an odom msg!", __FUNCTION__);
	
	geometry_msgs::PoseStamped odom;
	odom.header.frame_id = msg->header.frame_id;
	odom.header.stamp = msg->header.stamp;
	odom.pose.position.x = msg->pose.pose.position.x;	
	odom.pose.position.y = msg->pose.pose.position.y;
	odom.pose.orientation = msg->pose.pose.orientation;
	
	try{
		// Transform the odom into a pose in the map frame
		tf_.transformPose("map", odom, robot_pose_);
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("RandomWalk - %s - Error: %s", __FUNCTION__, ex.what());
		return;
	}
}

void RandomWalk::robotposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	//ROS_INFO("RandomWalk - %s - Got the robot pose!", __FUNCTION__);
	robot_pose_amcl_ = *msg;
	tf::Quaternion q(robot_pose_amcl_.pose.pose.orientation.x, robot_pose_amcl_.pose.pose.orientation.y,robot_pose_amcl_.pose.pose.orientation.z, robot_pose_amcl_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
	//ROS_INFO("POSITION = X:%lf Y:%lf Z:%lf ", robot_pose_amcl_.pose.pose.position.x, robot_pose_amcl_.pose.pose.position.y, robot_pose_amcl_.pose.pose.position.z);
	//setSmoothGoal();
}

void RandomWalk::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){
	ROS_INFO("RandomWalk - %s - Reached Destination", __FUNCTION__);
	//if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) ;	
	//if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) ;
	//ac_.sendGoal(goal, boost::bind(&RandomWalk::goalDoneCallback, this,  _1, _2), boost::bind(&RandomWalk::goalActiveCallback, this), boost::bind(&RandomWalk::goalFeedbackCallback, this, _1));
	//setGoal();
	setGoal();
}

void RandomWalk::goalActiveCallback(){
}

void RandomWalk::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
}

void RandomWalk::readMap(){
    ROS_INFO("Received a %d X %d map @ %.3f m/px\n", map_.info.width, map_.info.height, map_.info.resolution);

    rows = map_.info.height;
    cols = map_.info.width;
    int currCell = 0;
	
    // Dynamically resize the grid
    grid.resize(rows);
    for (int i = 0; i < rows; i++){
        grid[i].resize(cols);
    }
	/* reverse loop
    for (int i = rows; i-- > 0;){
        for (int j =cols; j-- > 0;){
            if (map_.data[currCell] == 0 ) // unoccupied cell
                grid[i][j] = false;
            else
                grid[i][j] = true; // occupied (100) or unknown cell (-1)
            currCell++;
        }
    }
	*/
	for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            if (map_.data[currCell] >= 0 && map_.data[currCell] <= threshold_for_costmap_) // unoccupied cell
                grid[i][j] = false;
            else
                grid[i][j] = true; // occupied (100) or unknown cell (-1)
            currCell++;
        }
    }
	printGridToFile();
}

void RandomWalk::printGridToFile(){
    ofstream gridFile;
    // file location:/home/kennedywai/catkin_ws/devel/lib/random_walk
    gridFile.open("global_costmap_lastest.txt");

    for (int i = grid.size() - 1; i >= 0; i--){        
        for (int j = 0; j < grid[0].size() - 1; j++){
	    gridFile << (grid[i][j] ? "1" : "0");           
        }
        gridFile << endl;
    }
	
    gridFile.close();
	ROS_INFO("MAP PRINTED");
}

double RandomWalk::random_double(double Min, double Max){
	//return ((float(rand())/float(RAND_MAX))*(Max - Min)) + Min;
	/*
	float range = (Max - Min); 
    float div = RAND_MAX / range;
    return Min + (rand() / div);
	*/
	int r = rand()%6284 - 3142; //this produces numbers between -3142 - +3142
	double random_num = r/1000.0; //this will create random floating point numbers between -0.2 upto //0.2
	return random_num;
}

int main(int argc, char **argv){
  	ros::init(argc, argv, "random_walk_node");
  	
  	ROS_INFO("RandomWalk for Adamgo/Andbot!!!");
	// initialize random seed: 
	// Make sure that we get a different random number when we run this program
	srand(time(NULL));

	RandomWalk rw;
	ros::spin();
	
 	return(0);
}