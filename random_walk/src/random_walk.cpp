/**
Smooth Wandering
**/
#include <ros/ros.h>
#include <angles/angles.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <vector>
#include <list>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RandomWalk{
	public:
	RandomWalk();
	~RandomWalk();
	
	private:
	/**
	*An array of cells in a 2D grid
	Header header
	float32 cell_width
	float32 cell_height
	geometry_msgs/Point[] cells
	**/
	
	//! Data structure that holds the obstacle cells
	nav_msgs::GridCells obstacles_;
	// Data structure that holds the inflated obstacle cells
	nav_msgs::GridCells inflated_;
	
	/**
	*This represents a 2-D grid map, in which each cell represents the probability ofoccupancy.
	Header header 

	*MetaData for the map
	MapMetaData info

	*The map data, in row-major order, starting with (0,0). 
	Occupancy is represented as an integer in the range [0,100], 
	with 0 meaning completely free and 100 meaning completely occupied, 
	and the special value -1 for completely unknown.
	int8[] data
	**/
	// Map
	nav_msgs::OccupancyGrid map_;
	
	// Robot pose on the global frame referential
	geometry_msgs::PoseStamped robot_pose_;
	
	// Node handler
	ros::NodeHandle n_;
	
	/**
	The Map class stores an occupancy grid as a two dimensional
    numpy array.Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --  In the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    Note that x increases with increasing column number and y increases
    with increasing row number. 
	**/

	// Map subscriber.
	ros::Subscriber map_sub_;
	// Obstacles sunscriber
	ros::Subscriber obstacles_sub_;
	// Inflated obstacles subscriber
	ros::Subscriber inflated_sub_;
	// Visited cells topic subscriber
	//ros::Subscriber cells_sub_;
	// Global costmap subscriber 
	ros::Subscriber global_costmap_sub_;
	
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
	
	// Move Base Action Server.
	// Tell the action client that we want to spin a thread by default
	MoveBaseClient ac_;
	
	// Inflation radius from the nav stack
	double inflation_radius_;
	
	ros::Time start_time_;
	
	void setGoal();
	bool checkGoal(move_base_msgs::MoveBaseGoal * goal);
	bool checkCell(int goal_cell_x, int goal_cell_y);
	//bool requestMap(ros::NodeHandle &nh);
	//void readMap(const nav_msgs::OccupancyGrid& msg);
	//void printGrid();
	
	void odomCallback(const boost::shared_ptr<const nav_msgs::Odometry>& msg);
	void obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg);
	void inflatedCallback(const nav_msgs::GridCells::ConstPtr& msg);
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	/*
	void globalCostMapCheck(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void cellsCallback(const nav_msgs::GridCells::ConstPtr& msg);
	
	void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
	void goalActiveCallback();
	void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
	*/
};


RandomWalk::RandomWalk() : ac_("move_base", true){
	bool ac_online = false;
	// wait for the action server to come up 
	while(!ac_.waitForServer(ros::Duration(3.0))){
		ROS_INFO("Waiting for the move_base action server to come up!");
		ac_online = true;
	}
	if(!ac_online){
		ROS_FATAL("RandomWalk -- I think you forgot to launch move base xD");
		ROS_BREAK();
		return;
	}
	ROS_INFO("RandomWalk -- Found the GOD DAMN move base!");
	
	// Subscribe to topics
	odom_sub_.subscribe(n_, "andbot/odom_diffdrive", 10);
	tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tf_, "map", 10);
    tf_filter_->registerCallback( boost::bind(&RandomWalk::odomCallback, this, _1) );
    
    map_sub_ = n_.subscribe("move_base/global_costmap/costmap", 1, &RandomWalk::mapCallback, this);
	/**
		Obstacles
		Topic: local_costmap/obstacles
		Type: nav_msgs/GridCells
		Desctiption: Displays the obstacles that the navigation stack sees in its costmap. 
		For the robot to avoid collision, the robot footprint should never intersect with a cell 
		that contains an obstacle.

		Inflated Obstacles
		Topic: local_costmap/inflated_obstacles
		Type: nav_msgs/GridCells
		Description: Displays obstacles in the navigation stack's costmap inflated by the inscribed radius 
		of the robot. For the robot to avoid collision, the center point of the robot should never overlap 
		with a cell that contains an inflated obstacle.

		/move_base/local_costmap/costmap
		......
	**/
	//global_costmap_sub_ = n_.subscribe("move_base/global_costmap/costmap", 1, &RandomWalk::globalCostMapCheck, this);
	
	obstacles_sub_ = n_.subscribe("move_base/local_costmap/obstacles", 1, &RandomWalk::obstaclesCallback, this);
	//obstacles_sub_ = n_.subscribe("move_base/local_costmap/costmap", 1, &RandomWalk::obstaclesCallback, this);
	//obstacles_sub_ = n_.subscribe("move_base/global_costmap/costmap", 1, &RandomWalk::obstaclesCallback, this);
	inflated_sub_ = n_.subscribe("move_base/local_costmap/inflated_obstacles", 1, &RandomWalk::inflatedCallback, this);
	//cells_sub_ = n_.subscribe("/visited_cells", 1, &RandomWalk::cellsCallback, this);
	
	n_.param("move_base/local_costmap/inflation_radius", inflation_radius_, 0.20);
	
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

RandomWalk::~RandomWalk(){
	ros::Duration time_spent = ros::Time::now() - start_time_;
	ROS_INFO("RandomWalk -- Took %lf seconds to complete the exploration! Got %lf%% of explored area!", time_spent.toSec(), (cells_.cell_height*cells_.cell_width*cells_.cells.size())/(map_.info.resolution*map_.info.resolution*map_known_cells_));
}


void RandomWalk::setGoal(){
	move_base_msgs::MoveBaseGoal goal;
	int x, y;

	do{
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
	
		/*double yaw = (rand() % 628 - 314)/100.0;
		double step = (rand() % 75 + 25)/100.0;

		goal.target_pose.pose.position.x = robot_pose_.pose.position.x + step * cos(tf::getYaw(robot_pose_.pose.orientation) + yaw);
		goal.target_pose.pose.position.y = robot_pose_.pose.position.y + step * sin(tf::getYaw(robot_pose_.pose.orientation) + yaw);
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(tf::getYaw(robot_pose_.pose.orientation) + yaw));*/
		
		/** 
		The map resolution [m/cell]
		OccupancyGrid has resolution in meters/cell(pixels)
		So pose.x = cell_x * resolution 
		**/
		x = rand() % map_.info.width;// Generate values in range [0,735]
		y = rand() % map_.info.height;// Generate values in range [0,671]
		
		goal.target_pose.pose.position.x = x*map_.info.resolution;
		goal.target_pose.pose.position.y = y*map_.info.resolution;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		
		ROS_INFO("RandomWalk - %s - Testing %lf %lf...", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
		
		if(!n_.ok()){
			ROS_INFO("RandomWalk - %s - Exiting...", __FUNCTION__);
			return;
		}
	}
	while(!checkCell(x, y) || (goal.target_pose.pose.position.x <= min_x) || (goal.target_pose.pose.position.x >= max_x) || (goal.target_pose.pose.position.y <= min_y) || (goal.target_pose.pose.position.y >= max_y) );
	//while(!checkCell(x, y));
	//while(!checkGoal(&goal));

	ROS_INFO("RandomWalk - %s - Sending robot to %lf %lf", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

	// Sending the goal 
	//ac_.sendGoal(goal, boost::bind(&RandomWalk::goalDoneCallback, this,  _1, _2), boost::bind(&RandomWalk::goalActiveCallback, this), boost::bind(&RandomWalk::goalFeedbackCallback, this, _1));
	//ac_.sendGoal(goal);
}

/**
About obstacles:
http://answers.ros.org/question/196976/navigation-stack-not-publishing-obstacles-inflation-obstacles-details/
http://answers.ros.org/question/205521/robot-coordinates-in-map/
**/

// Checking the goal to see if its inside the map(optional, its already done in setGoal method)
bool RandomWalk::checkGoal(move_base_msgs::MoveBaseGoal * goal){ 
	int goal_cell_x = goal->target_pose.pose.position.x / map_.info.resolution;
	int goal_cell_y = goal->target_pose.pose.position.y / map_.info.resolution;
	
	// 1. If goal is out of bounds, discard it!
	if(goal_cell_x > map_.info.width-1 || goal_cell_x < 0 || goal_cell_y > map_.info.height-1 || goal_cell_y < 0){
		//ROS_INFO("RandomWalk - %s -FAILED Target Goal Positions Target Goal_x:%lf gTarget Goal_y:%lf",__FUNCTION__, goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
		//ROS_INFO("RandomWalk - %s -FAILED Goal Cell Positions are out of bounds! goal_cell_x:%d goal_cell_y:%d",__FUNCTION__, goal_cell_x, goal_cell_y);
		return false;
	}
	/*
	if(!checkCell(goal_cell_x, goal_cell_y)) 
		return false;
	*/

	ROS_INFO("RandomWalk - %s - Target Goal Positions Target Goal_x:%lf gTarget Goal_y:%lf", __FUNCTION__, goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
	ROS_INFO("RandomWalk - %s - Goal Cell Positions goal_cell_x:%d goal_cell_y:%d", __FUNCTION__, goal_cell_x, goal_cell_y);

	return true;
}

// Checking the cell to see if its a free cell inside the global/local costmap(required)
bool RandomWalk::checkCell(int goal_cell_x, int goal_cell_y){
	/**
	When comparing to the threshold parameters, the occupancy probability of an image pixel is computed 
	as follows: occ = (255 - color_avg) / 255.0, 
	where color_avg is the 8-bit value that results from averaging over all channels
	**/
	std::vector<geometry_msgs::Point>::iterator obstacle;
	// Use vector instead of array
	//global_costmap_data = map_.data;
	//ROS_INFO("SIZE OF GLOBAL COSTMAP DATA: %d", sizeof(map_.data));
	//ROS_INFO("GLOBAL COSTMAP DATA: %d", global_costmap_data);

	ROS_INFO("RandomWalk - %s - MapData[]: %d ", __FUNCTION__, map_.data[goal_cell_y*map_.info.width+goal_cell_x]);
	
	// If goal is an obstacle or too close to one on the map, discard it!
	if(map_.data[goal_cell_y*map_.info.width+goal_cell_x] == 0) 
		return true;

	else 
		return false;
	/*
	// Round up the value				
	int cir = ceil(inflation_radius_/map_.info.resolution)+1;
	for(int i=(goal_cell_x-cir<0 ? 0 : goal_cell_x-cir) ; i<(goal_cell_x+cir>map_.info.width ? map_.info.width : goal_cell_x+cir) ; i++)
	{
		for(int j=(goal_cell_y-cir<0 ? 0 : goal_cell_y-cir) ; j<(goal_cell_y+cir>map_.info.height ? map_.info.height : goal_cell_y+cir) ; j++)
		{
			if(map_.data[j*map_.info.width+i] != 0 && sqrt((goal_cell_x-i)*(goal_cell_x-i)+(goal_cell_y-j)*(goal_cell_y-j)) <= (float)cir-1.0)
			{
				return false;
			}
		}
	}
	
	// 2. If goal is too close to an obstacle, discard it!
	for(obstacle = obstacles_.cells.begin() ; obstacle != obstacles_.cells.end() ; obstacle++)
	{
		if(goal_cell_x >= obstacle->x && goal_cell_x <= obstacle->x+obstacles_.cell_width && goal_cell_y >= obstacle->y && goal_cell_y <= obstacle->y+obstacles_.cell_height)
			return false;
	}
	
	// 3. If goal is too close to an inflated obstacle, discard it!
	for(obstacle = inflated_.cells.begin() ; obstacle != inflated_.cells.end() ; obstacle++)
	{
		if(goal_cell_x >= obstacle->x && goal_cell_x <= obstacle->x+obstacles_.cell_width && goal_cell_y >= obstacle->y && goal_cell_y <= obstacle->y+obstacles_.cell_height)
			return false;
	}
	*/
}

void RandomWalk::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	ROS_INFO("RandomWalk - %s - Got the map!", __FUNCTION__);
	map_ = *msg;
	//ROS_INFO("RandomWalk - %s - map_known_cells_:%d!", __FUNCTION__, map_known_cells_ );
	vector <vector<bool> > grid;
	if(map_known_cells_ == 0){
		// width = 736
		for(int i=0 ; i<map_.info.width ; i++){
			// height = 672
			for(int j=0 ; j<map_.info.height ; j++){
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

	// Dynamically resize the grid
	grid.resize(map_.info.height);
	for (int i = 0; i < map_.info.height; i++) {
		grid[i].resize(map_.info.width);
	}
	int currCell = 0;
	for (int i = 0; i < map_.info.height; i++) {
		for (int j = 0; j < map_.info.width; j++){
			if (map_.data[currCell] == 0) // unoccupied cell
				grid[i][j] = false;
			else
				grid[i][j] = true; // occupied (100) or unknown cell (-1)
			currCell++;
		}
	}

	ROS_INFO("Grid map:\n");
	int freeCells = 0;
	for (int i = 0; i < map_.info.height; i++){
		ROS_INFO("Row no. %d\n", i);
		for (int j = 0; j < map_.info.width; j++){
			ROS_INFO("%d ", grid[i][j] ? 1 : 0);
		}
	ROS_INFO("\n");
	}

	//ROS_INFO("RandomWalk - %s - map_known_cells_:%d!", __FUNCTION__, map_known_cells_);
	//ROS_INFO("RandomWalk - %s - map_unknown_cells_:%d!", __FUNCTION__, map_unknown_cells_);
	//ROS_INFO("RandomWalk - %s - map_occupied_cells_:%d!", __FUNCTION__, map_occupied_cells_);
	//ROS_INFO("RandomWalk - %s - map_free_cells_:%d!", __FUNCTION__, map_free_cells_);
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
	
	try {
		// Transform the odom into a pose in the map frame
		tf_.transformPose("map", odom, robot_pose_);
	}
	catch(tf::TransformException &ex) {
		ROS_ERROR("RandomWalk - %s - Error: %s", __FUNCTION__, ex.what());
		return;
	}
}
/*
// Read the map data
void RandomWalk::readMap(const nav_msgs::OccupancyGrid& map){
	ROS_INFO("Received a %d X %d map @ %.3f m/px\n",map.info.width,map.info.height,map.info.resolution);
	rows = map.info.height;
	cols = map.info.width;
	mapResolution = map.info.resolution;
	// Dynamically resize the grid
	grid.resize(rows);
	for (int i = 0; i < rows; i++) {
		grid[i].resize(cols);
	}
	int currCell = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++){
			if (map.data[currCell] == 0) // unoccupied cell
				grid[i][j] = false;
			else
				grid[i][j] = true; // occupied (100) or unknown cell (-1)
			currCell++;
		}
	}
}

//Print the grid 
void RandomWalk::printGrid(){
	printf("Grid map:\n");
	int freeCells = 0;
	for (int i = 0; i < rows; i++){
		printf("Row no. %d\n", i);
		for (int j = 0; j < cols; j++){
			printf("%d ", grid[i][j] ? 1 : 0);
		}
	printf("\n");
	}
}
*/
void RandomWalk::obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg){
	ROS_DEBUG("RandomWalk - %s - Got an obstacles msg!", __FUNCTION__);
	obstacles_ = *msg;
}

void RandomWalk::inflatedCallback(const nav_msgs::GridCells::ConstPtr& msg){
	ROS_DEBUG("RandomWalk - %s - Got an inflated obstacles msg!", __FUNCTION__);
	inflated_ = *msg;
}
/*
void RandomWalk::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){
	ROS_INFO("RandomWalk - %s - Already visited %lf%% of the map.", __FUNCTION__, (cells_.cell_height*cells_.cell_width*cells_.cells.size())/(map_.info.resolution*map_.info.resolution*map_known_cells_));

	//if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) ;	
	//if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) ;
	setGoal();
}

void RandomWalk::globalCostMapCheck(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	ROS_INFO("RandomWalk - %s - OccupancyGrid Callback", __FUNCTION__);
	
}


void RandomWalk::cellsCallback(const nav_msgs::GridCells::ConstPtr& msg){
	ROS_DEBUG("RandomWalk - %s - Got a visited cells msg!", __FUNCTION__);
	cells_ = *msg;
}

void RandomWalk::goalActiveCallback(){
	
}

void RandomWalk::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){

}
*/
int main(int argc, char **argv){
  	ros::init(argc, argv, "random_walk_node");
  	
  	ROS_INFO("RandomWalk for Adamgo/Andbot!!!");
	// Make sure that we get a different random number when we run this program
	srand(time(NULL));

	RandomWalk rw;
	ros::spin();
	
 	return(0);
}

