/**
Simplest Random Walk
**/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

#define MIN_SCAN_ANGLE_RAD -40.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD +40.0/180*M_PI

void readSensorCallback(const sensor_msgs::LaserScan::ConstPtr &sensor_msg);

bool obstacleFound = false;

void readSensorCallback(const sensor_msgs::LaserScan::ConstPtr &scan){
	bool isObstacle = false;

	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

	for (int i = minIndex; i <= maxIndex; i++){
		if (scan->ranges[i] < 0.8) {
			isObstacle = true;
		}
	}

	if (isObstacle){
		ROS_INFO("Obstacle found! Turning around");
		obstacleFound = true;
	} else{
		obstacleFound = false;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "random_walk_node");
	// initialize random seed: 
	// Make sure that we get a different random number when we run this program
	srand(time(NULL));
	ros::NodeHandle nh;

	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 10);
	ros::Subscriber base_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/rplidar_scan", 1, &readSensorCallback);

	geometry_msgs::Twist moveForwardCommand;
	moveForwardCommand.linear.x = 0.4;
	float r = (float)rand() / (float)RAND_MAX;
	geometry_msgs::Twist turnCommand;
	//turnCommand.angular.z = 0.25;
	turnCommand.angular.z = -0.35 + r*(0.7);
	ros::Rate loop_rate(10);

	while (ros::ok()){
		if (obstacleFound){
			cmd_vel_pub.publish(turnCommand);
		} else{
			cmd_vel_pub.publish(moveForwardCommand);
		}

		ros::spinOnce(); // let ROS process incoming messages
		loop_rate.sleep();
	}

	return 0;
}
