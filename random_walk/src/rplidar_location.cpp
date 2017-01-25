/*
* rplidar_location.cpp
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "rplidar_location");
    ros::NodeHandle node;

    tf::TransformListener listener;
    ros::Rate rate(2.0);

    //listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0) );
    //listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0) );

    while (ros::ok()){
        tf::StampedTransform transform;
        try {
            //listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
            listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();

            cout << "Current position: (" << x << "," << y << "," << z <<")" << endl;
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
        rate.sleep();
    }

    return 0;
}