//this is my hello world example ros node

//this head defines the standared ROS classes
#include <ros/ros.h>

int main(int argc, char **argv) {
	//Initialize the ROS system
	ros:: init(argc, argv, "hello_ros");
	
	//Establish this program as the ROS node.
	ros::NodeHandle nh;
	
	//Send some output as a log message.
	ROS_INFO_STREAM("Hello, ROS!");
}
