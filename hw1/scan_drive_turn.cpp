/**
 * Ian Davis
 * idavis@email.sc.edu
 * CSCE 574
 * HW1 node
**/
//This is my ros node for homework 1
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iomanip> 	//for stf::setprecision and std::fixed

class ScanDriveTurn {

public:	
	ScanDriveTurn() {
		
		
		//Created the publisher objects for all three robots
		pub = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);
//		ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 1000);
//		ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("robot_2/cmd_vel", 1000);
		
		//Subscriber objects for all three robots' laser scans.
		sub_lsr = nh.subscribe("robot_0/base_scan", 1000, &ScanDriveTurn::scanMessageRecieved, this);
//		ros::Subscriber sub1 = nh.subscribe("robot_1/base_scan", 1000, &scanMessageRecieved);
//		ros::Subscriber sub2 = nh.subscribe("robot_2/base_scan", 1000, &scanMessageRecieved);

		//Subsscriber object of odometry for the robot 
		sub_tm = nh.subscribe<nav_msgs::Odometry>("robot_0/odom", 1000, &ScandDriveTurn::posRecieved, this);
	
	}
	
	//Callback function for laserscans. Executed every time a robot recieves a laser scan. 
	void scanMessageRecieved(const sensor_msgs::LaserScan& msg) {
		//getting the distance for robot travel and the time it needs to travel
		travel_dist = msg.range_max/2; 
	
	
		//creating the publisher msg for speed
		geometry_msgs::Twist vel_msg;
	
		ros::Rate rate(1);
		while(ros::ok()) {
	
			//moving the robot as a speed of 2 
			vel_msg.linear.x = 1;
	
			pub.publish(vel_msg);
		
			rate.sleep();
		}
			
		//sub1.shutdown();
	}
	
	void posRecieved(const nav_msgs::Odometry& msg) {
		
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub_lsr;
	ros::Subscriber sub_tm;
	float travel_dist; 
};

int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "Scanning_Driving_Turning");
		
	ScanDriveTurn SDTObject; 
	
/*	while(ros::ok()) {
        geometry_msgs::Twist move;
        move.linear.x = 0;
        move.angular.z = 0;
        movement_pub.publish(move);		//Stuff I needed to stop the robot
    
        ros::spinOnce();
        rate.sleep();
    }
**/		
	//Let ROS take over.
	ros::spin();
}



