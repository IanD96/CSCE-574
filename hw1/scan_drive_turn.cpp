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
#include <math.h>

class ScanDriveTurn {

public:	
	ScanDriveTurn() {
		
		travel_dist = 0;
		//Created the publisher objects for all three robots
		pub_mv = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);
//		ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 1000);
//		ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("robot_2/cmd_vel", 1000);
		
		//Subscriber objects for all three robots' laser scans.
		sub_lsr = nh.subscribe("robot_0/base_scan", 1000, &ScanDriveTurn::scanMessageRecieved, this);
//		ros::Subscriber sub1 = nh.subscribe("robot_1/base_scan", 1000, &scanMessageRecieved);
//		ros::Subscriber sub2 = nh.subscribe("robot_2/base_scan", 1000, &scanMessageRecieved);

		//Subsscriber object of odometry for the robot 
		sub_tm = nh.subscribe<nav_msgs::Odometry>("robot_0/odom", 1000, &ScanDriveTurn::posRecieved, this);
	
	}
	
	//Callback function for laserscans. Executed every time a robot recieves a laser scan. 
	void scanMessageRecieved(const sensor_msgs::LaserScan& msg) {
		//getting the distance for robot travel and the time it needs to travel
		travel_dist = msg.range_max/2; 
	
	
		//sub1.shutdown();
	}
	
	void posRecieved(const nav_msgs::Odometry::ConstPtr& msg) {
		//current
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub_mv;
	ros::Subscriber sub_lsr;
	ros::Subscriber sub_tm;
	float travel_dist;
};

int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "Scanning_Driving_Turning");
		
	ScanDriveTurn SDTObject; 
	
	
	//creating degrees variable.
	const double PI = 3.14159265358979323846;
	
	//Got most of the Mobility loops done; CREATE FUNCTIONS FOR THESE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	ros::Rate rate(10);
	//drive forward
    ROS_INFO("move forward");
    ros::Time start = ros::Time::now();
    while(ros::ok() && current_pose.y > -1.5) {
        geometry_msgs::Twist vel_msg;
        //velocity controls
        vel_msg.linear.x = 0.2; //speed value m/s
        vel_msg.angular.z = 0;
        SDTObject.pub_mv.publish(vel_msg);
    
        ros::spinOnce();
        rate.sleep();
    }
    
    //move turn right 90 degrees.
    ROS_INFO("turning right 90 degrees");
    ros::Time start_turn = ros::Time::now();
    while(ros::ok() && current_pose.theta > -PI/4) {
        geometry_msgs::Twist vel_msg;
        //velocity controls
        vel_msg.linear.x = 0; //speed value m/s
        vel_msg.angular.z = -0.3;
        SDTObject.pub_mv.publish(vel_msg);
    
        ros::spinOnce();
        rate.sleep();
    }
	
	 //drive forward
    ROS_INFO("move forward");
    ros::Time start2 = ros::Time::now();
    while(ros::ok() && current_pose.y > -1.5) {
        geometry_msgs::Twist vel_msg;
        //velocity controls
        vel_msg.linear.x = 0.2; //speed value m/s
        vel_msg.angular.z = 0;
        SDTObject.pub_mv.publish(vel_msg);
    
        ros::spinOnce();
        rate.sleep();
    }
    		
	//this is to stop the robot completely. This will be used when it has traveled 4 edges.
	while(ros::ok()) {
       geometry_msgs::Twist vel_msg;
        //velocity controls
        vel_msg.linear.x = 0.2; //speed value m/s
        vel_msg.angular.z = 0;
        SDTObject.pub_mv.publish(vel_msg);
    
        ros::spinOnce();
        rate.sleep();
    }
*		
	//Let ROS take over.
	ros::spin();
}



