/**
 * Ian Davis
 * idavis@email.sc.edu
 * CSCE 574
 * HW2 node
**/
//This is my ros node for homework 2 (PD Controller)
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include  <tf/tf.h>
#include <angles/angles.h>
#include <math.h>

//Some parameters for the PD controller that need to be defined
#define WALL_DIST = 0.5 //The wanted distance between the robot and the wall
#define MAX_VEL = 0.02 //The top velocity of the robot
#define Kp = 8; //proportinal constant obtained from trial and error
#define Kd = 4; //derivative constant obtained from trial and error


class pdcontrol {
	
	public:	
		//Variables to be used with the PD controller:::::::::::::::::::::::::
		double wall_dist;		//wanted distance from wall
		double error; 			//difference between desired distance from wall and the actual distance
		double deriv; 			//derivative element of the controller
		double Kp; 				//proportional constant
		double Kd;				//derivative constant
		double max_vel;			//the top velocity of the robot
		double min_dist_angle	//the angle at which the distance from the wall was smallest
		double front_dist		//distance from the wall using front laser data
		//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		// Set the forward linear speed (in meters/sec).)
		double l_speed = 0.02;
		// Set the rotation speed in radians/sec.
    	double angular_speed = 0.012;
    	//publisher for the robot
    	ros::Publisher pub_vel;
		
	
		pdcontrol::pdcontrol(ros::Publisher pub, double wall_dist, double max_vel, double Kp, double Kd) {
			//Do Something
		}
	
};








int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "P(I)D_Controller");
	//Setting the nodehandle ofr the node	
	ros::NodeHandle nh;
	
	//Publisher for the robot velocity
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	//creating the object which contains methods for publishing and subscribing as well as the data 	//from the sensors which it uses to work the PD Controller.
	pdcontrol *pdcontrol = new pdcontrol; 
	
	ros::Subscriber sub_lsr = nh.subscribe("base_scan", 10, &pdcontrol::laserCallback, pdcontrol);
	
	
	
	ros::spin();	
}















