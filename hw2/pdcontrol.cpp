/**
 * Ian Davis
 * idavis@email.sc.edu
 * CSCE 574
 * HW2 node
**/
//This is my ros node for homework 2 (PD Controller)
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


class pdcontrol {
	
	public:	
		//:::::::::::Variables to be used with the PD controller:::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		double wall_dist;		//wanted distance from wall
		double max_vel;		//the top velocity of the robot
		double Kp; 			//proportional constant
		double Kd;			//derivative constant
		double PI;			//defined the number pi
		double side; 				//which side of the robot is following the wall
		double error; 				//difference between desired distance from the wall and the actual distance
		double deriv; 				//derivative element of the controller
		double min_dist_angle;		//the angle at which the distance from the wall was smallest
		double front_dist;			//distance from the wall using front laser data
		//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
		
    	//publisher for the robot
    	ros::Publisher pub_vel;
		
		//class constructor that initializes all of the varibales used in the class		
		pdcontrol(ros::Publisher pub) {
		
			pub_vel = pub;
			wall_dist = 0.55;;		//distance in meters
			max_vel = 0.55;			//velocity in meters/sec
			Kp = 4;
			Kd = 4;
			PI = 3.1415926535;
			side = -1; 				//in this case it's the right wall so the side is -1
			error = 0;				
			deriv = 0;				
			min_dist_angle = 0;		//this is in radians
		}
		
		//class destructor
		~pdcontrol() {
			//Generic Destructor
		}
		
		//function for publishing the robot's speeds
		void pubMessage() {
			//setting up the message
			geometry_msgs::Twist msg;
			
			//this is the PD controller (which is being used for the angular velocity/ turning the robot))
			msg.angular.z = side*(Kp*error + Kd*deriv) + (min_dist_angle + PI/2);
			
			//EDIT THIS STUFF FOR PERSONAL USE
			if (front_dist < wall_dist){
				msg.linear.x = 0;
			}else if (front_dist < wall_dist * 2){
				msg.linear.x = 0.5*max_vel;
			}else if (fabs(min_dist_angle)>1.75){
				msg.linear.x = 0.4*max_vel;
			}else {
				msg.linear.x = max_vel;
			}
			
			pub_vel.publish(msg); 
		}
		
		
		//Subscriber Callback function for reciving the robots' laser sensor data
		void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		  //the size of the robots' laser range array containg all of the ranges in a scan
		  int size = msg->ranges.size();

		  //Variables whith index of highest and lowest value in array.
		  int minIndex = size*(side+1)/4;
		  int maxIndex = size*(side+3)/4;

		  //This cycle goes through array and finds minimum
		  for(int i = minIndex; i < maxIndex; i++) {
			if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.0){
			  minIndex = i;
			}
		  }

		  //Calculation of angles from indexes and storing data to class variables.
		  min_dist_angle = (minIndex-size/2)*msg->angle_increment;
		  double dist_min;
		  dist_min = msg->ranges[minIndex];
		  front_dist = msg->ranges[size/2];
		  deriv = (dist_min - wall_dist) - error;
		  error = dist_min - wall_dist;

		  //Invoking method for publishing message
		  pubMessage();
		}
};








int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "PD_Controller");
	//Setting the nodehandle ofr the node	
	ros::NodeHandle nh;

	//Publisher for the robot velocity
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	//creating the object which contains methods for publishing and subscribing as well as the data from the sensors which it uses to 		//work the PD Controller.
	pdcontrol *controller = new pdcontrol(pub_vel); 
	
	//subscriber for the laser scan data 
	ros::Subscriber sub_lsr = nh.subscribe("base_scan", 10, &pdcontrol::laserCallback, controller);
	
	
	
	ros::spin();
	return 0;	
}















