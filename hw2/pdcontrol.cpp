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
		double set_point;		//wanted distance from wall
		double max_vel;		//the top velocity of the robot
		double Kp; 			//proportional constant
		double Kd;			//derivative constant
		double PI;			//defined the number pi
		double side; 				//which side of the robot is following the wall
		double error; 				//difference between desired distance from the wall and the actual distance
		double deriv; 				//derivative element of the controller
		double front_dist;			//distance from the wall using front laser data
		double fr_dist;				//front-right distance from wall
		//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    	//publisher for the robot
    	ros::Publisher pub_vel;
		
		//class constructor that initializes all of the varibales used in the class		
		pdcontrol(ros::Publisher pub) {
		
			pub_vel = pub;
			set_point = 0.6;		//distance in meters
			max_vel = 0.4;			//velocity in meters/sec
			Kp = 0.7; 	
			Kd = 0.5;	
			PI = 3.1415926535;
			side = -1; 				//in this case it's the right wall so the side is -1
			error = 0;				
			deriv = 0;				
			front_dist = 0;
			fr_dist = 0;
		}
		
		
		//class destructor
		~pdcontrol() {
			//Generic Destructor
		}
		
		
		//Subscriber Callback function for reciving the robots' laser sensor data
		void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
			//the size of the robots' laser range array containg all of the ranges in a scan
			int size = msg->ranges.size();

			//Variables whith indexs of the rightmost laser incrament and the middle incrament(in front of robot) we only need this 				//sensor range since were following the right wall
			int index_min = 0;			//beginning angle from the right
			int index_max = size/2;		//angle directly in front of robot

			//This array cycles through the ranges and finds the index of the smallest distance
			for(int i = index_min; i < index_max; i++) {
				if ((msg->ranges[i] > 0.0) && (msg->ranges[i] < msg->ranges[index_min])){
				  index_min = i;
				}
			}

			//obtaining the angles and calculating the errors.
			double dist_min = msg->ranges[index_min];
			front_dist = msg->ranges[size/2];
			fr_dist = msg->ranges[(size*1)/4];
			deriv = (dist_min - set_point) - error;
			error = (dist_min - set_point);
		
			
		  	//setting up the message
			geometry_msgs::Twist vel_msg;
		
			//this is the PD controller (which is being used for the angular velocity/turning the robot))
			vel_msg.angular.z = side*(Kp*error + Kd*deriv);
			
			
			
			//satements that prevent the robot from crashing into the wall 
			if ((front_dist <= set_point) || (fr_dist <= set_point*0.8)){
				vel_msg.linear.x = 0;
			}else if ((front_dist < set_point * 2) || (fr_dist <= (set_point*2))){
				vel_msg.linear.x = 0.5*max_vel;
			}else {
				vel_msg.linear.x = max_vel;
			}
			
			//publishing the new robot velocities
			pub_vel.publish(vel_msg); 
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















