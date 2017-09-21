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
#include  <tf/tf.h>
#include <iomanip> 	//for stf::setprecision and std::fixed
#include <math.h>



void posRecieved(const nav_msgs::Odometry::ConstPtr& msg) {
		//current
}
	
void driveForward(float travel_dist, ros::Publisher pub_mv, ros::Time start){
	ROS_INFO_STREAM("Travel Distance: " << travel_dist);
	
	//find the number of seconds to reach destination distance. Then I add the starting time plus drive time to get finishing time.
	double drv_time = travel_dist/0.2; 
	double fin_time = drv_time + start.toSec();
	
	ros::Rate rate(10);
	ros::Time curr_time;
	while(ros::ok() && curr_time.toSec() <= fin_time) {
		geometry_msgs::Twist vel_msg;
		//velocity controls
		vel_msg.linear.x = 0.2; //speed value m/s
		vel_msg.angular.z = 0;
		pub_mv.publish(vel_msg);
	
		curr_time = ros::Time::now();
	    ros::spinOnce();
	    rate.sleep();
    }
}

/**	
void turn90Deg(const double PI, float curr_theta){
	ros::Rate rate(10);
	while(ros::ok() && curr_theta > -PI/4) {
		geometry_msgs::Twist vel_msg;
		//velocity controls
		vel_msg.linear.x = 0; //speed value m/s
		vel_msg.angular.z = -0.3;
		pub_mv.publish(vel_msg);
		
	    ros::spinOnce();
	    rate.sleep();
    }
}
**/
void stopVel(ros::Publisher pub_mv){
	ros::Rate rate(10);
	while(ros::ok()) {
		geometry_msgs::Twist vel_msg;
		//velocity controls
		vel_msg.linear.x = 0; //speed value m/s
		vel_msg.angular.z = 0;
		pub_mv.publish(vel_msg);
		
		ros::spinOnce();
		rate.sleep();
   	}
}

	float LaserDistance(const double PI){
		//I only wanted to check the laser scan once, for each time the robot was stopped and not turning/moving. 
		sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot_0/base_scan");
		
		//this loop is for finding the robots laser sensor's farthest range since that will be the wall directly in fron of it.
		float max_range = msg->range_max;
		//for(int i = 0; i < msg->ranges.size(); i++){
		//	if(msg->ranges[i] > max_range){
		//		max_range = msg->ranges[i];
		//	}
		//}
		float travel_dist = max_range/2;
		return travel_dist;
	}


int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "Scanning_Driving_Turning");
	//Setting the nodehandle ofr the node	
	ros::NodeHandle nh;
	//publisher for publishing the robots movement
	ros::Publisher pub_mv = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);	
	//subscriber for subscribing the robots odometry readings
	ros::Subscriber sub_odom = nh.subscribe("odom", 1000, &posRecieved);
	
	//creating degrees variable.
	const double PI = 3.14159265358979323846;
	
	float travel_dist = LaserDistance(PI);
	ros::Time start = ros::Time::now();
	driveForward(travel_dist, pub_mv, start);
	
	
/**	
	//drive forward
    ROS_INFO("move forward");
    ros::Time start = ros::Time::now();
  	SDTObject.driveForward();
    
    //move turn right 90 degrees.
    ROS_INFO("turning right 90 degrees");
    ros::Time start_turn = ros::Time::now();
    SDTObject.turn90Deg(PI, current_pose.theta);
	
	 //drive forward
    ROS_INFO("move forward");
    ros::Time start2 = ros::Time::now();
    SDTObject.driveForward(current_pose.y);
**/    		
	//this is to stop the robot completely. This will be used when it has traveled 4 edges.
	//stopVel();
		
	//Let ROS take over.
	ros::spin();
}



