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
#include <geometry_msgs/Pose2D.h>
#include  <tf/tf.h>
#include <angles/angles.h>
#include <math.h>

//Have to use 9 global variables here for the current robots' position.
double x_curr_zero = 0;
double y_curr_zero = 0;
double theta_curr_zero = 0;


//callback function for recieving odometry messages from robot 0
void posRecievedZero(const nav_msgs::Odometry::ConstPtr& msg) {
	//getting the x and y positions
    x_curr_zero = msg->pose.pose.position.x;
    y_curr_zero = msg->pose.pose.position.y;
    
    // quaternion to RPY conversion
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, 
    	msg->pose.pose.orientation.w);
    
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // angular position
    theta_curr_zero = yaw;
}
 	 	
void drive(ros::Publisher pub_mv, double move_distance, double linearSpeed, int direction)
{
    // Current position
	double x = x_curr_zero;
    double y = y_curr_zero;

    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    // Set the movement command linear speed for forward motion
    msg_mv.linear.x = direction * linearSpeed;

    // How fast to update the robot's movement?
    // Set the equivalent ROS rate variable
    ros::Rate rate(30.0);

    double d = 0;
    // Enter the loop to move the robot
    while (d < move_distance && ros::ok()) {
        //Publish the Twist message and sleep 1 cycle
        pub_mv.publish(msg_mv);

        ros::spinOnce();
        rate.sleep();

        // Compute the Euclidean distance from the start position
		 d = sqrt(pow((x_curr_zero - x), 2) + pow((y_curr_zero - y), 2));		
		
        ROS_INFO("d: %f", d);
    }

    // Stop the robot
    msg_mv.linear.x = 0;
    pub_mv.publish(msg_mv);
    
    return;

}

void turn(ros::Publisher pub_mv, double turn_angle, double angularSpeed, int direction) {
    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    //Set the movement command rotation speed
    msg_mv.angular.z = direction * angularSpeed;

    // How fast to update the robot's movement.
    ros::Rate rate(30.0);

    // Current angle
    double last_angle = theta_curr_zero;
    double angle = 0;

	//I'm using greater than since it is turning right
     while ((angle > turn_angle) && ros::ok()) {
        //Publish the Twist message and sleep 1 cycle
        pub_mv.publish(msg_mv);

        ros::spinOnce();
        rate.sleep();

        // Compute the amount of rotation since the last loop
        angle += angles::normalize_angle(theta_curr_zero - last_angle);
        last_angle = theta_curr_zero;

        ROS_INFO("angle: %f", angle);
    }

    // Stop turning the robot
    msg_mv.angular.z = 0;
    pub_mv.publish(msg_mv);
    
    return;
}
   	
   	
   	
   	


float laserDistanceZero(){
	//I only wanted to check the laser scan once, for each time the robot was stopped and not turning/moving. 
	sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot_0/base_scan");
		
	//this loop is for finding the robots laser sensor's farthest range since that will be the wall directly in fron of it.
	float wall_range = msg->ranges[539];
	float travel_dist = wall_range/2;
	return travel_dist;
}


int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "Scanning_Driving_Turning");
	//Setting the nodehandle ofr the node	
	ros::NodeHandle nh;
	//publisher for publishing the robots' movement
	ros::Publisher pub_mv = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);	
	//subscriber for subscribing to the robots' odometry readings
	ros::Subscriber sub_odom = nh.subscribe("robot_0/odom", 1000, &posRecievedZero);
	
	//creating PI variable.
	const double PI = 3.14159265358979323846;
	
	
    // Set the forward linear speed (in meters/sec).)
    double linear_speed = 0.02;
    
    // Set the rotation speed in radians/sec.
    double angular_speed = 0.005;

    // Set the rotation angle to 90 degrees (to the right) in radians.
    double turn_angle = -1.5708;

	//loop for robot 0
	int edge_count_zero = 0;
	while(ros::ok() && edge_count_zero < 4) {
		// Set the travel distance in meters
    	double travel_distance = laserDistanceZero();
    	
		drive(pub_mv, travel_distance, linear_speed, 1);
  		turn(pub_mv, turn_angle, angular_speed, -1);
		
		edge_count_zero++;
	    ros::spinOnce();
    }
//    //loop for robot 1
//	int edge_count_one = 0;
//	while(ros::ok() && edge_count_one < 4) {
//		// Set the travel distance in meters
//    	double travel_distance = laserDistanceOne();
//    	
//		drive(pub_mv_1, travel_distance, linear_speed, 1);
//  		turn(pub_mv_1, turn_angle, angular_speed, -1);
//		
//		edge_count_one++;
//	    ros::spinOnce();
//    }
//    //loop for robot 2
//	int edge_count_two = 0;
//	while(ros::ok() && edge_count_two < 4) {
//		// Set the travel distance in meters
//    	double travel_distance = laserDistanceTwo();
//    	
//		drive(pub_mv_2, travel_distance, linear_speed, 1);
//  		turn(pub_mv_2, turn_angle, angular_speed, -1);
//		
//		edge_count_two++;
//	    ros::spinOnce();
//    }
    

	if(ros::ok()){
		ros::shutdown();
	}
	return 0; 
}



