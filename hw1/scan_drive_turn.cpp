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
#include <angles/angles.h>
#include <math.h>

//Have to use 12 global variables here for the current robots' position.
//------------robot 0-----------------
double laser_dist_zero = 0;
double x_curr_zero = 0;
double y_curr_zero = 0;
double theta_curr_zero = 0;
//------------robot 1-----------------
double laser_dist_one = 0;
double x_curr_one= 0;
double y_curr_one = 0;
double theta_curr_one = 0;
//------------robot 2-----------------
double laser_dist_two = 0;
double x_curr_two = 0;
double y_curr_two = 0;
double theta_curr_two = 0;



//subscriber callback function for the robots' laser scans
//--------------------------------------------robot 0-------------------------------------------------------------	
void laserRecievedZero(const sensor_msgs::LaserScan::ConstPtr& msg) {
	//this will get the range at the center (directly in fron of) the robot. 540 would be the 0 angle of the scan.
	laser_dist_zero = msg->ranges[540];
}
//--------------------------------------------robot 1-------------------------------------------------------------	
void laserRecievedOne(const sensor_msgs::LaserScan::ConstPtr& msg) {
	//this will get the range at the center (directly in fron of) the robot. 540 would be the 0 angle of the scan.
	laser_dist_one = msg->ranges[540];
}
//--------------------------------------------robot 2-------------------------------------------------------------	
void laserRecievedTwo(const sensor_msgs::LaserScan::ConstPtr& msg) {
	//this will get the range at the center (directly in fron of) the robot. 540 would be the 0 angle of the scan.
	laser_dist_two = msg->ranges[540];
}



//callback function for recieving odometry messages from the robots
//----------------------------------------------------------robot 0---------------------------------------------------------------------
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
//----------------------------------------------------------robot 1---------------------------------------------------------------------
void posRecievedOne(const nav_msgs::Odometry::ConstPtr& msg) {
	//getting the x and y positions
    x_curr_one = msg->pose.pose.position.x;
    y_curr_one = msg->pose.pose.position.y;
    
    // quaternion to RPY conversion
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, 
    	msg->pose.pose.orientation.w);
    
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // angular position
    theta_curr_one = yaw;
}
//----------------------------------------------------------robot 2---------------------------------------------------------------------
void posRecievedTwo(const nav_msgs::Odometry::ConstPtr& msg) {
	//getting the x and y positions
    x_curr_two = msg->pose.pose.position.x;
    y_curr_two = msg->pose.pose.position.y;
    
    // quaternion to RPY conversion
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, 
    	msg->pose.pose.orientation.w);
    
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // angular position
    theta_curr_two = yaw;
}



//Functions for driving the robots forward up to half the distance between the robot and the wall in front of it 
//--------------------------------------------robot 0-------------------------------------------------------------	 	
void driveZero(ros::Publisher pub_mv, double move_distance, double linearSpeed, int direction) {
    // Current position
	double x = x_curr_zero;
    double y = y_curr_zero;

    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    // Set the movement command linear speed for forward motion
    msg_mv.linear.x = direction * linearSpeed;

    // How fast to update the robot's movement?
    // Set the equivalent ROS rate variable
    ros::Rate rate(10.0);

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
//--------------------------------------------robot 1-------------------------------------------------------------
void driveOne(ros::Publisher pub_mv, double move_distance, double linearSpeed, int direction) {
    // Current position
	double x = x_curr_one;
    double y = y_curr_one;

    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    // Set the movement command linear speed for forward motion
    msg_mv.linear.x = direction * linearSpeed;

    // How fast to update the robot's movement?
    // Set the equivalent ROS rate variable
    ros::Rate rate(10.0);

    double d = 0;
    // Enter the loop to move the robot
    while (d < move_distance && ros::ok()) {
        //Publish the Twist message and sleep 1 cycle
        pub_mv.publish(msg_mv);

        ros::spinOnce();
        rate.sleep();

        // Compute the Euclidean distance from the start position
		 d = sqrt(pow((x_curr_one - x), 2) + pow((y_curr_one - y), 2));		
		
        ROS_INFO("d: %f", d);
    }

    // Stop the robot
    msg_mv.linear.x = 0;
    pub_mv.publish(msg_mv);
    
    return;

}
//--------------------------------------------robot 2-------------------------------------------------------------
void driveTwo(ros::Publisher pub_mv, double move_distance, double linearSpeed, int direction) {
    // Current position
	double x = x_curr_two;
    double y = y_curr_two;

    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    // Set the movement command linear speed for forward motion
    msg_mv.linear.x = direction * linearSpeed;

    // How fast to update the robot's movement?
    // Set the equivalent ROS rate variable
    ros::Rate rate(10.0);

    double d = 0;
    // Enter the loop to move the robot
    while (d < move_distance && ros::ok()) {
        //Publish the Twist message and sleep for a cycle
        pub_mv.publish(msg_mv);

        ros::spinOnce();
        rate.sleep();

        // Compute the Euclidean distance from the start position
		 d = sqrt(pow((x_curr_two - x), 2) + pow((y_curr_two - y), 2));		
		
        ROS_INFO("d: %f", d);
    }

    // Stop the robot
    msg_mv.linear.x = 0;
    pub_mv.publish(msg_mv);
    
    return;

}



//Functions for turning the robots 90 degrees to the right
//--------------------------------------------robot 0-------------------------------------------------------------
void turnZero(ros::Publisher pub_mv, double turn_angle, double angularSpeed, int direction) {
    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    //Set the movement command rotation speed
    msg_mv.angular.z = direction * angularSpeed;

    // How fast to update the robot's movement.
    ros::Rate rate(10.0);

    // Current angle
    double last_angle = theta_curr_zero;
    double angle = 0;

	//I'm using greater than since it is turning right
     while ((angle > turn_angle) && ros::ok()) {
        //Publish the Twist message and sleep for a cycle
        pub_mv.publish(msg_mv);

        ros::spinOnce();
        rate.sleep();

        // Compute the amount of rotation since the last loop
        angle += angles::normalize_angle(theta_curr_zero - last_angle);
        last_angle = theta_curr_zero;

        ROS_INFO("angle: %f", angle);
    }

    //Robot stops turning
    msg_mv.angular.z = 0;
    pub_mv.publish(msg_mv);
    
    return;
}
//--------------------------------------------robot 1------------------------------------------------------------
void turnOne(ros::Publisher pub_mv, double turn_angle, double angularSpeed, int direction) {
    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    //Set the movement command rotation speed
    msg_mv.angular.z = direction * angularSpeed;

    // How fast to update the robot's movement.
    ros::Rate rate(10.0);

    // Current angle
    double last_angle = theta_curr_one;
    double angle = 0;

	//I'm using greater than since it is turning right
     while ((angle > turn_angle) && ros::ok()) {
        //Publish the Twist message and sleep for a cycle
        pub_mv.publish(msg_mv);

        ros::spinOnce();
        rate.sleep();

        // Compute the amount of rotation since the last loop
        angle += angles::normalize_angle(theta_curr_one - last_angle);
        last_angle = theta_curr_one;

        ROS_INFO("angle: %f", angle);
    }

    //Robot stops turning
    msg_mv.angular.z = 0;
    pub_mv.publish(msg_mv);
    
    return;
}
//--------------------------------------------robot 2------------------------------------------------------------
void turnTwo(ros::Publisher pub_mv, double turn_angle, double angularSpeed, int direction) {
    // Initialize the movement command message
    geometry_msgs::Twist msg_mv;
    //Set the movement command rotation speed
    msg_mv.angular.z = direction * angularSpeed;

    // How fast to update the robot's movement.
    ros::Rate rate(10.0);

    // Current angle
    double last_angle = theta_curr_two;
    double angle = 0;

	//I'm using greater than since it is turning right
     while ((angle > turn_angle) && ros::ok()) {
        //Publish the Twist message and sleep 1 cycle
        pub_mv.publish(msg_mv);

        ros::spinOnce();
        rate.sleep();

        // Compute the amount of rotation since the last loop
        angle += angles::normalize_angle(theta_curr_two - last_angle);
        last_angle = theta_curr_two;

        ROS_INFO("angle: %f", angle);
    }

    //Robot stops turning
    msg_mv.angular.z = 0;
    pub_mv.publish(msg_mv);
    
    return;
}  	
   	
   	
   	
//These are the initial laser scan functions that use the watForMessage function to just get a single laser scan message and the unsubscribe.
//----------------------------------------------------------robot 0---------------------------------------------------------------------
float laserDistanceZero(){
	//I only wanted to check the laser scan once, for each time the robot was stopped and not turning/moving. 
	sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot_0/base_scan");
		
	//this will get the range at the center (directly in fron of) the robot. 540 would be the 0 angle of the scan.
	float wall_range = msg->ranges[540];
	float travel_dist = wall_range;
	return travel_dist;
}
//----------------------------------------------------------robot 1---------------------------------------------------------------------
float laserDistanceOne(){
	//I only wanted to check the laser scan once, for each time the robot was stopped and not turning/moving. 
	sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot_1/base_scan");
		
	//this will get the range at the center (directly in fron of) the robot. 540 would be the 0 angle of the scan.
	float wall_range = msg->ranges[540];
	float travel_dist = wall_range;
	return travel_dist;
}
//----------------------------------------------------------robot 2---------------------------------------------------------------------
float laserDistanceTwo(){
	//I only wanted to check the laser scan once, for each time the robot was stopped and not turning/moving. 
	sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("robot_2/base_scan");
		
	//this will get the range at the center (directly in fron of) the robot. 540 would be the 0 angle of the scan.
	float wall_range = msg->ranges[539];
	float travel_dist = wall_range;
	return travel_dist;
}




int main(int argc, char **argv) {
	//Initialize the ROS system
	ros::init(argc, argv, "Scanning_Driving_Turning");
	//Setting the nodehandle ofr the node	
	ros::NodeHandle nh;
	
	//----------ROBOT 0-------------------------------------------------------------------------
	//publisher for publishing the robots' movement
	ros::Publisher pub_mv = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 1000);	
	//subscriber for subscribing to the robots' odometry readings
	ros::Subscriber sub_odom = nh.subscribe("robot_0/odom", 1000, &posRecievedZero);
	//subscriber for subscribing to the robots' laser sensor readings.
	ros::Subscriber sub_lsr = nh.subscribe("robot_0/base_scan", 1000, &laserRecievedZero);
	
	//----------ROBOT 1-------------------------------------------------------------------------
	//publisher for publishing the robots' movement
	ros::Publisher pub_mv_1 = nh.advertise<geometry_msgs::Twist>("robot_1/cmd_vel", 1000);	
	//subscriber for subscribing to the robots' odometry readings
	ros::Subscriber sub_odom_1 = nh.subscribe("robot_1/odom", 1000, &posRecievedOne);
	//subscriber for subscribing to the robots' laser sensor readings.
	ros::Subscriber sub_lsr_1 = nh.subscribe("robot_1/base_scan", 1000, &laserRecievedOne);
	
	//----------ROBOT 2-------------------------------------------------------------------------
	//publisher for publishing the robots' movement
	ros::Publisher pub_mv_2 = nh.advertise<geometry_msgs::Twist>("robot_2/cmd_vel", 1000);	
	//subscriber for subscribing to the robots' odometry readings
	ros::Subscriber sub_odom_2 = nh.subscribe("robot_2/odom", 1000, &posRecievedTwo);
	//subscriber for subscribing to the robots' laser sensor readings.
	ros::Subscriber sub_lsr_2 = nh.subscribe("robot_2/base_scan", 1000, &laserRecievedTwo);
	
	
	//This is the shared driving characteristics of the robots
	//-----------------------------------------------------------------------
	//creating PI variable.
	const double PI = 3.14159265358979323846;
    // Set the forward linear speed (in meters/sec).)
    double linear_speed = 0.02;
    // Set the rotation speed in radians/sec.
    double angular_speed = 0.012;
    // Set the rotation angle to 90 degrees (to the right) in radians.
    double turn_angle = -1.5708;
	//------------------------------------------------------------------------	
	
	//Initial laser scan readings
		laser_dist_zero = laserDistanceZero();
		laser_dist_one = laserDistanceOne();
		laser_dist_two = laserDistanceTwo();
	
	//loop for robot 0
	int edge_count = 0;
	while(ros::ok() && edge_count < 4) {		
		// Set the travel distance in meters
    	double travel_distance_0 = laser_dist_zero/2;
   		double travel_distance_1 = laser_dist_one/2;
    	double travel_distance_2 = laser_dist_two/2;
    	
    	
		driveZero(pub_mv, travel_distance_0, linear_speed, 1);
  		turnZero(pub_mv, turn_angle, angular_speed, -1);
  		
  		driveOne(pub_mv_1, travel_distance_1, linear_speed, 1);
  		turnOne(pub_mv_1, turn_angle, angular_speed, -1);
		
 		driveTwo(pub_mv_2, travel_distance_2, linear_speed, 1);
		turnTwo(pub_mv_2, turn_angle, angular_speed, -1);
		
		edge_count++;
	    ros::spinOnce();
    }
    
	//I shutdown the node when it's done.
	if(ros::ok()){
		ros::shutdown();
	}
	return 0; 
}



