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
