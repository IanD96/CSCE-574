#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <time.h>


using namespace boost::posix_time;


class GridMapper {
public:
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh, int width, int height) :
    canvas(height, width, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("/scan", 1, \
      &GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("/odom", 1, \
      &GridMapper::poseCallback, this);
      
    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED); 
      
      //the timer performs the callback function for printing the occupancy grid to an img file every 30 sec
    timeGrid = nh.createWallTimer(ros::WallDuration(30.0), &GridMapper::printCallback, this);
  };
  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
    //Added this code so as to not overwrite the robots path when plotting obstacles and free-space
    	if (canvas.at<char>(x, y) != CELL_ROBOT) {
    		canvas.at<char>(x, y) = value;
    	}
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };

  //Callback function for printing the canvas to an img file every 30 sec.
  void printCallback(const ros::WallTimerEvent& event) {
  	saveSnapshot();
  }

  //using Bresenham's line drawing algorithm for drawing free space
  //this considers all of the octants for possible slopes
  void line(double x0, double y0, double x1, double y1){
  	int tempX,tempY,dx,dy,dx0,dy0,px,py,xe,ye;
	dx=x1-x0;
	dy=y1-y0;
	dx0=fabs(dx);
	dy0=fabs(dy);
	px=2*dy0-dx0;
	py=2*dx0-dy0;
	if(dy0<=dx0){
	  	if(dx>=0){
			tempX = x0;
		   	tempY = y0;
		   	xe=x1;
	  	}else{
			tempX = x1;
		   	tempY = y1;
		   	xe=x0;
	  	}
	  	plot(tempX, tempY, CELL_FREE);  
		for(int i=0; tempX<xe;i++){
		   tempX = tempX + 1;
		   if(px<0){
		 		px=px+2*dy0;
		   }else{
				if((dx<0 && dy<0) || (dx>0 && dy>0)){
				 	tempY = tempY + 1;
				}else{
				 	tempY =tempY - 1;
				}
		   		px=px+2*(dy0-dx0);
		   }
		   plot(tempX, tempY, CELL_FREE);
	  	}
	 }else{
	  	if(dy>=0){
	   		tempX = x0;
	   		tempY = y0;
	   		ye=y1;
	 	}else{
	   		tempX = x1;
	   		tempY = y1;
	   		ye=y0;
	 	}
	 	plot(tempX, tempY ,CELL_FREE);
	 	for(int i=0;tempY < ye;i++){
	 		tempY = tempY + 1;
	   		if(py<=0){
				py=py+2*dx0;
	   		}else{
				if((dx<0 && dy<0) || (dx>0 && dy>0)){
		 			tempX = tempX + 1;
				}else{
		 			tempX = tempX - 1;
				}
				py=py+2*(dx0-dy0);
	   		}
	   		plot(tempX,tempY,CELL_FREE);
	 	}
  	 }
  };
  
  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
   	
   	//creating a point cloud based on the data from the laser scan message
   	projection.projectLaser(*msg, ptcld);
   	
   	for (int i = 0; i < ptcld.points.size(); i++) {
   		//obtaining x and y points for the coordinate frame from the point cloud
   		double xTemp = -ptcld.points[i].y;
   		double yTemp = ptcld.points[i].x;
   		
   		//combines the temporary points and asjusts for the robot's heading
   		double xHead = (xTemp * cos(heading)) - (yTemp * sin(heading));
   		double yHead = (yTemp * cos(heading)) + (xTemp * sin(heading));
   		
   		//creates the new points of the obstacle on the robots coordinate frame
   		double xObst = x + xHead;
   		double yObst = y + yHead;
   		
   		//performing the plotting of the free space line and the obstacle
   		if(msg->ranges[i] < msg->range_max){
   			line(x * 10, y * 10, xObst * 10, yObst * 10);
   			plot(xObst * 10, yObst * 10, CELL_OCCUPIED);
   		}
   	}
   	
   	//plot the robot's path 
    plot(x * 10, y * 10, CELL_ROBOT);   
      
  };
  
  
  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = -msg->pose.pose.position.y;
    y = msg->pose.pose.position.x;
    heading=tf::getYaw(msg->pose.pose.orientation);
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();
    
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C	
        
//:::::::::::::::::::My code to drive the robot manually using keyboard::::::::::::::::::::::::::::::::::::::::::::::::::::
/**
	  key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
	  
      if(key == 'x' || key == 'X'){	//exiting program

        break;

      }else if (key == 'w'){		//driving forward
      
      	move(FORWARD_SPEED_MPS, 0.0);
      	
      }else if (key == 'a'){		//turning left
      
      	move(0.0, ROTATE_SPEED_RADPS);
      	
      }else if (key == 'd'){		//turning right
      
      	move(0.0, -ROTATE_SPEED_RADPS);
      	
      }else if (key == 's'){		//moving backwards
      
      	move(-FORWARD_SPEED_MPS, 0);
      	
      }else if (key == ' ') {		//taking a snapshot of the occupancy grid

        saveSnapshot();

      }else{						//stopping all movement
      	move(0.0, 0.0);
      }   	
**/ 
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 0.4;
  const static double ROTATE_SPEED_RADPS = M_PI/6;
  
  const static int SPIN_RATE_HZ = 30; 
  
  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic
  
  //my code for turning laser scans into point clouds
  laser_geometry::LaserProjection projection;
  sensor_msgs::PointCloud ptcld;
  //adding timer for printing occupancy grid to a img file every 30 sec.
  ros::WallTimer timeGrid;
  
 
  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
  
  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
};


int main(int argc, char **argv) {
  int width, height;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) { printUsage = true; }
      else if (height <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper robbie(n, width, height); // Create new grid mapper object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
