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
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
    poseSub = nh.subscribe("base_pose_ground_truth", 1, \
      &GridMapper::poseCallback, this);
      
    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
      
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
      canvas.at<char>(x, y) = value;
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

  //using Bresenham's line drawing algorithm for drawing free space
  void line(double x0, double y0, double x1, double y1){
  	double delta_x = x1 - x0;
  	double delta_y = y1 - y0;
  	
  	double delta_err;
  	if(delta_x == 0) delta_err = 0; //when line is vertical
  	else delta_err = abs(delta_y/delta_x); //when line isn't vertical
  	
  	double error = 0.0;  //No error at start
  	int yc = y0;
  	for(int xc = x0; xc < x1; xc++){
  		plot(xc, yc, CELL_FREE);
  		error = error + delta_err;
  		while (error >= 0.5){
  			yc = yc + sin(delta_y) * 1;
  			error = error - 1.0;
  		}
  	}
  	
  }
  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
   	int size = msg->ranges.size();
   	
   	for(int i = 0; i < size; i++){
		double range = msg->ranges[i];
		double angle = msg->angle_min + (i * msg->angle_increment);
		
		double xDist = range*cos(angle);
		double yDist = range*sin(angle);
		
		if(msg->ranges[i] <= msg->range_max){
   			 		plot(xDist, yDist, CELL_OCCUPIED);
   		}		
   		
   		line(x, y, xDist, yDist);	
   	}
  	
   	//ROS_INFO("farthest distance on laser: %f", msg->range_max);
   	plot(x, y, CELL_ROBOT);
    
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

  /**    // TODO: remove following demo code and make robot move around the environment
      plot(x, y, rand() % 255); // Demo code: plot robot's current position on canvas
      plotImg(0, 0, CELL_OCCUPIED); // Demo code: plot different colors at 4 canvas corners
      plotImg(0, canvas.rows-1, CELL_UNKNOWN);
      plotImg(canvas.cols-1, 0, CELL_FREE);
      plotImg(canvas.cols-1, canvas.rows-1, CELL_ROBOT);
	  
  **/  
	
	  //:::::::::::::::::::My code to drive the robot manually using keyboard::::::::::::::::::::::::::::::::::::::::::::::::::::
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
      
      //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;
  
  const static int SPIN_RATE_HZ = 30;
  
  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

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
