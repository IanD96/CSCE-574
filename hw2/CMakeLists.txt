cmake_minimum_required(VERSION 2.8.3)
project(hw2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs)



###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(pdcontrol pdcontrol.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pdcontrol ${catkin_LIBRARIES})
