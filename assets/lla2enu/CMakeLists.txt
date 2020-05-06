cmake_minimum_required(VERSION 2.8.3)
project(lla2enu)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geodesy sensor_msgs tf nav_msgs )






## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(lla2enu src/sub.cpp)
target_link_libraries(lla2enu ${catkin_LIBRARIES})


