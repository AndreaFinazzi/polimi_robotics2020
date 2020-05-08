cmake_minimum_required(VERSION 2.8.3)
project(robotics2020)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs tf nav_msgs message_generation dynamic_reconfigure)
#geodesy

generate_dynamic_reconfigure_options(
  cfg/CollisionGuardParams.cfg

)

add_service_files(
  FILES
  Distance.srv
)

add_message_files(
  FILES
  CollisionGuardMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(srv_distance src/srv/distance.cpp)
target_link_libraries(srv_distance ${catkin_LIBRARIES})

add_executable(lla2enu src/lla2enu.cpp)
target_link_libraries(lla2enu ${catkin_LIBRARIES})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(collision_guard src/collision_guard.cpp)
target_link_libraries(collision_guard ${catkin_LIBRARIES})
add_dependencies(collision_guard ${PROJECT_NAME}_gencfg)
