# Robotics 2020 - Course Project - Part One
## Finazzi Andrea  
## Personal ids 10530588 - 868454  
  
### Launch command:  
With the folder `robotics2020` inside `{workspace}/src/`:  
`catkin_make`  
`roslaunch robotics2020 project.launch`  

__dynamic_reconfigure GUI__:  
`rosrun rqt_reconfigure rqt_reconfigure`  
  
### Launch file (launch/project.launch):  
Launches a group of nodes for each veicle (front|obs).  
Each group contains:  
`tf/static_transform_publisher` -> static transform between the frames `sensors_gps` and `base_link`  
`robotics2020/lla2enu` -> ENU odometry publisher  
`robotics2020/tf_broadcaster` -> tf publisher  
  
Launches two nodes to provide distance informations:  
`robotics2020/srv_distance` -> stateless server for distance calculation  
`robotics2020/collision_guard` -> Custom message publisher  
  
### Published topics:  
`/r2020/obs/odom : nav_msgs/Odometry` -> Odometry of the `obs/sensors_gps` frame  
`/r2020/front/odom : nav_msgs/Odometry` -> Odometry of the `front/sensors_gps` frame  
`/r2020/collision_topic : robotics2020/CollisionGuardMsg` -> distance informations  
  
### tf tree:  
The tf tree is composed by two branches, one for each veicle, childs of the /map frame.  
Each branch is composed by a `base_link` frame, which is the frame representing the veicle's base, and a `sensors_gps` representing the GPS receiver.  
See the file `frames.pdf` for a graphical representation.  
  
### lla2odom (src/lla2enu.cpp):  
Subscribes to GPS data and publishes an odometry in ENU coordinates.  
__GPS(0,0,0) are republished as (nan,nan,nan)__.  
The __fixed point__ for ENU conversion can be set with the following parameters (float):  
`fixed_point_lat`  
`fixed_point_lon`  
`fixed_point_alt`  
  
If unset, the following default values are set:  
`lat: 45.518934`   
`lon: 9.247184`  
`alt: 222.00000`  
  
### tf_broadcaster (src/tf_broadcaster.cpp):  
Subscribes to an odom topic and transforms the received position to the `base_link` frame (by means of `tf::TransformListener`). 
Then, publishes the tf:
`frame_id` = `/map`  
`child_frame_id` = `<tf_prefix>/base_link`  
  
### distance_service (src/srv/distance.cpp):  
Stateless server. Returns distance between two points in space.  
input params:  
`geometry_msgs/Point pointA`  
`geometry_msgs/Point pointB`  
  
output:  
`float64 distance_m`  
  
### collision_guard (src/tf_broadcaster.cpp):  
Subscribes to ENU odometries (front/odom, obs/odom), asks the server to calculate the distance (with a permanent connection) and publishes a custom message (`msg/CollisionGuardMsg`) with the following structure:  
`float64 distance_m` -> distance between the two  
`string flag (safe|unsafe|crash|SRV_ERROR)` -> status flag  
  
If one of the two odometry position is missing (coord == nan), or if the two odometries are not referenced to the same frame, the following message is published:  
`distance_m: nan`  
`flag: "SRV_ERROR"`  
  
The thresholds determining the status flag value are taken through the __dynamic_reconfigure__ package (`cfg/CollisionGuardParams`):  
`collision_th_safe` (default to 5.0)  
`collision_th_crash` (default to 1.0)  
  
### Possible improvements:  
- A deeper parametrization of the nodes could improve the reusability of the code.  
- The `collision_guard` node could be improved implementing tf resolution, that is, if the odometries are not directly comparable, an attempt to transform the positions could resolve the task, increasing service flexibility.