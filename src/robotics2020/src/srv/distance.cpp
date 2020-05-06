#include "ros/ros.h"
#include "geometry_msgs/Point.h"
// #include <math.h>

#include "robotics2020/Distance.h"

#define NAMES_NODE "distance_service"
#define NAMES_SERVICE "distance_service"

// Stateless service that returns distance between two geometry_msgs::Point objects
class DistanceService {
    ros::NodeHandle node;
    ros::ServiceServer server;

    public:
        bool serviceCallback(robotics2020::Distance::Request &req, robotics2020::Distance::Response &res) {
            // 3D distance between points
            res.distance_m = sqrtf64(
                powf64(req.pointA.x - req.pointB.x, 2) + 
                powf64(req.pointA.y - req.pointB.y, 2) + 
                powf64(req.pointA.z - req.pointB.z, 2));
        }

        DistanceService() {
            server = node.advertiseService(NAMES_SERVICE, &DistanceService::serviceCallback, this);
        }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, NAMES_NODE);
    DistanceService service;

    ros::spin();
    return 0;
}