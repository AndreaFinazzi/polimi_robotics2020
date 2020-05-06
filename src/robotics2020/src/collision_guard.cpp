#include "ros/ros.h"

#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <robotics2020/CollisionGuardParamsConfig.h>

#include "robotics2020/CollisionGuardMsg.h"
#include "nav_msgs/Odometry.h"

#include "robotics2020/Distance.h"

#define NAMES_NODE "node_name"
#define NAMES_OUTPUT_TOPIC "collision_topic"
#define NAMES_ODOM_TOPIC_A "odom_topic_a"
#define NAMES_ODOM_TOPIC_B "odom_topic_b"
#define NAMES_DISTANCE_SERVICE "distance_service"

class CollisionGuard {
    private:
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> OdomsSyncPolicy;
        typedef message_filters::Synchronizer<OdomsSyncPolicy> OdomsSynchronizer;
        
        // fixed position
        static float collision_th_crash;
        static float collision_th_safe;

        ros::NodeHandle node;

        ros::Publisher publisher;
        robotics2020::CollisionGuardMsg output_msg;

        // TFListener to check comparability
        tf::TransformListener tf_listener;
        tf::Stamped<tf::Vector3> tf_stamped_v;

        // Message filter sync
        message_filters::Subscriber<nav_msgs::Odometry> odom_a_sub;
        message_filters::Subscriber<nav_msgs::Odometry> odom_b_sub;
        boost::shared_ptr<OdomsSynchronizer> synchronizer;

        // Service client
        ros::ServiceClientPtr client;
        robotics2020::Distance distance_srv;

        bool createServiceClient() {
            // Check if client is pointing to an active instance
            if (client && client -> exists()) 
                client -> shutdown();

            // Make client to point a new handle
            client.reset(new ros::ServiceClient(node.serviceClient<robotics2020::Distance>(NAMES_DISTANCE_SERVICE, true)));
            client->waitForExistence(ros::Duration(5));
                
            if (!client->exists()) {
                ROS_ERROR("Can't connect to Service");
                return false;
            }

            return true;
        }

        void callback(const nav_msgs::OdometryConstPtr& odom_a, const nav_msgs::OdometryConstPtr& odom_b) {
            // Check msgs and ask distance_service to calculate distance
            // ROS_INFO("sync callback called");
            distance_srv.request.pointA = odom_a -> pose.pose.position;
            distance_srv.request.pointB = odom_b -> pose.pose.position;

            if (odom_a -> header.frame_id != odom_b -> header.frame_id) {
                // --------- NOT TESTED! ---------
                // Points not directly comparable, look for a tf
                    try
                    {
                        if (tf_listener.canTransform(odom_a -> header.frame_id, odom_b -> header.frame_id, ros::Time(0))) {
                                // Try to get a valid transform
                            tf_listener.waitForTransform(odom_a -> header.frame_id, odom_b -> header.frame_id, ros::Time(0), ros::Duration(0.5));
                            tf_listener.transformPoint(
                                odom_a -> header.frame_id,
                                tf::Stamped<tf::Vector3>(
                                    tf::Vector3(odom_b->pose.pose.position.x, odom_b->pose.pose.position.y, odom_b->pose.pose.position.z),
                                    ros::Time(0),
                                    odom_b -> header.frame_id),
                                tf_stamped_v);
                            // Store transformed point
                            geometry_msgs::PointStamped point_stamped;
                            tf::pointStampedTFToMsg(tf_stamped_v, point_stamped);
                            distance_srv.request.pointB = point_stamped.point;
                        } else {
                            output_msg.distance_m = nanf("");
                            output_msg.flag = robotics2020::CollisionGuardMsg::_FLAG_ERROR_;
                            return;
                        }
                    }
                    catch(const std::exception& e)
                    {
                        std::cerr << e.what() << '\n';
                        output_msg.flag = robotics2020::CollisionGuardMsg::_FLAG_ERROR_;
                        return;
                    }

            }

            // Check if client connection is still alive, otherwise try to restore it
            if (client && client -> call(distance_srv)) {
                // Call service and publish custom message
                output_msg.distance_m = distance_srv.response.distance_m;
                
                // Set status flag based on thresholds
                if (output_msg.distance_m > collision_th_safe) output_msg.flag = robotics2020::CollisionGuardMsg::FLAG_SAFE;
                else if (output_msg.distance_m > collision_th_crash) output_msg.flag = robotics2020::CollisionGuardMsg::FLAG_UNSAFE;
                else output_msg.flag = robotics2020::CollisionGuardMsg::FLAG_CRASH;

                publisher.publish(output_msg);
            } else {
                if (!createServiceClient()) {
                    node.shutdown();
                    exit(-1);
                    // unnecessary?
                    return;
                }
                callback(odom_a, odom_b);
            }
        }

    public:
        CollisionGuard() : 
            odom_a_sub(node, NAMES_ODOM_TOPIC_A, 1000),
            odom_b_sub(node, NAMES_ODOM_TOPIC_B, 1000) {
                // set message filter with custom sync policy
                synchronizer.reset(new OdomsSynchronizer(OdomsSyncPolicy(1), odom_a_sub, odom_b_sub));      
                synchronizer->registerCallback(boost::bind(&CollisionGuard::callback, this, _1, _2));
                
                // Open persistence connection to distance_service
                if (!createServiceClient()) {
                    node.shutdown();
                    exit(-1);
                };

                // Initialize advertiser's handle
                publisher = node.advertise<robotics2020::CollisionGuardMsg>(NAMES_OUTPUT_TOPIC, 1000);
                ROS_INFO("\nCOLLISION GUARD STARTED\n");
            }
        
        static void setThresholds(double* crash, double* safe) {
            CollisionGuard::collision_th_safe = *safe;
            CollisionGuard::collision_th_crash = *crash;
        }

};

float CollisionGuard::collision_th_crash = 0;
float CollisionGuard::collision_th_safe = 0;

void dr_callback(robotics2020::CollisionGuardParamsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f", config.collision_th_crash, config.collision_th_safe);
  CollisionGuard::setThresholds(&(config.collision_th_crash), &(config.collision_th_safe));
}

int main(int argc, char **argv) {
    // sleep(5);
	ros::init(argc, argv, NAMES_NODE);
    //Initialize collision thresholds by dynamic_reconfigure server
    dynamic_reconfigure::Server<robotics2020::CollisionGuardParamsConfig> dr_server;
    dynamic_reconfigure::Server<robotics2020::CollisionGuardParamsConfig>::CallbackType dr_callback_ptr;

    dr_callback_ptr = boost::bind(&dr_callback, _1, _2);
    dr_server.setCallback(dr_callback_ptr);

    CollisionGuard collision_guard;
    
    ros::spin();
    return 0;
}