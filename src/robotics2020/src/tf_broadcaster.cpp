#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#define NAMES_NODE "node_name"
#define NAMES_ODOM_TOPIC "odom_topic"

#define NAMES_FIXED_FRAME_ID "map"
#define NAMES_SENSORS_GPS_FRAME_ID "sensors_gps"
#define NAMES_BASE_FRAME_ID "base_link"

class TFPublisher {
    std::string tf_prefix_key, tf_prefix, tf_frame_fixed, tf_frame_base, tf_frame_sensors_gps;

    ros::NodeHandle node;

    ros::Subscriber odom_sub;

    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;
    tf::Stamped<tf::Vector3> tf_stamped_v;
    tf::StampedTransform tf_stamped_tf;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

        tf_listener.waitForTransform(tf_frame_sensors_gps, tf_frame_base, odom -> header.stamp, ros::Duration(0.5));
        tf_listener.transformPoint(
            tf_frame_sensors_gps,
            tf::Stamped<tf::Vector3>(
                tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z),
                ros::Time(0),
                tf_frame_base),
            tf_stamped_v);
        
        tf_stamped_tf.setOrigin(tf_stamped_v);
        tf_stamped_tf.stamp_ = tf_stamped_v.stamp_;
        tf_broadcaster.sendTransform(tf_stamped_tf);
    }

    public:
        TFPublisher() {
            // Retrieve tf_prefix param
            // get the tf_prefix parameter from the closest namespace
            node.searchParam("tf_prefix", tf_prefix_key);
            node.param(tf_prefix_key, tf_prefix, std::string(""));
            // Resolve frames names
            tf_frame_fixed = tf::resolve("/", NAMES_FIXED_FRAME_ID);
            tf_frame_base = tf::resolve(tf_prefix, NAMES_BASE_FRAME_ID);
            tf_frame_sensors_gps = tf::resolve(tf_prefix, NAMES_SENSORS_GPS_FRAME_ID);

            // tf set up
            tf_stamped_tf.frame_id_ = tf_frame_fixed;
            tf_stamped_tf.child_frame_id_ = tf_frame_base;
            tf_stamped_tf.setRotation(tf::Quaternion(0, 0, 0));

            odom_sub = node.subscribe(NAMES_ODOM_TOPIC, 10, &TFPublisher::odomCallback, this);

            std::string param_name_lat, param_name_lon, param_name_alt;
        }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, NAMES_NODE);
    TFPublisher tf_pub;

    ros::spin();
    return 0;
}