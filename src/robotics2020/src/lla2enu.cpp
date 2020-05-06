#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#define WGS84_A 6378137
#define WGS84_B 6356752.3142
#define WGS84_FLATTENING ((WGS84_A - WGS84_B) / WGS84_A)
#define WGS84_E_SQ (WGS84_FLATTENING * (2 - WGS84_FLATTENING))
#define WGS84_DEG_TO_RAD 0.0174533
#define DEFAULTS_ENU_FP_LAT 45.518934
#define DEFAULTS_ENU_FP_LON 9.247184
#define DEFAULTS_ENU_FP_ALT 222.00000

#define NAMES_NODE "node_name"
#define NAMES_ODOM_TOPIC "odom_topic"
#define NAMES_LLA_TOPIC "lla_topic"
#define NAMES_PARAMS_FP_LAT "fixed_point_lat"
#define NAMES_PARAMS_FP_LON "fixed_point_lon"
#define NAMES_PARAMS_FP_ALT "fixed_point_alt"

#define NAMES_FIXED_FRAME_ID "map"
#define NAMES_SENSORS_GPS_FRAME_ID "sensors_gps"
#define NAMES_BASE_FRAME_ID "base_link"


// 
class ENUConverter {
    private:
        // fixed position
        static float latitude_init;
        static float longitude_init;
        static float h_init;
        
        // conversion tools
        // Here to reduce memory initialization occurrence
        float lamb;
        float phi;
        float s;
        float N;

        float  sin_lambda;
        float  cos_lambda;
        float  sin_phi;
        float  cos_phi;

        float  ECEF_x;
        float  ECEF_y;
        float  ECEF_z;

        float  x0;
        float  y0;
        float  z0;

        float  xd;
        float  yd;
        float  zd;

        float  ENU_x;
        float  ENU_y;
        float  ENU_z;

    public:
        static void setFixedPoint(float lat, float lon, float alt) {
            ENUConverter::latitude_init = lat;
            ENUConverter::longitude_init = lon;
            ENUConverter::h_init = alt;
        }

        void setOdomFromLLA(const double* lat, const double* lon, const double* alt, geometry_msgs::Point* position) {
            //lla to ecef
            lamb = WGS84_DEG_TO_RAD * (*lat);
            phi = WGS84_DEG_TO_RAD * (*lon);
            s = sin(lamb);
            N = WGS84_A / sqrt(1 - WGS84_E_SQ * s * s);

            sin_lambda = sin(lamb);
            cos_lambda = cos(lamb);
            sin_phi = sin(phi);
            cos_phi = cos(phi);

            ECEF_x = (*alt + N) * cos_lambda * cos_phi;
            ECEF_y = (*alt + N) * cos_lambda * sin_phi;
            ECEF_z = (*alt + (1 - WGS84_E_SQ) * N) * sin_lambda;
            
            // ecef to enu
            
            lamb = WGS84_DEG_TO_RAD * (ENUConverter::latitude_init);
            phi = WGS84_DEG_TO_RAD * (ENUConverter::longitude_init);
            s = sin(lamb);
            N = WGS84_A / sqrt(1 - WGS84_E_SQ * s * s);

            sin_lambda = sin(lamb);
            cos_lambda = cos(lamb);
            sin_phi = sin(phi);
            cos_phi = cos(phi);

            x0 = (ENUConverter::h_init + N) * cos_lambda * cos_phi;
            y0 = (ENUConverter::h_init + N) * cos_lambda * sin_phi;
            z0 = (ENUConverter::h_init + (1 - WGS84_E_SQ) * N) * sin_lambda;

            xd = ECEF_x - x0;
            yd = ECEF_y - y0;
            zd = ECEF_z - z0;

            ENU_x = -sin_phi * xd + cos_phi * yd;
            ENU_y = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
            ENU_z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

            // ROS_INFO("ENU position: [%f,%f, %f]", ENU_x, ENU_y,ENU_z);
            position->x = ENU_x;
            position->y = ENU_y;
            position->z = ENU_z;
        }
};

float ENUConverter::latitude_init = 0;
float ENUConverter::longitude_init = 0;
float ENUConverter::h_init = 0;

class ENUPublisher {
    std::string tf_prefix_key, tf_prefix, tf_frame_fixed, tf_frame_base, tf_frame_sensors_gps;

    ros::NodeHandle node;

    ros::Subscriber lla_sub;
    ros::Publisher enu_pub;
    nav_msgs::Odometry last_odom;

    ENUConverter enu_converter;

    void llaCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->latitude > 0 && msg->longitude > 0 && msg->altitude > 0) {
            last_odom.header.stamp = ros::Time::now();
            enu_converter.setOdomFromLLA(&(msg->latitude), &(msg->longitude), &(msg->altitude), &(last_odom.pose.pose.position));
        } else {
            // handle GPS (0,0,0)
        }

        enu_pub.publish(last_odom);
    }

    public:
        ENUPublisher() {
            // Retrieve tf_prefix param
            // get the tf_prefix parameter from the closest namespace
            node.searchParam("tf_prefix", tf_prefix_key);
            node.param(tf_prefix_key, tf_prefix, std::string(""));
            // Resolve frames names
            tf_frame_fixed = tf::resolve("/", NAMES_FIXED_FRAME_ID);
            tf_frame_base = tf::resolve(tf_prefix, NAMES_BASE_FRAME_ID);
            tf_frame_sensors_gps = tf::resolve(tf_prefix, NAMES_SENSORS_GPS_FRAME_ID);

            // Frames setting
            last_odom.header.frame_id = tf_frame_fixed;
            last_odom.child_frame_id = tf_frame_sensors_gps;

            lla_sub = node.subscribe(NAMES_LLA_TOPIC, 1000, &ENUPublisher::llaCallback, this);
            enu_pub =  node.advertise<nav_msgs::Odometry>(NAMES_ODOM_TOPIC, 1000);

            std::string param_name_lat, param_name_lon, param_name_alt;
            // Look for params
            if (node.searchParam(NAMES_PARAMS_FP_LAT, param_name_lat) && node.searchParam(NAMES_PARAMS_FP_LON, param_name_lon) && node.searchParam(NAMES_PARAMS_FP_ALT, param_name_alt))
                {
                    float fp_lat, fp_lon, fp_alt;
                    
                    if (node.getParam(param_name_lat, fp_lat) &&
                        node.getParam(param_name_lon, fp_lon) &&
                        node.getParam(param_name_alt, fp_alt)) {

                            ENUConverter::setFixedPoint(fp_lat, fp_lon, fp_alt);
                    } else {
                        ROS_WARN("Cannot retrieve Fixed Point parameters.");
                    }
                } else {
                    ENUConverter::setFixedPoint(DEFAULTS_ENU_FP_LAT, DEFAULTS_ENU_FP_LON, DEFAULTS_ENU_FP_ALT);
                }
        }
};

int main(int argc, char **argv) {
    sleep(10);
	ros::init(argc, argv, NAMES_NODE);
    ENUPublisher enu_pub;

    ros::spin();
    return 0;
}