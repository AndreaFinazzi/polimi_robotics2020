#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import math


def callback(msg):
    rospy.loginfo( "Input position: [%f,%f, %f]", msg.latitude, msg.longitude,msg.altitude)

    #fixed values

    a = 6378137;
    b = 6356752.3142;
    f = (a - b) / a;
    e_sq = f * (2-f);
    deg_to_rad = 0.0174533;
  
    # input data from msg
    latitude = msg.latitude;
    longitude = msg.longitude;
    h = msg.altitude;

    # fixed position
    latitude_init = 45.6311926152;
    longitude_init = 9.2947495255;
    h0 = 231.506675163;

    
    #lla to ecef
    lamb = math.radians(latitude)
    phi = math.radians(longitude)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    rospy.loginfo( "ECEF position: [%f,%f, %f]", x, y,z)

    #ecef to enu
    lamb = math.radians(latitude_init)
    phi = math.radians(longitude_init)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

    rospy.loginfo( "ENU position: [%f,%f, %f]", xEast, yNorth,zUp)
    
def lla2enu():


    rospy.init_node('lla2enu', anonymous=True)

    rospy.Subscriber("/swiftnav/front/gps_pose", NavSatFix, callback)

    rospy.spin()

if __name__ == '__main__':
    lla2enu()
