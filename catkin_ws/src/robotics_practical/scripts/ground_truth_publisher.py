#! /usr/bin/python3
""" listens to navsat fix msg and publishes odom and transform"""

import utm
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseWithCovariance
import geometry_msgs
import tf
import tf2_ros
# import geometry_msg.msg as msg
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist



def get_gt_from_gmaps():
    # coordinates of 10x7.33 m square
    corners = [(49.421000, 8.659692), # south west
               (49.421021, 8.659787), # south east
               (49.421107, 8.659742), # north east
               (49.421085, 8.659647)] # north west
    edges = [(corners[0], corners[1]),  # west
            (corners[1], corners[2]),  # south
            (corners[2], corners[3]),  # east
            (corners[3], corners[0])]  # north
    all_edges = []

    for edge in edges:
        a, b = edge  # a and b are points
        a = np.array(a)
        b = np.array(b)
        vect = b-a
        x = np.linspace(0,1,1000)
        edge_discretized = np.array([i * vect + a for i in x])
        all_edges += list(edge_discretized)
        # plt.scatter(edge_discretized[:,0], edge_discretized[:,1])
    # plt.show()  
    all_edges = np.array(all_edges)
    return all_edges


def callback(msg):
    all_points = get_gt_from_gmaps()
    current_estimate = np.array([msg.latitude, msg.longitude])
    distances = np.linalg.norm(current_estimate - all_points, axis=1)
    closest_point_index = np.argmin(distances)

    closest_point = all_points[closest_point_index]

    gtmsg = NavSatFix()
    gtmsg.latitude = closest_point[0] 
    gtmsg.longitude = closest_point[1]  
    gtmsg.altitude = msg.altitude
    gtmsg.header = msg.header
    publisher.publish(gtmsg)
    

if __name__=="__main__":
    rospy.init_node("gt_publisher")
    rospy.loginfo("starting gt publisher")
    subscriber = rospy.Subscriber("/vectornav/GPS", NavSatFix, callback)
    publisher = rospy.Publisher("/ground_truth_gps", NavSatFix, queue_size=10)
    rospy.spin()

