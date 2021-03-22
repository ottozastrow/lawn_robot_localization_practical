#!/usr/bin/python3
""" listens to odom msg and publishes an updated version"""

import time

import geometry_msgs.msg
import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseWithCovariance

import tf

class ShiftSensorPos():
    def __init__(self, topic, parent, child):
        self.tfBuffer = tf2_ros.Buffer()
        self.parent = parent
        self.child = child
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.publisher = rospy.Publisher(topic, Odometry, queue_size=1)
        rospy.loginfo("started shifting vectornav")

    def update(self):
        # publish updated odom msg
        try:
            trans = self.tfBuffer.lookup_transform(self.parent, self.child, rospy.Time(0))
            odom_msg = Odometry()
            odom_msg.pose.pose.position = trans.transform.translation
            odom_msg.pose.pose.orientation = trans.transform.rotation
            odom_msg.header.stamp = rospy.Time.now()
           
            # fake covariance
            covariance = np.array(
                [1.5, 0, 0,
                 0, 1.5, 0,
                 0, 0, 3])
            covariance = np.reshape(covariance, (3,3))
            cov6x6 = np.zeros((6,6))
            cov6x6[:3,:3] = covariance
            cov6x6_flat = np.reshape(cov6x6, (36, )).astype(np.float64)
            odom_msg.pose.covariance = cov6x6_flat

            odom_msg.header.frame_id = "map"
            self.publisher.publish(odom_msg)
            rospy.logdebug("found transform")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("transform with these frames not found")
            rospy.logerr(str(e))
if __name__ == '__main__':
    rospy.init_node('shift_publisher')

    rate = 60
    odom = ShiftSensorPos("vectornav/Odom/updated_shifted", "map", "vectornav_trailer")
    gps = ShiftSensorPos("vectornav/GPS/utm_odom/updated_shifted", "map", "vectornav_gps_trailer")
    transforms = [odom, gps]
    while not rospy.is_shutdown():
        for trans in transforms:
            trans.update()
        time.sleep(1/rate)
