#! /home/ottozastrow/anaconda3/bin/python
""" listens to navsat fix msg and publishes odom and transform"""


import utm
import rospy
from std_msgs.msg import String
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


class UtmTransformPublisher():
    def __init__(self, topic, 
                 fix_covariance=False, sensor_offset_transform=None):
        self.zone_number = 0
        self.zone_letter = "-"
        self.start_e = 0
        self.start_n = 0
        self.start_alt = 0
        self.start_pos_was_set = False
        self.topic = topic
        self.publisher = rospy.Publisher(
                topic + '/utm_odom', Odometry, queue_size=10)

        rospy.loginfo("started subscriber - entering spin")
        rospy.Subscriber(topic, NavSatFix, self.callback)
        self.offset = sensor_offset_transform
        self.fix_covariance = fix_covariance

    def callback(self, data):
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        e,n, self.zone_number, self.zone_letter = utm.from_latlon(lat, lon)
        if not self.start_pos_was_set:
            self.start_e = e
            self.start_n = n
            """
            if self.offset:
                self.start_e -= self.offset.translation.x
                self.start_n -= self.offset.translation.y
                """
            if self.offset:
                self.start_e -= 0.4
                self.start_n -= 0.4
            self.start_pos_was_set = True
            self.start_alt = data.altitude

        position = Point()
        position.x = e - self.start_e 
        position.y = n - self.start_n
        position.z = 0.0
        erel = e - self.start_e  # relative position to start position in meters
        nrel = n - self.start_n
        alt_rel = alt - self.start_alt

        # empty as the gps data doesnt include orientation
        orientation = Quaternion()
        orientation.w = 1.0
        
        mypose = Pose()
        mypose.orientation = orientation
        mypose.position = position

        if self.fix_covariance:
            covariance = np.array(
                [1.5, 0, 0,
                 0, 1.5, 0,
                 0, 0, 3])
        else:
            covariance = data.position_covariance
        covariance = np.reshape(covariance, (3,3))

        cov6x6 = np.zeros((6,6))
        cov6x6[:3,:3] = covariance
        cov6x6_flat = np.reshape(cov6x6, (36, )).astype(np.float64)
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.covariance = cov6x6_flat
        pose_with_cov.pose = mypose

        odom = Odometry()
        odom.pose = pose_with_cov
        odom.header = data.header
        odom.header.stamp = data.header.stamp
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        # odom.twist stays empty as gps doesn't have this data

        self.publisher.publish(odom)

if __name__ == '__main__':
    rospy.init_node('utm_transform', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    try:
        vectornav_offset = tfBuffer.lookup_transform(
            "vectornav_gps", "vectornav_gps_trailer", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("transform with these frames not found")
            rospy.logerr(str(e))
            vectornav_offset = None

    UtmTransformPublisher("/vectornav/GPS", fix_covariance=True,
                          sensor_offset_transform=vectornav_offset)   
    UtmTransformPublisher("/ublox_gps/fix")   
    #UtmTransformPublisher("/ground_truth_gps")   
    rospy.spin()

