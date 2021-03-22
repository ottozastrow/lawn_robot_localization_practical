#!/usr/bin/env python
""" listens to odom msg and publishes an updated version"""

import geometry_msgs.msg
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class TransformPublisher():
    def __init__(self, topic, frame_id, child_frame_id, publish_transform=False):
        rospy.loginfo("waiting for topic: " + str(topic))
        rospy.Subscriber(topic, Odometry, self.callback)
        self.publisher = rospy.Publisher(topic + '_updated', Odometry, queue_size=10)
        
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.child_frame_id = child_frame_id
        self.frame_id = frame_id
        self.publish_transform = publish_transform

    def callback(self, msg):
        # publish updated odom msg
        updated_msg = msg
        updated_msg.header.frame_id = self.frame_id
        rospy.loginfo(updated_msg)
        self.publisher.publish(updated_msg)
        
        # publish transform
        if self.publish_transform:
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = msg.header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = self.child_frame_id
            rospy.loginfo(msg)
            t.transform.translation = msg.pose.pose.position
            t.transform.translation.z = 0.0
            t.transform.rotation = msg.pose.pose.orientation

            self.broadcaster.sendTransform(t)



if __name__ == '__main__':
    rospy.init_node('imu_transform_publisher')
    TransformPublisher("/vectornav/Odom", "map", "base_vectornav")
    rospy.spin()
