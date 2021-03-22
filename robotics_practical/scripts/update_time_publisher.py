#!/usr/bin/python3
""" ROS Node that listens to given odom msg and publishes an updated version with the correct timestamp.
Additionally the node will publish the respective transform if specified.

The vectornav gps data does not include orientation data. In order to include the sensor mounting position offset this orientation data is needed. For this purpose the rotation_topic argument can be used.
"""

import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
import rospy
import tf2_ros
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler



class UpdatedPublisher():
    def __init__(self, topic, msgtype, frame=None, child_frame=None, rotation_topic= None, publish_updated_time=True):
  
        self.msgtype = msgtype
        self.frame=frame
        self.update_time = publish_updated_time
        self.rotation_topic = rotation_topic
        self.child_frame = child_frame
        rospy.Subscriber(topic, self.msgtype, self.callback)
        self.publisher = rospy.Publisher(topic + '_updated', msgtype, queue_size=1)

        if rotation_topic:
            self.last_rotation = Quaternion(0, 0, 0, 1)
            rospy.Subscriber(rotation_topic, self.msgtype, self.rotation_callback)
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def rotation_callback(self, msg):
        self.last_rotation = msg.pose.pose.orientation
        rospy.loginfo("updated last rotation to " + str(self.last_rotation))

    def callback(self, msg):
        # publish updated odom msg
            

        if self.update_time:
            updated_msg = msg
            updated_msg.header.stamp = rospy.Time.now()
            updated_msg.header.frame_id=self.frame
            self.publisher.publish(updated_msg)
        if self.frame:
            
            # publish transform
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.frame
            t.child_frame_id = self.child_frame
            t.transform.translation = msg.pose.pose.position
            if self.rotation_topic:
                # get rotation from another topic - use to add rotation data to gps
                t.transform.rotation = self.last_rotation
            else:
                t.transform.rotation = msg.pose.pose.orientation
            rospy.loginfo("publishing transform")
            
            self.broadcaster.sendTransform(t)
           
if __name__ == '__main__':
    rospy.init_node('updated_publisher')
    UpdatedPublisher("/vectornav/Odom", Odometry, frame="vectornav_2", child_frame="vectornav_2_rotated")
    UpdatedPublisher("/vectornav/GPS/utm_odom", Odometry, frame="map", child_frame="vectornav_gps", rotation_topic = "/vectornav/Odom_updated", publish_updated_time=False)
    UpdatedPublisher("/vectornav/IMU", Imu)
    UpdatedPublisher("/imu", Imu)

    rospy.spin()
