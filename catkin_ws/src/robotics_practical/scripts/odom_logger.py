#! /usr/bin/python3
""" logs all odom msgs to a format that is usable in matplotlib"""

import os
import time

import csv
import rospy
import numpy as np
from nav_msgs.msg import Odometry


class OdomListener():
    def __init__(self, topic, output_dir):
        self.topic = topic
        self.output_dir = output_dir

        self.captured_data = []
        self.last_timestamp = rospy.Time()
        self.reached_eof = False

        rospy.loginfo("started subscriber - entering spin")
        rospy.Subscriber(topic, Odometry, self.callback)
    
        rospy.on_shutdown(self.save_csv_and_exit)

    def callback(self, msg):
        timestamp = msg.header.stamp
   
        self.last_timestamp = timestamp
        position = msg.pose.pose.position
        position = [position.x, position.y, position.z]
        orientation = msg.pose.pose.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        covariance = msg.pose.covariance  # 6x6 covariance matrix in 36,1 shape
        seconds = timestamp.secs + timestamp.nsecs / 1000000000

        row = np.array(position + orientation + [seconds] + list(covariance))
        self.captured_data.append(row)

    def save_csv_and_exit(self):
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        output_name = self.output_dir + "/" + self.topic.replace("/","_") + ".csv"
        rospy.loginfo("saving csv at " + str(output_name))
        self.captured_data = np.array(self.captured_data)
        np.savetxt(output_name, self.captured_data, delimiter=",")


if __name__ == '__main__':
    rospy.init_node('odom_logger', anonymous=True)
    bagfile = rospy.get_param("/bagfile_name")
    
    output_dir = "/home/ubuntu/tmp/odomdata_" + bagfile.split("/")[-1][:-4]
    
    OdomListener("/odometry/filtered", output_dir) 
    OdomListener("/vectornav/GPS/utm_odom/updated_shifted", output_dir) 
    OdomListener("/vectornav/Odom/updated_shifted", output_dir) 
    OdomListener("/ublox_gps/fix/utm_odom", output_dir) 
    #OdomListener("/ground_truth_gps/utm_odom", output_dir) 
    rospy.spin()
