#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

class SaveNPNode:
    def __init__(self):
        self.datapoints=5000
        self.u_k = np.zeros((self.datapoints,2))
        self.t_k = np.zeros((self.datapoints,2))
        self.i = 0

        self.imu_sub = rospy.Subscriber("/imu/data_raw", Imu, self.imu_cb)
        self.fake_enc_sub = rospy.Subscriber("/fake_wheel/twist", TwistWithCovarianceStamped, self.fake_enc_cb)


    def save_np(self):
        np.save("/home/dan/straight_dmp.npy",[self.u_k, self.t_k])
        rospy.signal_shutdown('Quit')

    def check_full(self):
        rospy.loginfo("Datapoint {} of 300 saved".format(self.i + 1))
        if self.i == self.datapoints-1:
            rospy.loginfo("Array full")
            self.save_np()

    def imu_cb(self, imu_msg):
        self.u_k[self.i][1] = imu_msg.linear_acceleration.x
        self.t_k[self.i][1] = rospy.get_rostime().to_sec()
        self.save_np()
        self.i = self.i + 1

    def fake_enc_cb(self, twist_msg):
        self.u_k[self.i][0] = twist_msg.twist.twist.linear.x
        self.t_k[self.i][0] = rospy.get_rostime().to_sec()
        self.save_np()
        self.i = self.i + 1

if __name__ == '__main__':
    rospy.loginfo("Initialising save_np_node")
    rospy.loginfo("Saving up to 300 datapoints")
    rospy.init_node("save_np_node")
    save_np_node = SaveNPNode()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
