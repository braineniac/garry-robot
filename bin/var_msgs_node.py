#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
import numpy as np

class VarMsgsNode:
    def __init__(self, imu_in="/imu",
                       imu_out="/imu/var",
                       twist_in="/fake_wheel/twist",
                       twist_out="/fake_wheel/twist_var"
                       ):
        self.twist_sub = rospy.Subscriber(twist_in, TwistWithCovarianceStamped, self.twist_cb)
        self.twist_pub = rospy.Publisher(twist_out, TwistWithCovarianceStamped, queue_size=1)

        self.imu_sub = rospy.Subscriber(imu_in, Imu, self.imu_cb)
        self.imu_pub = rospy.Publisher(imu_out, Imu, queue_size=1)

        self.twist_peak_vel = 0.282087447/2.0

        self.covariance_euler = np.zeros(36)
        self.covariance_euler[0] = 0.04 * ratio*ratio

    def twist_cb(self, twist_msg):
        new_twist_msg = TwistWithCovarianceStamped()

        new_twist_msg.header = twist_msg.header
        new_twist_msg.header.frame_id = "base_link"

        new_twist_msg.twist = twist_msg.twist

        new_twist_msg.twist.covariance = self.covariance_euler

        self.twist_pub.publish(new_twist_msg)

    def imu_cb(self, imu_msg):
        new_imu_msg = Imu()

        new_imu_msg.header = imu_msg.header
        new_imu_msg.header.frame_id = "imu_link"

        new_imu_msg.orientation = imu_msg.orientation
        new_imu_msg.orientation_covariance = imu_msg.orientation_covariance

        new_imu_msg.angular_velocity = imu_msg.angular_velocity
        new_imu_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance

        new_imu_msg.linear_acceleration = imu_msg.linear_acceleration
        new_imu_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance

        self.imu_pub.publish(new_imu_msg)

if __name__ == '__main__':
    rospy.loginfo("Initialising var_msgs_node")
    rospy.init_node("var_msgs_node")
    ratio = rospy.get_param("~ratio", 1/3.)
    rospy.loginfo("The ratio is: {}".format(ratio))

    var_covar_node = VarMsgsNode(ratio=ratio)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
