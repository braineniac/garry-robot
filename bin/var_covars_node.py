#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np

class VarCovarsNode:
    def __init__(self):
        self.fake_wheel_sub = rospy.Subscriber("/fake_wheel/twist", TwistWithCovarianceStamped, self.fake_wheel_cb)
        self.fake_wheel_pub = rospy.Publisher("/fake_wheel/twist_var", TwistWithCovarianceStamped, queue_size=1)

        self.covariance_euler = np.zeros(36)
        self.covariance_euler[0] = ((400/1000000) * 9.80655)**2 / 100000.0

    def fake_wheel_cb(self, msg):
        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.twist = msg.twist

        new_msg.twist.covariance = self.covariance_euler

        self.fake_wheel_pub.publish(new_msg)

if __name__ == '__main__':
    rospy.loginfo("Initialising var_covar_node")
    rospy.init_node("var_covars_node")

    var_covar_node = VarCovarsNode()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
