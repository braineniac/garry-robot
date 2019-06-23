#!/usr/bin/env python

import rospy
import matplotlib2tikz
import argparse
from matplotlib import pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry

class LocPlotterNode:
    def __init__(self, topic_odom="/odom", plot_timeout=5.0):
        self.odom_sub = rospy.Subscriber(topic_odom, Odometry, self.odom_cb)

        self.plot_x = []
        self.plot_v = []
        self.plot_t = []

        self.stamps = []

    def odom_cb(self, odom_msg):
        pos_x = odom_msg.pose.position.x
        vel_x = odom.msg.twist.twist.linear.x
        stamp = odom.msg.header.stamp

        self.plot_x.append(pos_x)
        self.plot_v.append(vel_x)
        self.stamps.append(stamp)

        t = stamp - self.stamps[0]
        self.plot_t.append(t)

        if t > self.timeout:
            self.plot_all()
            rospy.signal_shutdown("Finished data collection")

    def plot_all(self):
        plt.figure(1)

        plt.subfigure(211)
        plt.title("Estimated distance")
        plt.plot(self.plot_t, self.plot_x)

        plt.subfigure(212)
        plt.title("Estimated velocity")
        plt.plot(sellf.plot_t, self.plot_v)

        plt.show()

    def export_all(self):
        plt.figure(1)
        plt.title("Estimated distance")
        plt.plot(self.plot_t, self.plot_x)
        matplotlib2tikz.save("plots/ekf_x.tex", figureheight='4cm', figurewidth='14cm' )

        plt.figure(2)
        plt.title("Estimated velocity")
        plt.plot(self.plot_t, self.plot_v)
        matplotlib2tikz.save("plots/ekf_v.tex", figureheight='4cm', figurewidth='14cm' )

if __name__ == '__main__':
    rospy.loginfo("Initialising loc_plotter_node")
    rospy.init_node("loc_plotter_node")

    loc_plotter_node = LocPlotterNode("/odometry/filtered")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
