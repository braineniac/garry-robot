#!/usr/bin/env python

import numpy as np
import rosbag

class Bag2NP:
    def __init__(self, bag_path, out_path):
        self.bag_path = bag_path
        self.out_path = out_path
        self.u_k = [[],[]]
        self.t_k = [[],[]]
        self.read_bag()

    def read_bag(self):
        bag = rosbag.Bag(self.bag_path)
        imu_msgs = bag.read_messages(topics=['/imu/data_raw'])
        for imu_msg in imu_msgs:
            self.imu2np(imu_msg.message)
        twist_msgs = bag.read_messages(topics=['/fake_wheel/twist'])
        for twist_msg in twist_msgs:
            self.twist2np(twist_msg.message)
        print(self.u_k)
        self.save_np()

    def save_np(self):
        np.save(self.out_path,[self.u_k, self.t_k])
        print("Data saved")

    def imu2np(self, imu_msg):
        self.u_k = np.append(self.u_k,[[0],[imu_msg.linear_acceleration.x]],axis=1)
        self.t_k =np.append(self.t_k,[[0],[imu_msg.header.stamp.to_sec()]],axis=1)

    def twist2np(self, twist_msg):
        self.u_k = np.append(self.u_k,[[twist_msg.twist.twist.linear.x],[0]],axis=1)
        self.t_k = np.append(self.t_k,[[twist_msg.header.stamp.to_sec()],[0]],axis=1)

if __name__ == '__main__':
    bag2np = Bag2NP(bag_path="/home/dan/straight_dmp_40hz_46cm.bag", \
                out_path="/home/dan/ros/src/simple_kalman/numpy/straight_dmp.npy")
