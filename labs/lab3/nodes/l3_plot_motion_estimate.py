#!/usr/bin/env python3
import rospy
import rosbag
from utils import convert_pose_to_tf, euler_from_ros_quat, ros_quat_from_euler
import numpy as np
import matplotlib.pyplot as plt
import rospkg

def plot(bag):
    data = {"odom_est":{"time":[], "data":[]},
            "odom_onboard":{'time':[], "data":[]}}
    start_time = None
    for topic, msg, t in bag.read_messages(topics=['odom_est', 'odom_onboard']):
        if start_time is None:
            start_time = msg.header.stamp.to_sec()
        d = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                euler_from_ros_quat(msg.pose.pose.orientation)[2]]
        data[topic]["time"].append(msg.header.stamp.to_sec() - start_time)
        data[topic]["data"].append(d)
    for key1 in data:
        for key2 in data[key1]:
            data[key1][key2] = np.array(data[key1][key2])

    data_labels = ["x", "y", "theta"]
    f, axes = plt.subplots(3, 1, sharex=True)
    for k in range(3):
        for key, val in data.items():
            axes[k].plot(val["time"], val["data"][:, k], label=key)
            axes[k].set_ylabel(data_labels[k])
            axes[k].legend()
            axes[k].grid()
    axes[2].set_xlabel("time (s)")
    
    plt.show()


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    path = rospack.get_path("rob521_lab3")
    bag = rosbag.Bag(path+"/motion_estimate.bag")
    plot(bag)