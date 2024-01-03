#!/usr/bin/env python3
#coding:utf-8

import rospy
import threading
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
import time

# 全局变量存储路径数据和时间戳
path_data = {'fused_path': {'time': [], 'x': [], 'y': [], 'z': []},
             'imu_path': {'time': [], 'x': [], 'y': [], 'z': []},
             'gps_path': {'time': [], 'x': [], 'y': [], 'z': []}}

def update_plot():
    plt.ion()
    fig, ax = plt.subplots(3, 1, figsize=(10, 8))

    while not rospy.is_shutdown():
        for i, axis in enumerate(['x', 'y', 'z']):
            ax[i].clear()
            for path_id, data in path_data.items():
                ax[i].plot(data['time'], data[axis], label=f'{path_id} {axis.upper()} Axis')
            ax[i].legend(loc='upper right')
            ax[i].set_title(f'{axis.upper()} Axis Data vs Time')
            ax[i].set_xlabel('Time (s)')
            ax[i].set_ylabel(f'{axis.upper()} Axis Value')

        plt.draw()
        plt.pause(0.1)

def path_callback(data, path_id):
    global path_data
    current_time = time.time()  # 获取当前时间
    path_data[path_id]['time'].append(current_time)
    path_data[path_id]['x'].append(data.poses[-1].pose.position.x)
    path_data[path_id]['y'].append(data.poses[-1].pose.position.y)
    path_data[path_id]['z'].append(data.poses[-1].pose.position.z)


if __name__ == '__main__':
    rospy.init_node('result_process')

    rospy.Subscriber('/fused_path', Path, path_callback, 'fused_path')
    rospy.Subscriber('/imu_path', Path, path_callback, 'imu_path')
    rospy.Subscriber('/gps_path', Path, path_callback, 'gps_path')

    plot_thread = threading.Thread(target=update_plot)
    plot_thread.start()

    rospy.spin()

