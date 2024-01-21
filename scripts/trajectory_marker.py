#! /usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion

import matplotlib.pyplot as plt
import numpy as np


POS_SCALE = 0.01
MSE_PLOT_MAX_TIME = 120 # in secs


class TrajectoryMarkers:
    def __init__(self):
        self.filtered_marker_counter     = 1
        self.ground_truth_marker_counter = 0
        self.initial_ground_truth_pos    = (0, 0, 0)
        self.filtered_pos                = np.array([0, 0, 0]).astype(np.float64)
        self.ground_truth_pos            = np.array([0, 0, 0]).astype(np.float64)

        self.start_time = rospy.get_time()
        self.saved_plot = False

        rospy.Subscriber("odometry/filtered", Odometry, self.plot_filtered_pos)
        rospy.Subscriber("kitti/oxts/odom", Odometry, self.plot_ground_truth_pos)

        self.filtered_odom_pub = rospy.Publisher('trajectory_markers/filtered', Marker, queue_size = 10)
        self.ground_truth_odom_pub = rospy.Publisher('trajectory_markers/ground_truth', Marker, queue_size = 10)


    def plot_filtered_pos(self, msg):

        self.filtered_pos = np.vstack((self.filtered_pos, [
            msg.pose.pose.position.x * POS_SCALE,
            msg.pose.pose.position.y * POS_SCALE,
            msg.pose.pose.position.z * POS_SCALE
        ]))

        # Creater marker
        marker = Marker(type     = Marker.SPHERE, 
                        lifetime = rospy.Duration(0),
                        pose     = Pose(Point
                                        (
                                            self.filtered_pos[self.filtered_marker_counter][0], 
                                            self.filtered_pos[self.filtered_marker_counter][1], 
                                            self.filtered_pos[self.filtered_marker_counter][2]), 
                                            Quaternion(0.0, 0.0, 0.0, 1.0)
                                        ),
                        scale    = Vector3(0.1, 0.1, 0.1), 
                        header   = Header(frame_id = 'odom'),
                        color    = ColorRGBA(0.0, 1.0, 0.0, 1.0), # Green
                        id       = self.filtered_marker_counter)
        
        # Publish marker
        self.filtered_odom_pub.publish(marker)

        self.filtered_marker_counter += 1


    def plot_ground_truth_pos(self, msg):
        if self.ground_truth_marker_counter == 0:
            self.initial_ground_truth_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            self.ground_truth_marker_counter += 1
            return

        self.ground_truth_pos = np.vstack((self.ground_truth_pos, [
            (msg.pose.pose.position.x - self.initial_ground_truth_pos[0]) * POS_SCALE, 
            (msg.pose.pose.position.y - self.initial_ground_truth_pos[1]) * POS_SCALE,
            (msg.pose.pose.position.z - self.initial_ground_truth_pos[2]) * POS_SCALE
        ]))

        # Creater marker
        marker = Marker(type     = Marker.SPHERE, 
                        lifetime = rospy.Duration(0),
                        pose     = Pose(Point
                                    (
                                        self.ground_truth_pos[self.ground_truth_marker_counter][0], 
                                        self.ground_truth_pos[self.ground_truth_marker_counter][1], 
                                        self.ground_truth_pos[self.ground_truth_marker_counter][2]), 
                                        Quaternion(0.0, 0.0, 0.0, 1.0)
                                    ),
                        scale    = Vector3(0.1, 0.1, 0.1), 
                        header   = Header(frame_id = 'odom'),
                        color    = ColorRGBA(1.0, 1.0, 0.0, 1.0), # Yellow
                        id       = self.ground_truth_marker_counter)

        # Publish marker
        self.ground_truth_odom_pub.publish(marker)

        self.ground_truth_marker_counter += 1

        if rospy.get_time() - self.start_time > MSE_PLOT_MAX_TIME and not self.saved_plot:
            self.calc_and_save_mse_plot()
            self.saved_plot = True


    def calc_and_save_mse_plot(self):
        fig = plt.figure(figsize = (12,6))

        ax_x = plt.subplot(121)
        ax_x.set_title("MSE X")
        ax_x.set_xlabel("steps")
        ax_x.set_ylabel("mse")

        ax_y = plt.subplot(122)
        ax_y.set_title("MSE Y")
        ax_y.set_xlabel("steps")
        ax_y.set_ylabel("mse")

        # Calculate mean squared error
        se_x = 0.0       # sum of squared error for x
        se_y = 0.0       # sum of squared error for y
        mse_x = []       # mean sqaured error for x at each step
        mse_y = []       # mean sqaured error for y at each step

        print("Calculating mse for x and y both...")

        for i in range(1, self.ground_truth_marker_counter):
            se_x += (self.ground_truth_pos[i][0] - self.filtered_pos[i][0]) * (self.ground_truth_pos[i][0] - self.filtered_pos[i][0])
            se_y += (self.ground_truth_pos[i][1] - self.filtered_pos[i][1]) * (self.ground_truth_pos[i][1] - self.filtered_pos[i][1])
            mse_x.append(se_x / i)
            mse_y.append(se_y / i)

        ax_x.plot(mse_x)
        ax_y.plot(mse_y)

        plt.show()
        fig.savefig("fig") # path $HOME/.ros/fig.png
        print("SAVED PLOT")


if __name__ == '__main__':
    rospy.init_node("trajectory_markers_node", anonymous = True)
    trajectory_interactive_markers = TrajectoryMarkers()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
