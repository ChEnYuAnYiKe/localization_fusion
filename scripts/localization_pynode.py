#!/usr/bin/env python
#coding:utf-8

from sympy import false
import rospy as rp
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu, MagneticField


class imu_data:
    def __init__(self):
        self.timestamp = 0.
        self.acc = np.zeros(3)
        self.gyro = np.zeros(3)
        

class mag_data:
    def __init__(self):
        self.timestamp = 0.
        self.mag_xyz = np.zeros(3)
        self.cov = np.zeros([3, 3])
        

class gps_data:
    def __init__(self):
        self.timestamp = 0.
        self.lla = np.zeros(3)
        self.cov = np.zeros([3, 3])


class model_initializer:
    def __init__(self):
        kImuDataBufferLength = 100
        kMagDataBufferLength = 100
        kAccStdLimit         = 3.
        initialized = false  # whether sys is initialized, default 
        
        init_I_p_Gps = np.zeros(3)
        imu_buffer = deque()
        mag_buffer = deque()


class ekf_localization:
    def __init__(self):
        # get the init param
        acc_noise = rp.get_param("acc_noise", 1e-2)
        gyro_noise = rp.get_param("gyro_noise", 1e-4)
        acc_bais_noise = rp.get_param("acc_bais_noise", 1e-6)
        gyro_bais_noise = rp.get_param("gyrp_bais_noise", 1e-8)
        gravity = np.array([0, 0, -9.81007])

        x = rp.get_param("I_p_Gps_x", 0)
        y = rp.get_param("I_p_Gps_y", 0)
        z = rp.get_param("I_p_Gps_z", 0)
        
        I_p_Gps = np.array([x, y, z])
        
        # Subscribe and publisher
        imu_sub = rp.Subscriber("/imu/data", Imu, self.ImuCallback, queue_size=10)
        mag_sub = rp.Subscriber("/imu/mag", MagneticField, self.MagCallback, queue_size=10)
        gps_sub = rp.Subscriber("/fix", NavSatFix, self.GpsCallback, queue_size=10)
        
        imu_pub = rp.Publisher("imu_path", Path, queue_size=10)
        gps_pub = rp.Publisher("gps_path", Path, queue_size=10)
        fused_pub = rp.Publisher("fused_path", Path, queue_size=10)
        

    def ImuCallback(self, imu_msg):
        imu_data_ = imu_data()
        imu_data_.acc = np.array([imu_msg.linear_acceleration.x, 
                                  imu_msg.linear_acceleration.y,
                                  imu_msg.linear_acceleration.z])
        imu_data_.gyro = np.array([imu_msg.angular_velocity.x,
                                   imu_msg.angular_velocity.y,
                                   imu_msg.angular_velocity.z])
        
    def MagCallback(self, mag_msg):
        pass
    
    def GpsCallback(self, gps_msg):
        pass
    
    
                
if __name__ == '__main__':
    try:
        rp.init_node('ekf_localization')
        
        
    except rp.ROSInterruptException: 
        pass
