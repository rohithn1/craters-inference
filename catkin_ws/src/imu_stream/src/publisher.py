#!/usr/bin/env python3

import smbus
import os
import sys
import time
from imusensor.MPU9250 import MPU9250
import rospy
from sensor_msgs.msg import Imu
import numpy as np

def euler_to_quaternion(r, p, y):
    r = np.deg2rad(r)
    p = np.deg2rad(p)
    y = np.deg2rad(y)
    qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
    qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
    qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
    qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
    return [qx, qy, qz, qw]
    
def talker():
    rospy.init_node('imu', anonymous=False)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size= 10)
    rate = rospy.Rate(120)
    
    address = 0x68
    bus = smbus.SMBus(1)
    imu = MPU9250.MPU9250(bus, address)
    imu.begin()
    
    while not rospy.is_shutdown():
        imu.readSensor()
        imu.computeOrientation()
        
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        orientation_quaternion = euler_to_quaternion(imu.roll, imu.pitch, imu.yaw)
        
        imu_msg.orientation.x = orientation_quaternion[0]
        imu_msg.orientation.y = orientation_quaternion[1]
        imu_msg.orientation.z = orientation_quaternion[2]
        imu_msg.orientation.w = orientation_quaternion[3]
        imu_msg.orientation_covariance[0] = -1
        
        imu_msg.angular_velocity.x = imu.GyroVals[0]
        imu_msg.angular_velocity.y = imu.GyroVals[1]
        imu_msg.angular_velocity.z = imu.GyroVals[2]
        imu_msg.angular_velocity_covariance[0] = -1
        
        imu_msg.linear_acceleration.x = imu.AccelVals[0]
        imu_msg.linear_acceleration.y = imu.AccelVals[1]
        imu_msg.linear_acceleration.z = imu.AccelVals[2]
        imu_msg.linear_acceleration_covariance[0] = -1
        
        imu_pub.publish(imu_msg)
        
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except Exception as e:
        rospy.logerr(e)
