import os
import sys
import time
import smbus

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

while True:
    imu.readSensor()
    imu.computeOrientation()
    data = """
                    x                           y                          z
    Acc    {0}         {1}          {2}
    Gyr    {3}         {4}          {5}
    Mag    {6}         {7}          {8}
    RPY    {9}         {10}          {11}
    """.format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], imu.roll, imu.pitch, imu.yaw)
    print("\033[H\033[J", end='')
    print(data)
    time.sleep(0.1)
