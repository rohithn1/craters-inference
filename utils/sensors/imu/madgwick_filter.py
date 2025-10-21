#!/usr/bin/env python3
import time
import sys
import numpy as np
import cv2
from smbus2 import SMBus
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from Madgwick import Madgwick, q2R

# =============================
#  IMU INITIALIZATION
# =============================

I2C_BUS = 1  # 1 or 2 depending on your board
MPU_ADDRESS = MPU9050_ADDRESS_68  # 0x68 if AD0->GND

bus = SMBus(I2C_BUS)
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU_ADDRESS,
    address_mpu_slave=None,
    bus=I2C_BUS,
    gfs=GFS_1000,      # ±1000 °/s
    afs=AFS_8G,        # ±8 g
    mfs=AK8963_BIT_16, # 16-bit magnetometer
    mode=AK8963_MODE_C100HZ
)
mpu.configure()
print(f"IMU initialized on I2C bus {I2C_BUS}, address 0x{MPU_ADDRESS:02X}\n")

# =============================
#  MADGWICK FILTER SETUP
# =============================

sample_period = 0.05  # 20 Hz sampling
fuse = Madgwick(sampleperiod=sample_period, beta=0.1)
q = np.array([1.0, 0.0, 0.0, 0.0])  # initial quaternion

# =============================
#  OPENCV VISUALIZATION SETUP
# =============================

win_name = "IMU Orientation"
cv2.namedWindow(win_name)

# Define cube points in 3D (unit cube)
cube_points = np.float32([
    [-1, -1, -1],
    [ 1, -1, -1],
    [ 1,  1, -1],
    [-1,  1, -1],
    [-1, -1,  1],
    [ 1, -1,  1],
    [ 1,  1,  1],
    [-1,  1,  1]
]) * 50  # scale up for visibility

# Cube edges (pairs of point indices)
edges = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7)
]

# Simple camera parameters for 3D projection
focal_length = 300
center = (320, 240)
camera_matrix = np.array([
    [focal_length, 0, center[0]],
    [0, focal_length, center[1]],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((4,1))  # no distortion

# =============================
#  MAIN LOOP
# =============================

try:
    while True:
        start_time = time.time()

        # ---- READ IMU ----
        accel = np.array(mpu.readAccelerometerMaster())
        gyro = np.array(mpu.readGyroscopeMaster())
        mag = np.array(mpu.readMagnetometerMaster())

        # ---- UPDATE MADGWICK FILTER ----
        q = fuse.update(q, gyr=gyro, acc=accel, mag=mag)
        R = q2R(q)

        # ---- PROJECT CUBE ----
        # Convert rotation matrix to rotation vector
        rvec, _ = cv2.Rodrigues(R)
        tvec = np.array([[0.0], [0.0], [400.0]])  # move cube away from camera

        projected, _ = cv2.projectPoints(cube_points, rvec, tvec, camera_matrix, dist_coeffs)
        projected = projected.reshape(-1, 2).astype(int)

        # ---- DRAW CUBE ----
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        for (i, j) in edges:
            pt1 = tuple(projected[i])
            pt2 = tuple(projected[j])
            cv2.line(frame, pt1, pt2, (255, 255, 0), 2, cv2.LINE_AA)

        # Display quaternion
        cv2.putText(frame, f"q = [{q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}]",
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Show frame
        cv2.imshow(win_name, frame)

        # Wait briefly and break if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Keep consistent sampling rate
        elapsed = time.time() - start_time
        if elapsed < sample_period:
            time.sleep(sample_period - elapsed)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    bus.close()
    cv2.destroyAllWindows()

