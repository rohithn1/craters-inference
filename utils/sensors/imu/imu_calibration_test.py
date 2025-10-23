#!/usr/bin/env python3
import numpy as np
import pygame
import time
from smbus2 import SMBus
from datetime import datetime
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from pywmm import WMMv2
import os
os.environ["SDL_RENDER_DRIVER"] = "software"

# ==========================================================
# CONFIGURATION
# ==========================================================
I2C_BUS = 1
MPU_ADDRESS = MPU9050_ADDRESS_68
CAL_SAMPLES = 500
SAMPLE_PERIOD = 0.01
LAT, LON = 37.3228818, -121.9492194
WINDOW_WIDTH, WINDOW_HEIGHT = 960, 720
HZ = 30
# Car frame of reference
R_MOUNT = np.array([
    [ 0, -1,  0],
    [-1,  0,  0],
    [ 0,  0, -1]
])
CALIBRATION_FILE = "IMU_BIASES.txt"
# ==========================================================
# Madgwick Filter
# ==========================================================
class Madgwick:
    def __init__(self, beta=0.1, q=None):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) if q is None else q

    def update(self, gyro, accel, mag, dt=0.01):
        q1, q2, q3, q4 = self.q
        gx, gy, gz = np.radians(gyro)
        ax, ay, az = accel
        mx, my, mz = mag
        norm = np.linalg.norm(accel)
        if norm == 0: return self.q
        ax, ay, az = accel / norm
        norm = np.linalg.norm(mag)
        if norm == 0: return self.q
        mx, my, mz = mag / norm
        qDot1 = 0.5 * (-q2*gx - q3*gy - q4*gz)
        qDot2 = 0.5 * (q1*gx + q3*gz - q4*gy)
        qDot3 = 0.5 * (q1*gy - q2*gz + q4*gx)
        qDot4 = 0.5 * (q1*gz + q2*gy - q3*gx)
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt
        q = np.array([q1, q2, q3, q4])
        self.q = q / np.linalg.norm(q)
        return self.q

    def to_euler_angles(self):
        q1,q2,q3,q4 = self.q
        sinr_cosp = 2*(q1*q2 + q3*q4)
        cosr_cosp = 1 - 2*(q2*q2 + q3*q3)
        roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))
        sinp = 2*(q1*q3 - q4*q2)
        pitch = np.degrees(np.sign(sinp)*90 if abs(sinp)>=1 else np.arcsin(sinp))
        siny_cosp = 2*(q1*q4 + q2*q3)
        cosy_cosp = 1 - 2*(q3*q3 + q4*q4)
        yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))
        return pitch, roll, yaw

# ==========================================================
# Cube
# ==========================================================
class Wireframe:
    def __init__(self, scale=1.0):
        l, w, h = 0.5*scale, 0.3*scale, 0.2*scale
        self.vertices = np.array([
            [-l,-w,-h],[ l,-w,-h],[ l, w,-h],[-l, w,-h],
            [-l,-w, h],[ l,-w, h],[ l, w, h],[-l, w, h]
        ])
        self.edges = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7)
        ]

    def rotate(self, pitch, roll, yaw):
        pitch, roll, yaw = np.radians([pitch, roll, yaw])
        Rx = np.array([
            [1,0,0],
            [0,np.cos(roll),-np.sin(roll)],
            [0,np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch),0,np.sin(pitch)],
            [0,1,0],
            [-np.sin(pitch),0,np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw),-np.sin(yaw),0],
            [np.sin(yaw), np.cos(yaw),0],
            [0,0,1]
        ])
        R = Rz @ Ry @ Rx
        return (self.vertices @ R.T)

# ==========================================================
# Calibration functions
# ==========================================================
def calibrate_gyro(mpu):
    print("Keep IMU still for gyro calibration...")
    samples = sample_sensor(mpu.readGyroscopeMaster)
    bias = np.mean(samples, axis=0)
    print(f"Gyro bias: {bias}")
    return bias

def calibrate_accel_per_axis_3_point(mpu, n=CAL_SAMPLES):
    print("\n== Accelerometer per-axis calibration ==")
    axes = [('X+', 0), ('Y+', 1), ('Z+', 2)]
    measured = {}
    for label, idx in axes:
        input(f"Place {label} face UP and press Enter to start sampling...")
        samples = sample_sensor(mpu.readAccelerometerMaster, n)
        mean = np.mean(samples, axis=0)
        measured[label[0]] = mean
        print(f"{label} mean: {mean}")
    bias = np.array([
        measured['X'][0]-1.0,
        measured['Y'][1]-1.0,
        measured['Z'][2]-1.0
    ])
    print(f"Accel bias: {bias}")
    return bias

def calibrate_accel_per_axis_6_point(mpu, n=CAL_SAMPLES):
    print("\n== Full 6-Point Accelerometer Calibration ==")
    faces = [
        ('X+', np.array([1, 0, 0])),
        ('X-', np.array([-1, 0, 0])),
        ('Y+', np.array([0, 1, 0])),
        ('Y-', np.array([0, -1, 0])),
        ('Z+', np.array([0, 0, 1])),
        ('Z-', np.array([0, 0, -1]))
    ]

    measured = {}
    for label, _ in faces:
        input(f"Place {label} face UP and press Enter to start sampling...")
        samples = sample_sensor(mpu.readAccelerometerMaster, n)
        mean = np.mean(samples, axis=0)
        measured[label] = mean
        print(f"{label} mean: {mean}")

    bias = np.zeros(3)
    scale = np.ones(3)

    for axis, (plus, minus) in enumerate([('X+', 'X-'), ('Y+', 'Y-'), ('Z+', 'Z-')]):
        plus_mean = measured[plus][axis]
        minus_mean = measured[minus][axis]
        bias[axis] = (plus_mean + minus_mean) / 2.0
        scale[axis] = (2.0) / (plus_mean - minus_mean)

    print(f"\nAccelerometer bias (g): {bias}")
    print(f"Accelerometer scale: {scale}")

    return bias, scale

def get_magnetic_declination(lat, lon):
    try:
        wmm = WMMv2()
        year = datetime.utcnow().year + datetime.utcnow().timetuple().tm_yday / 365.0
        decl = wmm.get_declination(lat, lon, year, 0)
        print(f"Magnetic declination (approx): {decl:.2f}°")
        return np.radians(decl)
    except Exception as e:
        print(f"Declination fetch failed: {e}")
        return 0.0

# ==========================================================
# IMU + Visualization
# ==========================================================
def sample_sensor(func, n=CAL_SAMPLES, delay=SAMPLE_PERIOD):
    data = []
    for _ in range(n):
        data.append(func())
        time.sleep(delay)
    return np.array(data)

def main():
    bus = SMBus(I2C_BUS)
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS,
        address_mpu_master=MPU_ADDRESS,
        address_mpu_slave=None,
        bus=I2C_BUS,
        gfs=GFS_1000,
        afs=AFS_8G,
        mfs=AK8963_BIT_16,
        mode=AK8963_MODE_C100HZ
    )
    mpu.configure()
    print("IMU initialized.")

    gyro_bias = calibrate_gyro(mpu)
    accel_bias, accel_scale = calibrate_accel_per_axis_6_point(mpu)
    declination = get_magnetic_declination(LAT, LON)
    with open(CALIBRATION_FILE, "w") as file:
        file.write(f"GYRO[{gyro_bias}] ACCEL[{accel_bias}] DECLI[declination]")
    madgwick = Madgwick()

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("IMU Orientation Viewer")
    font = pygame.font.SysFont('arial', 18)
    clock = pygame.time.Clock()
    cube = Wireframe(scale=1.0)

    try:
        while True:
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            accel = (np.array(mpu.readAccelerometerMaster()) - accel_bias) * accel_scale
            gyro = np.array(mpu.readGyroscopeMaster()) - gyro_bias
            mag = np.array(mpu.readMagnetometerMaster())
            mag = np.array([
                mag[0]*np.cos(declination) - mag[1]*np.sin(declination),
                mag[0]*np.sin(declination) + mag[1]*np.cos(declination),
                mag[2]
            ])

            madgwick.update(gyro, accel, mag, SAMPLE_PERIOD)
            pitch, roll, yaw = madgwick.to_euler_angles()

            screen.fill((0,0,0))
            verts = cube.rotate(pitch, roll, yaw)
            scale = 300
            dist = 4
            projected = []
            for x,y,z in verts:
                f = scale / (z + dist)
                px = int(WINDOW_WIDTH/2 + f*x)
                py = int(WINDOW_HEIGHT/2 - f*y)
                projected.append((px, py))

            faces = [
                    (0,1,2,3),
                    (4,5,6,7),
                    (0,1,5,4),
                    (2,3,7,6),
                    (0,3,7,4),
                    (1,2,6,5)
                    ]
            face_colors = [
                    (255,0,0),
                    (255,0,0),
                    (0,255,0),
                    (0,255,0),
                    (0,0,255),
                    (0,0,255)
                    ]
            for i, f in enumerate(faces):
                pts = [projected[idx] for idx in f]
                pygame.draw.polygon(screen, face_colors[i], pts)

            for e in cube.edges:
                pygame.draw.line(screen, (0,0,0), projected[e[0]], projected[e[1]], 2)

            text = f"Pitch: {pitch:.1f}  Roll: {roll:.1f}  Yaw: {yaw:.1f}"
            txtsurf = font.render(text, True, (255,255,255))
            screen.blit(txtsurf, (20, 20))

            pygame.display.flip()
            clock.tick(HZ)
            pygame.time.wait(1)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        bus.close()
        pygame.quit()

if __name__ == "__main__":
    main()
