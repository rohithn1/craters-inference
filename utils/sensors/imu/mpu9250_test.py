#!/usr/bin/env python3
import time
import sys
from smbus2 import SMBus
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

I2C_BUS = 1 # 1 or 2
MPU_ADDRESS = MPU9050_ADDRESS_68  # 0x68 since AD0 -> GND and NCS -> floating

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

print(f"IMU initialized on I2C bus {I2C_BUS}, address 0x{MPU_ADDRESS:02X}")

print("\n" * 3)

try:
    while True:
        accel = mpu.readAccelerometerMaster()
        gyro = mpu.readGyroscopeMaster()
        mag = mpu.readMagnetometerMaster()

        sys.stdout.write("\033[F" * 3)

        print(f"Accel (g):  x={accel[0]: .3f}, y={accel[1]: .3f}, z={accel[2]: .3f}")
        print(f"Gyro (°/s): x={gyro[0]: .3f}, y={gyro[1]: .3f}, z={gyro[2]: .3f}")
        print(f"Mag  (µT):  x={mag[0]: .3f}, y={mag[1]: .3f}, z={mag[2]: .3f}")

        sys.stdout.flush()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    bus.close()
