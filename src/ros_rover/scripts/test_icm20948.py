#!/usr/bin/env python3
"""
ICM20948 standalone sensor test.
Run directly on the robot (no ROS needed):

    python3 scripts/test_icm20948.py

Prerequisites:
    pip3 install icm20948          # or with --break-system-packages on Ubuntu 24.04
Wiring:
    SDA -> GPIO2 (Pi pin 3)
    SCL -> GPIO3 (Pi pin 5)
    VCC -> 3.3V,  GND -> GND
    I2C address: 0x68 (default, AD0 low)
"""
import sys
import time

try:
    from icm20948 import ICM20948
except ImportError:
    print("ERROR: icm20948 library not found.")
    print("Install with:  pip3 install icm20948")
    print("  (on Ubuntu 24.04 you may need --break-system-packages)")
    sys.exit(1)

print("Initialising ICM20948 on I2C bus 1, address 0x68 ...")
try:
    imu = ICM20948()
    print("OK - device found.\n")
except Exception as e:
    print(f"ERROR: {e}")
    print("Check i2cdetect -y 1 and wiring.")
    sys.exit(1)

ACCEL_G   = "g"
GYRO_UNIT = "deg/s"
MAG_UNIT  = "uT"

print("Reading sensor data — Ctrl+C to stop.\n")
try:
    while True:
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
        mx, my, mz = imu.read_magnetometer_data()

        print("\033[H\033[J", end="")   # clear screen
        print("=== ICM20948  (Ctrl+C to stop) ===\n")
        print(f"Accelerometer ({ACCEL_G})")
        print(f"  X: {ax:+8.4f}   Y: {ay:+8.4f}   Z: {az:+8.4f}")
        print(f"\nGyroscope ({GYRO_UNIT})")
        print(f"  X: {gx:+8.3f}   Y: {gy:+8.3f}   Z: {gz:+8.3f}")
        print(f"\nMagnetometer ({MAG_UNIT})")
        print(f"  X: {mx:+8.2f}   Y: {my:+8.2f}   Z: {mz:+8.2f}")

        # Sanity checks — flag obvious problems
        accel_mag = (ax**2 + ay**2 + az**2) ** 0.5
        print(f"\n  |accel| = {accel_mag:.4f} g  ", end="")
        if 0.9 < accel_mag < 1.1:
            print("(OK — near 1g as expected at rest)")
        else:
            print("(WARNING — expected ~1g at rest)")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n\nDone.")
