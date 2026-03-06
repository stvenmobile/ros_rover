#!/usr/bin/env python3
"""
ICM20948 ROS2 driver node.

Publishes:
    /imu/data  (sensor_msgs/Imu)           — accel + gyro at 50 Hz
    /imu/mag   (sensor_msgs/MagneticField) — magnetometer at 20 Hz

Frame: imu_link (defined in URDF, fixed to chassis centre-top)

Orientation is NOT computed here (covariance[0] = -1).
Downstream fusion (robot_localization EKF) will derive orientation
from accel + gyro + mag and produce a corrected odometry estimate.

Unit conversions applied:
    Accelerometer : g         -> m/s²  (multiply by 9.80665)
    Gyroscope     : deg/s     -> rad/s (multiply by pi/180)
    Magnetometer  : microtesla -> tesla (multiply by 1e-6)
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

try:
    from icm20948 import ICM20948
    _LIB_OK = True
except ImportError:
    _LIB_OK = False

GRAVITY = 9.80665  # m/s²

# Diagonal covariance values (variance per axis).
# These are conservative starting estimates — tune after calibration.
ACCEL_VAR = 0.0016   # (m/s²)²  ≈ (0.04 m/s²)² noise floor
GYRO_VAR  = 3.0e-5   # (rad/s)² ≈ (0.3 deg/s)² noise floor
MAG_VAR   = 1.0e-10  # T²


class ICM20948Driver(Node):
    def __init__(self):
        super().__init__('icm20948_driver')

        if not _LIB_OK:
            self.get_logger().fatal(
                "icm20948 Python library not found. "
                "Install with: pip3 install icm20948"
            )
            return

        try:
            self.imu = ICM20948()
            self.get_logger().info("ICM20948 initialised at I2C bus 1, address 0x68")
        except Exception as e:
            self.get_logger().fatal(f"ICM20948 init failed: {e}  — check wiring/I2C")
            return

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        self.create_timer(0.02,  self._publish_imu)   # 50 Hz
        self.create_timer(0.05,  self._publish_mag)   # 20 Hz

    # ------------------------------------------------------------------
    def _publish_imu(self):
        try:
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()
        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}", throttle_duration_sec=5.0)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Accelerometer: g -> m/s²
        msg.linear_acceleration.x = ax * GRAVITY
        msg.linear_acceleration.y = ay * GRAVITY
        msg.linear_acceleration.z = az * GRAVITY
        cov = msg.linear_acceleration_covariance
        cov[0] = cov[4] = cov[8] = ACCEL_VAR

        # Gyroscope: deg/s -> rad/s
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)
        cov = msg.angular_velocity_covariance
        cov[0] = cov[4] = cov[8] = GYRO_VAR

        # Orientation not provided by this node; EKF will compute it
        msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(msg)

    # ------------------------------------------------------------------
    def _publish_mag(self):
        try:
            mx, my, mz = self.imu.read_magnetometer_data()
        except Exception as e:
            self.get_logger().warn(f"Mag read error: {e}", throttle_duration_sec=5.0)
            return

        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Magnetometer: microtesla -> tesla
        msg.magnetic_field.x = mx * 1e-6
        msg.magnetic_field.y = my * 1e-6
        msg.magnetic_field.z = mz * 1e-6
        cov = msg.magnetic_field_covariance
        cov[0] = cov[4] = cov[8] = MAG_VAR

        self.mag_pub.publish(msg)


def main():
    rclpy.init()
    node = ICM20948Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
