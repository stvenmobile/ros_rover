#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import lgpio
import math
import time

# --- VIAM PINOUT MAPPING (BCM) ---
# Left Motor
L_ENA = 22  # PWM Speed
L_IN1 = 17  # Direction 1
L_IN2 = 27  # Direction 2
L_ENC = 19  # Encoder Input (Pulses)

# Right Motor
R_ENB = 25  # PWM Speed
R_IN3 = 23  # Direction 1
R_IN4 = 24  # Direction 2
R_ENC = 26  # Encoder Input (Pulses)

# Physical Constants for VIAM Rover 1
WHEEL_RADIUS = 0.0325      # 65mm diameter
WHEEL_SEPARATION = 0.160   # Distance between wheel centers
TICKS_PER_REV = 40         # Encoder resolution (adjust if needed)
PWM_FREQ = 1000            # 1kHz for smooth motor operation

class ViamHardwareBridge(Node):
    def __init__(self):
        super().__init__('viam_hardware_bridge')
        self.get_logger().info("VIAM Bridge Active: Broadcasting Odom & JointStates")

        # Initialize lgpio
        try:
            self.h = lgpio.gpiochip_open(0)
        except Exception as e:
            self.get_logger().error(f"GPIO Error: {e}")
            return

        # Setup Motor Control Pins
        for pin in [L_ENA, L_IN1, L_IN2, R_ENB, R_IN3, R_IN4]:
            lgpio.gpio_claim_output(self.h, pin)
            lgpio.gpio_write(self.h, pin, 0)
        
        # Setup Encoder Pins
        lgpio.gpio_claim_input(self.h, L_ENC)
        lgpio.gpio_claim_input(self.h, R_ENC)

        # State Variables
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        
        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # Interrupts for high-speed encoder tracking
        lgpio.gpio_claim_alert(self.h, L_ENC, lgpio.BOTH_EDGES)
        lgpio.gpio_claim_alert(self.h, R_ENC, lgpio.BOTH_EDGES)
        lgpio.callback(self.h, L_ENC, lgpio.BOTH_EDGES, self._l_enc_cb)
        lgpio.callback(self.h, R_ENC, lgpio.BOTH_EDGES, self._r_enc_cb)

        # Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 20Hz Control/Odometry Loop (50ms)
        self.timer = self.create_timer(0.05, self.update_loop)

    def _l_enc_cb(self, chip, gpio, level, tick):
        self.left_ticks += 1

    def _r_enc_cb(self, chip, gpio, level, tick):
        self.right_ticks += 1

    def cmd_vel_cb(self, msg):
        """Processes Twist messages and drives motors"""
        self.last_cmd_time = self.get_clock().now()
        v = msg.linear.x
        w = msg.angular.z
        
        # Differential Drive Kinematics
        left_v = v - (w * WHEEL_SEPARATION / 2.0)
        right_v = v + (w * WHEEL_SEPARATION / 2.0)

        self._set_motor(left_v, L_ENA, L_IN1, L_IN2)
        self._set_motor(right_v, R_ENB, R_IN3, R_IN4)

    def _set_motor(self, speed, ena, in1, in2):
        """Maps speed (-1.0 to 1.0) to PWM and Direction pins"""
        if abs(speed) < 0.01:
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 0)
            lgpio.tx_pwm(self.h, ena, PWM_FREQ, 0)
            return

        duty = min(abs(speed) * 100.0, 100.0)
        
        if speed > 0:
            lgpio.gpio_write(self.h, in1, 1)
            lgpio.gpio_write(self.h, in2, 0)
        else:
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 1)

        lgpio.tx_pwm(self.h, ena, PWM_FREQ, duty)

    def update_loop(self):
        now = self.get_clock().now()
        
        # 0.5s Watchdog: Stop if no commands received
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self._set_motor(0, L_ENA, L_IN1, L_IN2)
            self._set_motor(0, R_ENB, R_IN3, R_IN4)

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # Calculate Distance Traveled per Wheel
        dist_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
        d_left = (self.left_ticks - self.prev_left_ticks) * dist_per_tick
        d_right = (self.right_ticks - self.prev_right_ticks) * dist_per_tick
        
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # Pose Integration (Odometry)
        d = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / WHEEL_SEPARATION

        self.x += d * math.cos(self.th + dth/2.0)
        self.y += d * math.sin(self.th + dth/2.0)
        self.th += dth

        # 1. Publish Joint States (Wheel rotation for URDF)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        # Position in Radians
        l_rad = (self.left_ticks / TICKS_PER_REV) * 2 * math.pi
        r_rad = (self.right_ticks / TICKS_PER_REV) * 2 * math.pi
        js.position = [l_rad, r_rad]
        self.joint_pub.publish(js)

        # 2. Publish Odometry and TF Transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        self.last_time = now

def main():
    rclpy.init()
    node = ViamHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
