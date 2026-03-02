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

# --- VIAM PINOUT MAPPING (Based on viam2.png) ---
# GPIO BCM Numbers
L_ENA = 22  # Physical Pin 15
L_IN1 = 17  # Physical Pin 11
L_IN2 = 27  # Physical Pin 13
L_ENC = 19  # Physical Pin 35

R_ENB = 25  # Physical Pin 22
R_IN3 = 23  # Physical Pin 16
R_IN4 = 24  # Physical Pin 18
R_ENC = 26  # Physical Pin 37

# Physical Constants (Adjust to your specific Rover 1 wheels)
WHEEL_RADIUS = 0.0325  # 65mm diameter
WHEEL_SEPARATION = 0.160 # 160mm track width
TICKS_PER_REV = 40     # Update based on your encoder disk resolution

class ViamHardwareBridge(Node):
    def __init__(self):
        super().__init__('viam_hardware_bridge')
        self.get_logger().info("Starting VIAM Bridge (Optimized for 1GB RAM)")

        # Initialize lgpio
        try:
            self.h = lgpio.gpiochip_open(0)
        except Exception as e:
            self.get_logger().error(f"Failed to open GPIO chip: {e}")
            return

        # Claim Output Pins
        for pin in [L_ENA, L_IN1, L_IN2, R_ENB, R_IN3, R_IN4]:
            lgpio.gpio_claim_output(self.h, pin)

        # Claim Encoder Pins as Inputs
        lgpio.gpio_claim_input(self.h, L_ENC)
        lgpio.gpio_claim_input(self.h, R_ENC)

        # Set Silent 20kHz PWM
        lgpio.tx_pwm(self.h, L_ENA, 20000, 0)
        lgpio.tx_pwm(self.h, R_ENB, 20000, 0)

        # Encoder Tracking State
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        
        # Odom State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Setup High-Accuracy Interrupts
        lgpio.gpio_claim_alert(self.h, L_ENC, lgpio.BOTH_EDGES)
        lgpio.gpio_claim_alert(self.h, R_ENC, lgpio.BOTH_EDGES)
        self.cb_l = lgpio.callback(self.h, L_ENC, lgpio.BOTH_EDGES, self._l_enc_cb)
        self.cb_r = lgpio.callback(self.h, R_ENC, lgpio.BOTH_EDGES, self._r_enc_cb)

        # ROS Interface
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Main Update Loop (20Hz)
        self.timer = self.create_timer(0.05, self.update_loop)

    def _l_enc_cb(self, chip, gpio, level, tick):
        self.left_ticks += 1

    def _r_enc_cb(self, chip, gpio, level, tick):
        self.right_ticks += 1

    def cmd_vel_cb(self, msg):
        """Differential Drive Kinematics"""
        v = msg.linear.x
        w = msg.angular.z
        
        left_v = v - (w * WHEEL_SEPARATION / 2.0)
        right_v = v + (w * WHEEL_SEPARATION / 2.0)

        # Apply speeds to L298N logic
        self._set_motor(left_v, L_ENA, L_IN1, L_IN2)
        self._set_motor(right_v, R_ENB, R_IN3, R_IN4)

    def _set_motor(self, speed, ena, in1, in2):
        # 1.0 m/s mapped to 100% Duty Cycle (Adjust scaling as needed)
        duty = min(abs(speed) * 100, 100)
        
        if speed > 0.01:
            lgpio.gpio_write(self.h, in1, 1)
            lgpio.gpio_write(self.h, in2, 0)
        elif speed < -0.01:
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 1)
        else:
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 0)
            duty = 0

        lgpio.tx_pwm(self.h, ena, 20000, duty)

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # Distance calculation based on ticks
        dist_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
        d_left = (self.left_ticks - self.prev_left_ticks) * dist_per_tick
        d_right = (self.right_ticks - self.prev_right_ticks) * dist_per_tick
        
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # Pose integration
        d = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / WHEEL_SEPARATION

        self.x += d * math.cos(self.th + dth/2.0)
        self.y += d * math.sin(self.th + dth/2.0)
        self.th += dth

        # Broadcast Odom
        self.publish_odom(now)
        self.last_time = now

    def publish_odom(self, now):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ViamHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        # Emergency stop on exit
        lgpio.tx_pwm(node.h, L_ENA, 20000, 0)
        lgpio.tx_pwm(node.h, R_ENB, 20000, 0)
        lgpio.gpiochip_close(node.h)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
