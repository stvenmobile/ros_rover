#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import lgpio
import math
import time

# --- VIAM PINOUT MAPPING (Based on viam2.png) ---
L_ENA = 22  # PWM Speed
L_IN1 = 17  # Direction 1
L_IN2 = 27  # Direction 2
L_ENC = 19  # Encoder

R_ENB = 25  # PWM Speed
R_IN3 = 23  # Direction 1
R_IN4 = 24  # Direction 2
R_ENC = 26  # Encoder

# PWM Settings
PWM_FREQ = 1000 
MIN_DUTY = 35.0  # Minimum duty cycle to overcome static friction (Adjust this!)
MAX_SPEED = 1.0  # Assumed max speed in m/s for 100% duty cycle

# Physical Constants for VIAM Rover 1
WHEEL_RADIUS = 0.0325  
WHEEL_SEPARATION = 0.160 
TICKS_PER_REV = 40     

class ViamHardwareBridge(Node):
    def __init__(self):
        super().__init__('viam_hardware_bridge')
        self.get_logger().info("Starting VIAM Bridge with Dead-Zone Compensation")

        # Initialize lgpio
        try:
            self.h = lgpio.gpiochip_open(0)
        except Exception as e:
            self.get_logger().error(f"Failed to open GPIO chip: {e}")
            return

        # Setup Pins
        for pin in [L_ENA, L_IN1, L_IN2, R_ENB, R_IN3, R_IN4]:
            lgpio.gpio_claim_output(self.h, pin)
        lgpio.gpio_claim_input(self.h, L_ENC)
        lgpio.gpio_claim_input(self.h, R_ENC)

        # State Variables
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now() # Watchdog timer

        # Interrupts
        lgpio.gpio_claim_alert(self.h, L_ENC, lgpio.BOTH_EDGES)
        lgpio.gpio_claim_alert(self.h, R_ENC, lgpio.BOTH_EDGES)
        self.cb_l = lgpio.callback(self.h, L_ENC, lgpio.BOTH_EDGES, self._l_enc_cb)
        self.cb_r = lgpio.callback(self.h, R_ENC, lgpio.BOTH_EDGES, self._r_enc_cb)

        # ROS
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_loop)

    def _l_enc_cb(self, chip, gpio, level, tick):
        self.left_ticks += 1

    def _r_enc_cb(self, chip, gpio, level, tick):
        self.right_ticks += 1

    def cmd_vel_cb(self, msg):
        self.last_cmd_time = self.get_clock().now() # Reset watchdog
        v = msg.linear.x
        w = msg.angular.z
        
        left_v = v - (w * WHEEL_SEPARATION / 2.0)
        right_v = v + (w * WHEEL_SEPARATION / 2.0)

        self._set_motor(left_v, L_ENA, L_IN1, L_IN2)
        self._set_motor(right_v, R_ENB, R_IN3, R_IN4)

    def _set_motor(self, speed, ena, in1, in2):
        if abs(speed) < 0.01:
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 0)
            lgpio.tx_pwm(self.h, ena, PWM_FREQ, 0)
            return

        # Dead-zone compensation mapping
        # Scales 0.0 -> MAX_SPEED to MIN_DUTY -> 100.0
        ratio = min(abs(speed) / MAX_SPEED, 1.0)
        duty = MIN_DUTY + (ratio * (100.0 - MIN_DUTY))
        
        if speed > 0:
            lgpio.gpio_write(self.h, in1, 1)
            lgpio.gpio_write(self.h, in2, 0)
        else:
            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 1)

        lgpio.tx_pwm(self.h, ena, PWM_FREQ, duty)

    def update_loop(self):
        now = self.get_clock().now()
        
        # --- WATCHDOG CHECK ---
        # If no command received for 0.5s, stop motors
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self._set_motor(0, L_ENA, L_IN1, L_IN2)
            self._set_motor(0, R_ENB, R_IN3, R_IN4)

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # Odometry Integration
        dist_per_tick = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
        d_left = (self.left_ticks - self.prev_left_ticks) * dist_per_tick
        d_right = (self.right_ticks - self.prev_right_ticks) * dist_per_tick
        
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        d = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / WHEEL_SEPARATION

        self.x += d * math.cos(self.th + dth/2.0)
        self.y += d * math.sin(self.th + dth/2.0)
        self.th += dth

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
        pass
    finally:
        if hasattr(node, 'h'):
            lgpio.tx_pwm(node.h, L_ENA, PWM_FREQ, 0)
            lgpio.tx_pwm(node.h, R_ENB, PWM_FREQ, 0)
            lgpio.gpiochip_close(node.h)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
