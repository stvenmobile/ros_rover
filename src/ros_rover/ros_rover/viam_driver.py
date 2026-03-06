#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import lgpio
import math

# --- VIAM PINOUT MAPPING (BCM) ---
L_ENA = 22  # PWM Speed
L_IN1 = 17  # Direction 1
L_IN2 = 27  # Direction 2
L_ENC = 19  # Encoder Input

R_ENB = 25  # PWM Speed
R_IN3 = 23  # Direction 1
R_IN4 = 24  # Direction 2
R_ENC = 26  # Encoder Input

# Physical Constants for VIAM Rover 1
WHEEL_RADIUS     = 0.035   # 70mm diameter (measured)
WHEEL_SEPARATION = 0.230   # centre-to-centre (measured)
TICKS_PER_REV    = 2000    # encoder resolution (measured)
PWM_FREQ         = 1000    # Hz


class ViamHardwareBridge(Node):
    def __init__(self):
        super().__init__('viam_hardware_bridge')
        self.get_logger().info("VIAM Bridge Active: Publishing Odom & JointStates")

        # PID gains — tunable at runtime:
        #   ros2 param set /viam_hardware_bridge pid_kp 2.0
        self.declare_parameter('pid_kp', 1.5)
        self.declare_parameter('pid_ki', 0.8)
        self.declare_parameter('pid_kd', 0.02)

        try:
            self.h = lgpio.gpiochip_open(0)
        except Exception as e:
            self.get_logger().error(f"GPIO Error: {e}")
            return

        # Free any pins left claimed by a previous (crashed) process
        for pin in [L_ENA, L_IN1, L_IN2, R_ENB, R_IN3, R_IN4, L_ENC, R_ENC]:
            try:
                lgpio.gpio_free(self.h, pin)
            except Exception:
                pass

        # Motor output pins
        for pin in [L_ENA, L_IN1, L_IN2, R_ENB, R_IN3, R_IN4]:
            lgpio.gpio_claim_output(self.h, pin)
            lgpio.gpio_write(self.h, pin, 0)

        # Encoder input pins
        lgpio.gpio_claim_input(self.h, L_ENC)
        lgpio.gpio_claim_input(self.h, R_ENC)

        # Encoder tick counters
        self.left_ticks       = 0
        self.right_ticks      = 0
        self.prev_left_ticks  = 0
        self.prev_right_ticks = 0
        self.left_dir         = 1   # +1 forward, -1 reverse
        self.right_dir        = 1

        # Odometry pose
        self.x  = 0.0
        self.y  = 0.0
        self.th = 0.0

        # PID state
        self.left_target  = 0.0   # desired wheel speed (m/s)
        self.right_target = 0.0
        self.left_integral  = 0.0
        self.right_integral = 0.0
        self.left_prev_err  = 0.0
        self.right_prev_err = 0.0

        self.last_time     = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # Encoder interrupts (BOTH_EDGES = 2000 ticks/rev)
        lgpio.gpio_claim_alert(self.h, L_ENC, lgpio.BOTH_EDGES)
        lgpio.gpio_claim_alert(self.h, R_ENC, lgpio.BOTH_EDGES)
        lgpio.callback(self.h, L_ENC, lgpio.BOTH_EDGES, self._l_enc_cb)
        lgpio.callback(self.h, R_ENC, lgpio.BOTH_EDGES, self._r_enc_cb)

        # NOTE: odom->base_link TF is published by the EKF node, not here.
        self.odom_pub  = self.create_publisher(Odometry,   'odom',         10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # 20 Hz control loop
        self.timer = self.create_timer(0.05, self.update_loop)

    # ------------------------------------------------------------------
    # Encoder callbacks
    # ------------------------------------------------------------------
    def _l_enc_cb(self, chip, gpio, level, tick):
        self.left_ticks += self.left_dir

    def _r_enc_cb(self, chip, gpio, level, tick):
        self.right_ticks += self.right_dir

    # ------------------------------------------------------------------
    # cmd_vel callback — sets PID targets, does NOT drive motors directly
    # ------------------------------------------------------------------
    def cmd_vel_cb(self, msg):
        self.last_cmd_time = self.get_clock().now()
        v = msg.linear.x
        w = msg.angular.z

        left_v  = v - (w * WHEEL_SEPARATION / 2.0)
        right_v = v + (w * WHEEL_SEPARATION / 2.0)

        self.left_target  = left_v
        self.right_target = right_v

        # Update encoder direction from commanded speed
        self.left_dir  = 1 if left_v  >= 0 else -1
        self.right_dir = 1 if right_v >= 0 else -1

        # Reset integrators when stopping to prevent windup
        if abs(left_v) < 0.01:
            self.left_integral = 0.0
        if abs(right_v) < 0.01:
            self.right_integral = 0.0

    # ------------------------------------------------------------------
    # Per-wheel PID — returns PWM output in [-1.0, 1.0]
    # ------------------------------------------------------------------
    def _compute_pid(self, target, actual, dt, integral, prev_err):
        Kp = self.get_parameter('pid_kp').value
        Ki = self.get_parameter('pid_ki').value
        Kd = self.get_parameter('pid_kd').value

        error      = target - actual
        integral  += error * dt
        # Anti-windup: clamp so Ki*integral stays within output bounds
        max_i      = 1.0 / Ki if Ki > 1e-6 else 10.0
        integral   = max(-max_i, min(max_i, integral))
        derivative = (error - prev_err) / dt if dt > 1e-6 else 0.0

        output = Kp * error + Ki * integral + Kd * derivative
        output = max(-1.0, min(1.0, output))

        return output, integral, error

    # ------------------------------------------------------------------
    # Low-level motor driver — speed in [-1.0, 1.0] maps to PWM + dir
    # ------------------------------------------------------------------
    def _set_motor(self, speed, ena, in1, in2):
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

    # ------------------------------------------------------------------
    # Main 20 Hz loop
    # ------------------------------------------------------------------
    def update_loop(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Watchdog: zero targets and reset integrators if cmd_vel goes stale
        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self.left_target    = 0.0
            self.right_target   = 0.0
            self.left_integral  = 0.0
            self.right_integral = 0.0

        # --- Encoder deltas → actual wheel speeds ---
        dist_per_tick = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
        d_left  = (self.left_ticks  - self.prev_left_ticks)  * dist_per_tick
        d_right = (self.right_ticks - self.prev_right_ticks) * dist_per_tick
        self.prev_left_ticks  = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        actual_left  = d_left  / dt
        actual_right = d_right / dt

        # --- PID: compute PWM outputs ---
        left_pwm,  self.left_integral,  self.left_prev_err  = self._compute_pid(
            self.left_target,  actual_left,  dt,
            self.left_integral,  self.left_prev_err)

        right_pwm, self.right_integral, self.right_prev_err = self._compute_pid(
            self.right_target, actual_right, dt,
            self.right_integral, self.right_prev_err)

        # Update encoder direction from actual motor command
        self.left_dir  = 1 if left_pwm  >= 0 else -1
        self.right_dir = 1 if right_pwm >= 0 else -1

        self._set_motor(left_pwm,  L_ENA, L_IN1, L_IN2)
        self._set_motor(right_pwm, R_ENB, R_IN3, R_IN4)

        # --- Odometry integration ---
        d   = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / WHEEL_SEPARATION

        self.x  += d * math.cos(self.th + dth / 2.0)
        self.y  += d * math.sin(self.th + dth / 2.0)
        self.th += dth

        vx   = d   / dt
        vyaw = dth / dt

        # --- Publish JointStates ---
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        l_rad = (self.left_ticks  / TICKS_PER_REV) * 2.0 * math.pi
        r_rad = (self.right_ticks / TICKS_PER_REV) * 2.0 * math.pi
        js.position = [l_rad, r_rad]
        self.joint_pub.publish(js)

        # --- Publish Odometry (EKF input) ---
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vyaw

        odom.pose.covariance[0]  = 0.010   # x   (m²)
        odom.pose.covariance[7]  = 0.010   # y   (m²)
        odom.pose.covariance[35] = 0.100   # yaw (rad²)
        odom.twist.covariance[0]  = 0.005  # vx   (m/s)²
        odom.twist.covariance[35] = 0.010  # vyaw (rad/s)²

        self.odom_pub.publish(odom)

        self.last_time = now

    # ------------------------------------------------------------------
    # Clean shutdown
    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            self._set_motor(0, L_ENA, L_IN1, L_IN2)
            self._set_motor(0, R_ENB, R_IN3, R_IN4)
            lgpio.gpiochip_close(self.h)
        except Exception:
            pass
        super().destroy_node()


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
