# ros_rover

A lightweight ROS 2 Jazzy workspace for the VIAM Rover 1 chassis, optimized for the Raspberry Pi 4 (1GB RAM).

This project implements a high-performance Python-based hardware bridge that bypasses the resource-heavy ros2_control framework to ensure stable operation on low-memory single-board computers.

## Current Project Status

[x] ROS 2 Jazzy Base Installation on Ubuntu 24.04.

[x] Hardware Bridge for L298N Motor Driver (lgpio based).

[x] Differential Drive Kinematics.

[x] Real-time Odometry (Encoder -> Odom/TF).

[x] Safety Watchdog (0.5s timeout).

[x] Dead-zone compensation for low-speed movement.

[ ] Odometry Tuning & Calibration (Next Step).

[ ] Lidar Integration & SLAM.

##  Hardware Setup

- Chassis: VIAM Rover 1
- Driver: L298N H-Bridge (Standard VIAM Board).
- Motors: 2x DC Brushed Motors with Encoders.
- Power: 4-Cell 18650 Battery Pack (~14.8V).

## Pinout Mapping (BCM)

Component | Pin (GPIO) | Function
------------- | --------- | ----------------------
Left Motor | 22 | PWM Speed (ENA)
Left Motor | 17 / 27 | Direction (IN1 / IN2)
Left Motor | 19| Encoder Input
Right Motor | 25 | PWM Speed (ENB)
Right Motor | 23 / 24 | Direction (IN3 / IN4)
Right Motor | 26 | Encoder Input

---

## Software Requirements

- OS: Ubuntu 24.04 Server (64-bit).
- ROS Version: ROS 2 Jazzy Jalisco (Base).
- Python Library: python3-lgpio.

## Installation & Build

1. Install Dependencies

```bash
sudo apt update && sudo apt install python3-lgpio ros-jazzy-ros-base
```

2. Workspace Setup

```bash
mkdir -p ~/ros_rover/src
cd ~/ros_rover
# Clone your repo into src/
colcon build --symlink-install --parallel-workers 1
source install/setup.bash
```


## Usage

Launching the Robot

To start the hardware bridge, encoder processing, and static transforms:

```bash
ros2 launch ros_rover rover_bringup.launch.py
```


## Manual Testing

Move the robot forward at 0.4 m/s (ensure the robot is on a stand):

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```


## Monitoring Feedback

Check real-time odometry calculated from wheel encoders:

```bash
ros2 topic echo /odom
```


## Optimizations for 1GB RAM

Build workers: limited to 1 (--parallel-workers 1) to prevent OOM (Out of Memory) crashes during compilation.

Symlink Install: Uses --symlink-install to avoid redundant file copying.

Python-based Bridge: Significant memory savings compared to C++ controller managers.

Software PWM: Frequency set to 1000Hz to balance motor smoothness with CPU overhead.

## License

Apache-2.0
