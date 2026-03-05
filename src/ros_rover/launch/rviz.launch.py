"""
RViz launch for live robot viewing.
Run this on SER5 while rover_bringup.launch.py is running on the Pi.
The Pi already provides RSP, TF, joint_states and odom — do NOT run
robot_state_publisher or joint_state_publisher_gui here.

    ros2 launch ros_rover rviz.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
