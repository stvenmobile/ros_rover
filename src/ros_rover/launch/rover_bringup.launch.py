from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Hardware Bridge Node
        Node(
            package='ros_rover',
            executable='viam_driver',
            name='viam_hardware_bridge',
            output='screen',
        ),

        # Static Transform for Lidar (Base to Laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
        )
    ])
