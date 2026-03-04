"""
Display launch: use this on a desktop (no hardware) to view the URDF in RViz2.

    ros2 launch ros_rover display.launch.py

Launches:
  - robot_state_publisher  (URDF -> TF)
  - joint_state_publisher_gui  (GUI sliders to manually drive joints)
  - rviz2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ros_rover')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')
        )
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        rsp,
        joint_state_publisher_gui,
        rviz2,
    ])
