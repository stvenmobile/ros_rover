import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ros_rover')

    # Robot State Publisher (URDF -> TF tree)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')
        )
    )

    # Hardware Bridge: drives motors, reads encoders, publishes odom + joint_states
    viam_driver = Node(
        package='ros_rover',
        executable='viam_driver',
        name='viam_hardware_bridge',
        output='screen',
    )

    # NOTE: laser_frame TF is defined in the URDF (chassis -> laser_frame).
    # Do NOT add a separate static_transform_publisher for laser_frame here,
    # as that would create a conflicting duplicate in the TF tree.

    return LaunchDescription([
        rsp,
        viam_driver,
    ])
