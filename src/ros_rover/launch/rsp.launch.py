import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ros_rover'
    
    # Path to the URDF file
    urdf_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'viam_rover.urdf')
    
    with open(urdf_file_path, 'r') as infp:
        robot_description_config = infp.read()
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        # Joint State Publisher (Fixed joints don't need this, but continuous wheels do)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        )
    ])
