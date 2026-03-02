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
    
    # 1. Robot State Publisher
    # Frequency set to 30Hz for smoother transforms in RViz
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False,
            'publish_frequency': 30.0
        }]
    )

    # 2. Joint State Publisher
    # Passing the URDF directly to the 'robot_description' parameter
    # This is the most reliable way to bypass discovery hangs
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        rsp_node,
        jsp_node
    ])
