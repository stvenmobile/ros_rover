import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    package_name = 'ros_rover'
    
    # Path to the URDF file
    urdf_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'viam_rover.urdf')
    
    with open(urdf_file_path, 'r') as infp:
        robot_description_config = infp.read()
    
    # 1. Robot State Publisher
    # This node takes the URDF and publishes it to the /robot_description topic
    # It also handles the static transforms between robot links
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False
        }]
    )

    # 2. Joint State Publisher
    # FIX: We pass the URDF directly as a parameter here.
    # This prevents the node from hanging while waiting for a DDS discovery handshake.
    jsp_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description_config,
                    'use_sim_time': False
                }]
            )
        ]
    )
    
    return LaunchDescription([
        rsp_node,
        jsp_node
    ])
