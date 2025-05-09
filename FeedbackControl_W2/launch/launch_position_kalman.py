from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('feeedback_control')
    parameters = os.path.join(pkg_share, 'config', 'params.yaml')
    
    position = Node(
        package='feedback_control',
        executable='position_node',
        name='position_node',
        output='screen'
    )
    spawn = TimerAction(
        period=10.0,  # seconds
        actions=[
            Node(
                package='feedback_control',
                executable='qd_node',
                name='qd_node',
                output='screen',
                parameters=[parameters]
            )
        ])
    spawn_2 = TimerAction(
        period=20.0,  # seconds
        actions=[Node(
                package='feedback_control',
                executable='kalman_node',
                name='kalman_node',
                output='screen',
                )
        ]
            
    )
    
    return LaunchDescription([position, spawn, spawn_2])
