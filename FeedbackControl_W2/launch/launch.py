from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('feedback_control'),
                                    'config',
                                    'params.yaml')
    return LaunchDescription([
        Node(
            package='yahboomcar_bringup',
            executable='Mcnamu_driver',
            name='Mcnamu_driver',
            output='screen'
        ),
        Node(
            package='feedback_control',
            executable='position_node',
            name='position_node',
            output='screen'
        ),
        Node(
            package='feedback_control',
            executable='qd_node',
            name='qd_node',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='feedback_control',
            executable='kalman_node',
            name='kalman_node',
            output='screen',
        ),
        Node(
            package='feedback_control',
            executable='control_node',
            name='control_node',
            output='screen',
        ),
        
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot_kalman',
            output='screen',
            arguments=['/pose_kalman']
        ),
    ])
