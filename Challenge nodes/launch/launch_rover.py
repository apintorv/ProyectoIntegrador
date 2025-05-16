from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lidar launch file path
    lidar_launch_path = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
    )

    # Parameters YAML path
    pkg_share = get_package_share_directory('feedback_control')
    parameters = os.path.join(pkg_share, 'config', 'params.yaml')

    # Nodes
    position_node = Node(
        package='feedback_control',
        executable='position_node',
        name='position_node',
        output='screen'
    )
    

    static_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_link']
    )


    static_odom_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    qd_node_timer = TimerAction(    ##this node wont be used in the real challenge
        period=10.0,
        actions=[
            Node(
                package='feedback_control',
                executable='qd_node',
                name='qd_node',
                output='screen',
                parameters=[parameters]
            )
        ]
    )

    kalman_node_timer = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='feedback_control',
                executable='kalman_node',
                name='kalman_node',
                output='screen',
            )
        ]
    )

    # a_star_node = Node(
    #     package='feedback_control',
    #     executable='a_star_planner',
    #     name='a_star_planner',
    #     output='screen'
    # )

    # control_trajectory_node = Node(
    #     package='feedback_control',
    #     executable='control_trajectory',
    #     name='control_trajectory',
    #     output='screen'
    #)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),
        position_node,
        qd_node_timer,
        kalman_node_timer,
        static_link,
        static_odom_map
        # a_star_node,
        # control_trajectory_node,
    ])
