from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get paths to the launch files
    lidar_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
    )

    description_launch = os.path.join(
        get_package_share_directory('yahboomcar_description'),
        'launch',
        'description_launch.py'
    )

    return LaunchDescription([

        # Step 1: Run the Mcnamu_driver node immediately
        ExecuteProcess(
            cmd=['ros2', 'run', 'yahboomcar_bringup', 'Mcnamu_driver'],
            output='screen'
        ),

        # Step 2: After 10 seconds, launch the LIDAR
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch)
                )
            ]
        ),

        # Step 3: After 20 seconds, launch the robot description
        TimerAction(
            period=20.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(description_launch)
                )
            ]
        )
    ])
