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

    return LaunchDescription([

        # Step 2: After 10 seconds, launch the LIDAR
        TimerAction(
            period=0.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch)
                )
            ]
        )
        
        
    ])
