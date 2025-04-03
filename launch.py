import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('open_cv_example'),
                                    'config',
                                    'params.yaml')

    control_node = Node(
        package='open_cv_example',
        executable='line_control_listener',
        name = 'Control_Listener',
        parameters=[config]
    )
    
    line_node = Node(
        package='open_cv_example',
        executable='line_follower_talker',
        name = 'line_follower',
    )

    l_d = LaunchDescription([control_node, line_node])
    return l_d