from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='feedback_control', 
            executable='mission_manager',  # Ejecutable del nodo que maneja las dos poses
            name='mission_manager',
            output='screen'
        ),

        Node(
            package='feedback_control',  
            executable='a_star_planner',  # Ejecutable del planner
            name='a_star_planner',
            output='screen'
        ),
        
        Node(
            package='feedback_control',
            executable='splines',
            name='splines',
            output='screen'
        ),

        Node(
            package='feedback_control',  
            executable='control_trajectory',  # Ejecutable del nodo de control
            name='control_trajectory',
            output='screen'
        ),
    ])
