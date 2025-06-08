from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Lidar launch file path
    lidar_launch_path = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py'
    )
    
    ##ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
    yahboom_bringup = os.path.join(
        get_package_share_directory('yahboomcar_bringup'),
        'launch',
        'yahboomcar_bringup_launch.py'
    )
    
    # Define your launch arguments if needed
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default='src/feedback_control/maps/yahboom_map.yaml')

    # Path to the nav2_bringup localization_launch.py
    nav2_bringup_path = FindPackageShare('nav2_bringup')
    localization_launch = PathJoinSubstitution(
        [nav2_bringup_path, 'launch', 'localization_launch.py']
    )
    
    # AMCL custom params 
    amcl_params_file = LaunchConfiguration(
    'amcl_params',
    default=os.path.join('src', 'feedback_control', 'config', 'amcl.yaml'))

    localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            # 'params_file': amcl_params_file
        }.items()
    )

    # Parameters YAML path
    pkg_share = get_package_share_directory('feedback_control')
    parameters = os.path.join(pkg_share, 'config', 'params.yaml')
    
    urdf_path = os.path.join(pkg_share, 'urdf', 'yahboomcar.urdf')
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Nodes
    position_node = Node(
        package='feedback_control',
        executable='position_node',
        name='position_node',
        output='screen'
    )
    
    static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_footprint',
    arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    static_link_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_link_to_lidar_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link']
    )
    

    static_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )


    static_odom_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    

    qd_node_timer = Node(
                package='feedback_control',
                executable='qd_node',
                name='qd_node',
                output='screen',
                parameters=[parameters]
    )


    kalman_node_timer = Node(
                package='feedback_control',
                executable='kalman_node',
                name='kalman_node',
                output='screen',
            )
    
    odo_tf = Node(
            package='feedback_control',
            executable='odom_tf',
            name='odom_tf',
            output='screen'
        )
    
    pose_slam = Node(
            package='feedback_control',
            executable='pose_slam_publisher',
            name='pose_slam_publisher',
            output='screen'
        )
    
    planner = Node(
            package='feedback_control',
            executable='a_star_planner',
            name='a_star_planner',
            output='screen'
        )
    
    pose_example = Node(                ##VERIFY IF 0,0 IS SET CORRECTLY IN MAP
            package='feedback_control',
            executable='pose',
            name='pose',
            output='screen'
        )

    description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,
                    'use_sim_time': True
                    }],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yahboom_bringup)
        ),
        # description,
        # position_node,
        localization_launch_include,
        # qd_node_timer,
        # pose_slam, 
        # planner
        # pose_example,  
        # kalman_node_timer,
        # static_tf,
        # static_link_lidar,
        # static_link,
        # static_odom_map,
        # odo_tf
    ])
    
    
    
    
    ##ESTE LAUNCH VA A QUEAR -> LIDAR+BRINGUP+LOCALIZATION
    
    ##EL OTRO LAUNCH CORRERÁ -> ADMIN DE COMPORTAMIENTO+SPLINES+CONTROL
    
    ##COMPUTADORA -> POSE_SLAM+PLANNER 
