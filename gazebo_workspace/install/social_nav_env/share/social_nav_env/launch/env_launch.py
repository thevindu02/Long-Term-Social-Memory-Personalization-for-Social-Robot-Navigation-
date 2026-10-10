import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo headless (no GUI)'
    )
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='proxy_map.sdf',
        description='World file to load'
    )

    pkg_social_nav_env = FindPackageShare('social_nav_env')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    world_path = PathJoinSubstitution([pkg_social_nav_env, 'worlds', world])
    map_path = PathJoinSubstitution([pkg_social_nav_env, 'maps', 'proxy_map.yaml'])
    
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-s -r ', world_path]}.items(),
        condition=IfCondition(headless)
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
        condition=UnlessCondition(headless)
    )

    # ROS 2 to Ignition Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    # Static Transform: base_link -> lidar_link
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'simple_robot/lidar_link/lidar'],
        output='screen'
    )

    static_tf_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    # Nav2 Bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_headless_arg,
        declare_world_arg,
        gz_sim_headless,
        gz_sim_gui,
        bridge,
        static_tf_lidar,
        static_tf_footprint,
        nav2
    ])
