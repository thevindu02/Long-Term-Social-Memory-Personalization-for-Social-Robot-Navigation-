import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
    
    world_path = PathJoinSubstitution([pkg_social_nav_env, 'worlds', world])

    from launch.conditions import IfCondition, UnlessCondition
    
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

    return LaunchDescription([
        declare_headless_arg,
        declare_world_arg,
        gz_sim_headless,
        gz_sim_gui
    ])
