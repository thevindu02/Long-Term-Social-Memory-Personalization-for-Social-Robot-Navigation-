import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                            DeclareLaunchArgument, AppendEnvironmentVariable,
                            SetEnvironmentVariable, LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, PythonExpression, LaunchConfiguration, EnvironmentVariable)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    # World generation parameters
    environment_name = LaunchConfiguration('environment_name')
    configuration_file = LaunchConfiguration('configuration_file')
    
    declare_arg_world = DeclareLaunchArgument(
        'environment_name', default_value='proxy_map',
        description='Specify world file name (without .sdf)'
    )
    declare_agents_file = DeclareLaunchArgument(
        'configuration_file', default_value='proxy_agents.yaml',
        description='Specify configuration file name in the config directory'
    )

    pkg_social_nav_env = FindPackageShare('social_nav_env')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    map_path = PathJoinSubstitution([pkg_social_nav_env, 'maps', 'proxy_map.yaml'])

    # world base file (read from social_nav_env)
    world_file = PathJoinSubstitution([
        pkg_social_nav_env,
        'worlds',
        PythonExpression(["'", environment_name, ".sdf'"])
    ])

    # agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_fortress_wrapper'),
        'scenarios',
        configuration_file
    ])

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
    )

    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_fortress_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[
            {'base_world': world_file},
            {'use_gazebo_obs': True},
            {'update_rate': 1000.0},
            {'robot_name': 'simple_robot'},
            {'global_frame_to_publish': 'map'},
            {'use_navgoal_to_start': False},
            {'ignore_models': 'ground_plane sunVisual sun point_lightVisual point_light'},
            {'plugin_position': 0}
        ]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )

    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_fortress_wrapper'),
        'worlds',
    ])
    
    set_env_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''), ':', my_gazebo_models]
    )

    generated_world = PathJoinSubstitution([
        pkg_social_nav_env,
        'worlds',
        'generatedWorld.sdf'
    ])

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s --record-resources --record-path /home/rosdev/gazebo_workspace/sim_record ', generated_world], 'on_exit_shutdown': 'true'}.items()
    )


    # Do not launch Gazebo until the world has been generated
    ordered_launch_event2 = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[gzserver_cmd],
                )
            ]
        )
    )

    # hunav_manager node
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
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
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    # Nav2 Bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        set_env_gz_resources,
        declare_arg_world,
        declare_agents_file,
        hunav_loader_node,
        ordered_launch_event,
        ordered_launch_event2,
        hunav_manager_node,
        bridge,
        static_tf_lidar,
        static_tf_footprint,
        nav2
    ])
