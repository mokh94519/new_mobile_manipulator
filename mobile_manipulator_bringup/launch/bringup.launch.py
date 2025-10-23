#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_default = PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'worlds', 'maze.world'
    ])
    world = LaunchConfiguration('world', default=world_default)

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'urdf', 'mobile_manipulator.urdf.xacro'
    ])

    # Launch Gazebo (gz sim) with the maze world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world], output='screen'
    )

    # Spawn the robot
    # Generate URDF from xacro to a temp file for spawning
    urdf_out = '/tmp/mobile_manipulator.urdf'
    xacro_cmd = [FindExecutable(name='xacro'), urdf_xacro, '-o', urdf_out]
    gen_urdf = ExecuteProcess(cmd=xacro_cmd, output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'mobile_manipulator', '-x', '0', '-y', '0', '-z', '0.1', '-file', urdf_out]
    )

    # Bridge topics using YAML
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['--ros-args', '--params-file', PathJoinSubstitution([
            FindPackageShare('mobile_manipulator_bringup'), 'config', 'ros_gz_bridge.yaml'
        ])]
    )

    # robot_state_publisher for TF tree from URDF
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command([FindExecutable(name='xacro'), ' ', urdf_xacro])},
        ],
    )

    # SLAM Toolbox
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('mobile_manipulator_bringup'), 'config', 'slam_toolbox.yaml'
        ])]
    )

    # Nav2 Bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_bringup'), 'config', 'nav2_params.yaml'
            ])
        }.items()
    )

    # Example exploration and collection nodes
    explore = Node(
        package='mobile_manipulator_bringup', executable='explore_and_collect', output='screen'
    )
    detector = Node(
        package='mobile_manipulator_bringup', executable='object_detector', output='screen'
    )
    picker = Node(
        package='mobile_manipulator_bringup', executable='pick_place', output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_default),
        gz_sim,
        gen_urdf,
        spawn,
        bridge,
        rsp,
        slam,
        nav2,
        explore,
        detector,
        picker,
    ])
