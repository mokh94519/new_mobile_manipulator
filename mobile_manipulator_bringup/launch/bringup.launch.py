from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'worlds', 'maze.world'
    ]))

    # Gazebo Harmonic (gz sim)
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # Xacro to URDF
    xacro_file = PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'urdf', 'mobile_manipulator.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    # Spawn the robot in Gazebo using ros_gz_sim create
    spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-string', robot_description, '-name', 'mobile_manipulator', '-allow_renaming', 'true'],
        output='screen'
    )

    # ros_gz_bridge from YAML (pass via config_file param)
    bridge_yaml = PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'config', 'bridge_topics.yaml'
    ])

    parameter_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_yaml}]
    )

    # SLAM Toolbox
    slam_params = PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'config', 'slam_toolbox.yaml'
    ])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node', output='screen',
        parameters=[slam_params]
    )

    # Nav2 bringup (include launch)
    nav2_params = PathJoinSubstitution([
        FindPackageShare('mobile_manipulator_bringup'), 'config', 'nav2.yaml'
    ])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'slam': 'True'
        }.items()
    )

    # Nodes
    explore_and_collect = Node(
        package='mobile_manipulator_bringup', executable='explore_and_collect',
        output='screen'
    )
    arm_controller = Node(
        package='mobile_manipulator_bringup', executable='arm_controller',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world),
        gz,
        robot_state_publisher,
        spawn,
        parameter_bridge,
        slam,
        nav2,
        arm_controller,
        explore_and_collect,
    ])
