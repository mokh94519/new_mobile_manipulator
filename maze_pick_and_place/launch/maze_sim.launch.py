#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_pick_and_place')
    world_path = os.path.join(pkg_share, 'worlds', 'maze_world.sdf')
    bridge_yaml = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    # Path to the robot xacro in workspace
    robot_xacro = os.path.join('/workspace', 'mobile_manipulator.urdf.xacro')

    robot_name = 'mobile_manipulator'

    # Spawn the world with gz sim
    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r', world_path,
        ],
        output='screen'
    )

    # Process the xacro to SDF (gz sim prefers SDF)
    # We'll use xacro -> urdf -> sdformat via gz command
    gen_sdf = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f"xacro {robot_xacro} > /tmp/{robot_name}.urdf && gz sdf -p /tmp/{robot_name}.urdf > /tmp/{robot_name}.sdf"
        ],
        output='screen'
    )

    # Spawn the robot into the running sim
    spawn_robot = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/maze_world/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'sdf_filename: "/tmp/{robot_name}.sdf", name: "{robot_name}", pose: {{position: {{x: -2.0, y: -1.8, z: 0.0}}}}'
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['-p', bridge_yaml],
        output='screen'
    )

    # IK node
    ik_node = Node(
        package='maze_pick_and_place',
        executable='pick_and_place_node',
        output='screen'
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    return LaunchDescription([
        gz_sim,
        gen_sdf,
        delayed_spawn,
        bridge,
        ik_node,
    ])
