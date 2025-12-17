#!/usr/bin/env python3
"""
Spawn Humanoid Robot in Gazebo

This launch file demonstrates how to:
1. Start Gazebo Sim with a custom world
2. Spawn a URDF robot model into the simulation
3. Bridge topics between Gazebo and ROS 2

Usage:
    ros2 launch humanoid_gazebo spawn_humanoid.launch.py

Requirements:
    - ros-humble-ros-gz (Gazebo Harmonic integration)
    - Humanoid URDF from Module 1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='humanoid_world.sdf')

    # Get package paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to world file (adjust based on your setup)
    world_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'worlds',
        'humanoid_world.sdf'
    )

    # Path to URDF (from Module 1 - adjust path as needed)
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        '..',
        '..',
        '..',
        'module-1',
        'chapter-3',
        'humanoid_arm.urdf'
    )

    # Read URDF content
    # In production, use xacro to process the URDF if needed
    robot_description = ''
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as urdf_file:
            robot_description = urdf_file.read()

    # Start Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot entity in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '1.0',  # Spawn above ground
        ],
        output='screen',
    )

    # Robot State Publisher - publishes TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # Bridge clock from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Bridge joint states from Gazebo to ROS 2
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/humanoid_world/model/humanoid/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='humanoid_world.sdf',
            description='World file to load'
        ),

        # Start Gazebo
        gz_sim,

        # Publish robot description
        robot_state_publisher,

        # Spawn robot after Gazebo is ready
        spawn_robot,

        # Bridge topics
        clock_bridge,
        joint_state_bridge,
    ])
