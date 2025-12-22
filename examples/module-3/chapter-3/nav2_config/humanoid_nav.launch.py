#!/usr/bin/env python3
"""
Humanoid Robot Navigation Launch File

This launch file brings up the complete Nav2 navigation stack configured
for humanoid robot navigation with localization and path planning.

Usage:
    ros2 launch humanoid_nav humanoid_nav.launch.py \
        map:=/path/to/map.yaml \
        params_file:=/path/to/nav2_params.yaml

Requirements:
    - Nav2 packages installed
    - Map file from SLAM Toolbox
    - Localization source (LiDAR scan)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description for humanoid navigation."""

    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            'nav2_params.yaml'
        ),
        description='Full path to the ROS2 parameters file for Nav2'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the Nav2 stack'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    # Set environment variables for better logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Include Nav2 bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    # RViz2 launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_rviz)
    )

    # Static transform: map to odom (initial, will be updated by AMCL)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'map', 'odom'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Lifecycle manager for Nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
        }]
    )

    return LaunchDescription([
        # Environment
        stdout_linebuf_envvar,

        # Launch arguments
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        autostart_arg,
        use_rviz_arg,

        # Nav2 stack
        nav2_bringup_launch,

        # Visualization
        rviz_launch,

        # Transforms
        map_to_odom_tf,

        # Lifecycle management
        lifecycle_manager,
    ])
