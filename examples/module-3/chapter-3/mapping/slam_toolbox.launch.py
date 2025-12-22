#!/usr/bin/env python3
"""
SLAM Toolbox Launch File for Mapping

This launch file starts SLAM Toolbox in online asynchronous mode
for building 2D occupancy grid maps from LiDAR data.

Usage:
    # Start mapping
    ros2 launch slam_toolbox slam_toolbox.launch.py

    # Save map when done
    ros2 run nav2_map_server map_saver_cli -f my_map

Requirements:
    - slam_toolbox package installed
    - LiDAR publishing to /scan topic
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description for SLAM Toolbox mapping."""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value='',
        description='Full path to SLAM Toolbox parameters file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # SLAM Toolbox parameters (inline if no file provided)
    slam_params = {
        'use_sim_time': use_sim_time,

        # Solver configuration
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        'ceres_loss_function': 'None',

        # Map parameters
        'resolution': 0.05,  # 5cm grid resolution
        'map_update_interval': 5.0,
        'max_laser_range': 12.0,
        'minimum_time_interval': 0.5,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 30.0,

        # Scan matching
        'minimum_travel_distance': 0.5,
        'minimum_travel_heading': 0.5,
        'scan_buffer_size': 10,
        'scan_buffer_maximum_scan_distance': 10.0,
        'link_match_minimum_response_fine': 0.1,
        'link_scan_maximum_distance': 1.5,

        # Loop closure
        'loop_search_maximum_distance': 3.0,
        'do_loop_closing': True,
        'loop_match_minimum_chain_size': 10,
        'loop_match_maximum_variance_coarse': 3.0,
        'loop_match_minimum_response_coarse': 0.35,
        'loop_match_minimum_response_fine': 0.45,

        # Correlation parameters
        'correlation_search_space_dimension': 0.5,
        'correlation_search_space_resolution': 0.01,
        'correlation_search_space_smear_deviation': 0.1,

        # Loop closure correlation
        'loop_search_space_dimension': 8.0,
        'loop_search_space_resolution': 0.05,
        'loop_search_space_smear_deviation': 0.03,

        # Distance variance penalty
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty': 1.0,
        'fine_search_angle_offset': 0.00349,
        'coarse_search_angle_offset': 0.349,
        'coarse_angle_resolution': 0.0349,

        # Performance
        'use_scan_matching': True,
        'use_scan_barycenter': True,
        'transform_publish_period': 0.02,

        # Frame IDs
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_link',
        'scan_topic': '/scan',

        # Mode: mapping (not localization)
        'mode': 'mapping',
    }

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
        remappings=[
            ('/scan', '/scan'),
        ]
    )

    # Map visualization (optional - saves resources if disabled)
    map_visualizer = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
            'frame_id': 'map',
        }]
    )

    # Static TF: odom to base_link (in case odometry is not publishing TF)
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        slam_toolbox_node,
        # Uncomment if needed:
        # map_visualizer,
        # odom_to_base_tf,
    ])
