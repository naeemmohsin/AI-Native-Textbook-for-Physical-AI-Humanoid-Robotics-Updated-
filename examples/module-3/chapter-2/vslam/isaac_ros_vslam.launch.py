#!/usr/bin/env python3
"""
Isaac ROS Visual SLAM Launch File

This launch file configures and starts the cuVSLAM node for visual odometry
and mapping with stereo cameras.

Usage:
    ros2 launch isaac_ros_vslam isaac_ros_vslam.launch.py

Requirements:
    - Isaac ROS Visual SLAM package installed
    - Stereo camera providing left/right images
    - Optional: IMU for sensor fusion
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for Isaac ROS Visual SLAM."""

    # Declare launch arguments
    enable_slam_viz_arg = DeclareLaunchArgument(
        'enable_slam_viz',
        default_value='true',
        description='Enable SLAM visualization (point clouds, landmarks)'
    )

    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Enable IMU fusion for improved accuracy'
    )

    # Get launch configurations
    enable_slam_viz = LaunchConfiguration('enable_slam_viz')
    enable_imu = LaunchConfiguration('enable_imu')

    # Visual SLAM node configuration
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        output='screen',
        parameters=[{
            # SLAM mode configuration
            'enable_localization_n_mapping': True,
            'enable_slam_visualization': enable_slam_viz,
            'enable_observations': True,

            # Image processing
            'denoise_input_images': True,
            'rectified_images': True,

            # Ground constraints (disable for humanoid robots)
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,

            # IMU fusion parameters
            'enable_imu_fusion': enable_imu,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,

            # Timing and synchronization
            'image_jitter_threshold_ms': 35.0,

            # Frame IDs
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }],
        remappings=[
            # Stereo camera input (remap to your camera topics)
            ('stereo_camera/left/image', '/camera/left/image_raw'),
            ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
            ('stereo_camera/right/image', '/camera/right/image_raw'),
            ('stereo_camera/right/camera_info', '/camera/right/camera_info'),
            # IMU input (optional)
            ('visual_slam/imu', '/imu/data'),
        ]
    )

    # Static transform: odom to base_link (initial transform)
    # This is overwritten by visual_slam once tracking starts
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_static',
        arguments=[
            '0', '0', '0',  # translation x, y, z
            '0', '0', '0',  # rotation (roll, pitch, yaw)
            'odom', 'base_link'
        ]
    )

    # Static transform: base_link to camera_link
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_static',
        arguments=[
            '0.1', '0', '0.5',  # Camera is 10cm forward, 50cm up from base
            '0', '0', '0',
            'base_link', 'camera_link'
        ]
    )

    return LaunchDescription([
        enable_slam_viz_arg,
        enable_imu_arg,
        visual_slam_node,
        odom_to_base_tf,
        base_to_camera_tf,
    ])
