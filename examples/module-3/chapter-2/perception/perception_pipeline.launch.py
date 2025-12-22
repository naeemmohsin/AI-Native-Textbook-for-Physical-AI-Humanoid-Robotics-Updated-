#!/usr/bin/env python3
"""
Isaac ROS Perception Pipeline Launch File

This launch file sets up a GPU-accelerated image processing pipeline
including rectification, resizing, and format conversion.

Usage:
    ros2 launch perception_pipeline perception_pipeline.launch.py

Requirements:
    - Isaac ROS Image Pipeline package installed
    - Camera publishing raw images
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the perception pipeline."""

    # Declare launch arguments
    input_width_arg = DeclareLaunchArgument(
        'input_width',
        default_value='640',
        description='Input image width'
    )

    input_height_arg = DeclareLaunchArgument(
        'input_height',
        default_value='480',
        description='Input image height'
    )

    output_width_arg = DeclareLaunchArgument(
        'output_width',
        default_value='300',
        description='Output image width for DNN inference'
    )

    output_height_arg = DeclareLaunchArgument(
        'output_height',
        default_value='300',
        description='Output image height for DNN inference'
    )

    # Get launch configurations
    input_width = LaunchConfiguration('input_width')
    input_height = LaunchConfiguration('input_height')
    output_width = LaunchConfiguration('output_width')
    output_height = LaunchConfiguration('output_height')

    # Image rectification node (left camera)
    rectify_left_node = Node(
        package='isaac_ros_image_pipeline',
        executable='rectify_node',
        name='rectify_left',
        output='screen',
        parameters=[{
            'output_width': input_width,
            'output_height': input_height,
        }],
        remappings=[
            ('image_raw', '/camera/left/image_raw'),
            ('camera_info', '/camera/left/camera_info'),
            ('image_rect', '/camera/left/image_rect'),
        ]
    )

    # Image rectification node (right camera)
    rectify_right_node = Node(
        package='isaac_ros_image_pipeline',
        executable='rectify_node',
        name='rectify_right',
        output='screen',
        parameters=[{
            'output_width': input_width,
            'output_height': input_height,
        }],
        remappings=[
            ('image_raw', '/camera/right/image_raw'),
            ('camera_info', '/camera/right/camera_info'),
            ('image_rect', '/camera/right/image_rect'),
        ]
    )

    # Resize node for DNN input
    resize_node = Node(
        package='isaac_ros_image_pipeline',
        executable='resize_node',
        name='resize_for_dnn',
        output='screen',
        parameters=[{
            'output_width': output_width,
            'output_height': output_height,
            'keep_aspect_ratio': False,
            'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image', '/camera/left/image_rect'),
            ('camera_info', '/camera/left/camera_info'),
            ('resize/image', '/camera/left/image_resized'),
            ('resize/camera_info', '/camera/left/camera_info_resized'),
        ]
    )

    # Color format conversion (if needed)
    color_convert_node = Node(
        package='isaac_ros_image_pipeline',
        executable='image_format_converter_node',
        name='format_converter',
        output='screen',
        parameters=[{
            'encoding_desired': 'rgb8',
            'image_width': output_width,
            'image_height': output_height,
        }],
        remappings=[
            ('image_raw', '/camera/left/image_resized'),
            ('image', '/camera/left/image_rgb'),
        ]
    )

    # AprilTag detection (fiducial markers)
    apriltag_node = Node(
        package='isaac_ros_apriltag',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[{
            'size': 0.166,  # Tag size in meters
            'max_tags': 16,  # Maximum number of tags to detect
            'tile_size': 4,  # Processing tile size for GPU
        }],
        remappings=[
            ('image', '/camera/left/image_rect'),
            ('camera_info', '/camera/left/camera_info'),
            ('tag_detections', '/apriltag/detections'),
        ]
    )

    return LaunchDescription([
        input_width_arg,
        input_height_arg,
        output_width_arg,
        output_height_arg,
        rectify_left_node,
        rectify_right_node,
        resize_node,
        color_convert_node,
        apriltag_node,
    ])
