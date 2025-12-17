#!/usr/bin/env python3
"""
Sensor Bridge Launch File

Bridges all sensor topics from Gazebo to ROS 2.

This launch file configures ros_gz_bridge to connect:
- LiDAR scan data
- Depth camera images and point clouds
- IMU data

Usage:
    ros2 launch sensor_processing sensor_bridge.launch.py

Requirements:
    - ros-humble-ros-gz-bridge package
    - Gazebo Sim running with sensor-equipped robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Bridge node for all sensors
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # LiDAR - LaserScan message
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

            # Depth Camera - RGB Image
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',

            # Depth Camera - Depth Image
            '/camera/depth@sensor_msgs/msg/Image@gz.msgs.Image',

            # Depth Camera - Point Cloud
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            # Depth Camera - Camera Info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

            # IMU
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

            # Clock (for time synchronization)
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
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

        # Start the bridge
        sensor_bridge,
    ])
