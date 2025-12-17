#!/usr/bin/env python3
"""
ROS TCP Endpoint Launch File

Starts the ROS-TCP-Endpoint server for Unity-ROS 2 communication.

This launch file configures the TCP endpoint that Unity's
ROS-TCP-Connector package connects to for bidirectional
message passing.

Usage:
    ros2 launch ros_unity_bridge ros_tcp_endpoint.launch.py

    # With custom settings:
    ros2 launch ros_unity_bridge ros_tcp_endpoint.launch.py \
        ros_ip:=0.0.0.0 ros_tcp_port:=10000

Requirements:
    - ros_tcp_endpoint package installed
    - Unity project with ROS-TCP-Connector configured
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    ros_ip = LaunchConfiguration('ros_ip', default='0.0.0.0')
    ros_tcp_port = LaunchConfiguration('ros_tcp_port', default='10000')

    # ROS TCP Endpoint node
    ros_tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[{
            'ROS_IP': ros_ip,
            'ROS_TCP_PORT': ros_tcp_port,
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'ros_ip',
            default_value='0.0.0.0',
            description='IP address for the ROS TCP endpoint (0.0.0.0 for all interfaces)'
        ),
        DeclareLaunchArgument(
            'ros_tcp_port',
            default_value='10000',
            description='Port for the ROS TCP endpoint'
        ),

        # Start the endpoint
        ros_tcp_endpoint_node,
    ])
