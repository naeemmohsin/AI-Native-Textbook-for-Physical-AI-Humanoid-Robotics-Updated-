"""
Display Launch File Example

This launch file loads a URDF robot description and launches:
- robot_state_publisher: Publishes the robot description and TF transforms
- joint_state_publisher_gui: GUI to manually control joint angles
- rviz2: Visualization tool

Usage:
  ros2 launch robot_arm_description display.launch.py

Prerequisites:
  - robot_state_publisher package
  - joint_state_publisher_gui package
  - rviz2 package
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    # In a real package, use get_package_share_directory('your_package')
    # For this example, we assume the URDF is in the same directory
    pkg_path = get_package_share_directory('robot_arm_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot state publisher - publishes TF transforms based on URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Joint state publisher GUI - allows manual joint control
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # Optionally load a saved RViz config:
            # arguments=['-d', os.path.join(pkg_path, 'rviz', 'display.rviz')],
        ),
    ])
