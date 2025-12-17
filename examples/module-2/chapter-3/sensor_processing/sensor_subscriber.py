#!/usr/bin/env python3
"""
Sensor Data Subscriber Node

This ROS 2 node demonstrates how to subscribe to and process
simulated sensor data from Gazebo:
- LiDAR scan data (LaserScan)
- Depth camera images (Image)
- IMU data (Imu)

Usage:
    ros2 run sensor_processing sensor_subscriber

    # With specific sensors:
    ros2 run sensor_processing sensor_subscriber --ros-args \
        -p enable_lidar:=true \
        -p enable_camera:=true \
        -p enable_imu:=true
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from geometry_msgs.msg import Vector3
import numpy as np


class SensorSubscriber(Node):
    """
    A ROS 2 node that subscribes to simulated sensor data.

    This node demonstrates:
    - Subscribing to multiple sensor topics
    - Processing sensor data for basic analysis
    - Handling different QoS profiles for sensors
    """

    def __init__(self):
        super().__init__('sensor_subscriber')

        # Declare parameters
        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('enable_camera', True)
        self.declare_parameter('enable_imu', True)

        # Get parameters
        self.enable_lidar = self.get_parameter('enable_lidar').value
        self.enable_camera = self.get_parameter('enable_camera').value
        self.enable_imu = self.get_parameter('enable_imu').value

        # QoS profile for sensor data (best effort for real-time sensors)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers based on enabled sensors
        if self.enable_lidar:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.lidar_callback,
                sensor_qos
            )
            self.get_logger().info('LiDAR subscriber enabled on /scan')

        if self.enable_camera:
            self.depth_sub = self.create_subscription(
                Image,
                '/camera/depth',
                self.depth_callback,
                sensor_qos
            )
            self.rgb_sub = self.create_subscription(
                Image,
                '/camera/image',
                self.rgb_callback,
                sensor_qos
            )
            self.get_logger().info('Camera subscribers enabled on /camera/*')

        if self.enable_imu:
            self.imu_sub = self.create_subscription(
                Imu,
                '/imu',
                self.imu_callback,
                sensor_qos
            )
            self.get_logger().info('IMU subscriber enabled on /imu')

        # Statistics tracking
        self.lidar_count = 0
        self.camera_count = 0
        self.imu_count = 0

        # Create timer for periodic statistics reporting
        self.stats_timer = self.create_timer(5.0, self.report_statistics)

        self.get_logger().info('Sensor subscriber node initialized')

    def lidar_callback(self, msg: LaserScan):
        """
        Process LiDAR scan data.

        Demonstrates:
        - Extracting range values
        - Finding minimum distance (closest obstacle)
        - Calculating scan statistics
        """
        self.lidar_count += 1

        # Convert ranges to numpy array for analysis
        ranges = np.array(msg.ranges)

        # Filter out invalid readings (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_range = np.min(valid_ranges)
            max_range = np.max(valid_ranges)
            mean_range = np.mean(valid_ranges)

            # Find angle of closest obstacle
            min_idx = np.nanargmin(ranges)
            min_angle = msg.angle_min + min_idx * msg.angle_increment

            # Log every 10th message to avoid spam
            if self.lidar_count % 10 == 0:
                self.get_logger().info(
                    f'LiDAR: min={min_range:.2f}m @ {np.degrees(min_angle):.1f}°, '
                    f'mean={mean_range:.2f}m, valid_points={len(valid_ranges)}'
                )

    def depth_callback(self, msg: Image):
        """
        Process depth camera image.

        Demonstrates:
        - Understanding image dimensions
        - Extracting depth statistics
        """
        self.camera_count += 1

        # Log every 30th message (once per second at 30 FPS)
        if self.camera_count % 30 == 0:
            self.get_logger().info(
                f'Depth Image: {msg.width}x{msg.height}, '
                f'encoding={msg.encoding}'
            )

    def rgb_callback(self, msg: Image):
        """
        Process RGB camera image.

        Demonstrates:
        - Image metadata extraction
        """
        # Could add image processing here
        # For now, just acknowledge receipt
        pass

    def imu_callback(self, msg: Imu):
        """
        Process IMU data.

        Demonstrates:
        - Extracting orientation (quaternion)
        - Reading angular velocity
        - Reading linear acceleration
        """
        self.imu_count += 1

        # Extract data
        orientation = msg.orientation
        angular_vel = msg.angular_velocity
        linear_acc = msg.linear_acceleration

        # Calculate roll, pitch, yaw from quaternion
        # (simplified - for production use tf2 transformations)
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        # Log every 100th message (once per second at 100 Hz)
        if self.imu_count % 100 == 0:
            self.get_logger().info(
                f'IMU: roll={np.degrees(roll):.1f}°, '
                f'pitch={np.degrees(pitch):.1f}°, '
                f'yaw={np.degrees(yaw):.1f}° | '
                f'acc=({linear_acc.x:.2f}, {linear_acc.y:.2f}, {linear_acc.z:.2f})'
            )

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Note: For production code, use tf2 library for robust conversions.
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def report_statistics(self):
        """Report sensor message statistics periodically."""
        self.get_logger().info(
            f'Statistics (5s): LiDAR={self.lidar_count}, '
            f'Camera={self.camera_count}, IMU={self.imu_count}'
        )
        # Reset counters
        self.lidar_count = 0
        self.camera_count = 0
        self.imu_count = 0


def main(args=None):
    rclpy.init(args=args)

    node = SensorSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor subscriber...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
