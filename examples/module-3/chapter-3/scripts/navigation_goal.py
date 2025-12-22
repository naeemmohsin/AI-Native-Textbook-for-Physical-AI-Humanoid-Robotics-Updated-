#!/usr/bin/env python3
"""
Navigation Goal Sender

This script sends navigation goals to Nav2 programmatically.
Use this for automated testing or waypoint-based missions.

Usage:
    # Send single goal
    ros2 run my_package navigation_goal.py --ros-args -p x:=2.0 -p y:=1.5 -p yaw:=1.57

    # As a library
    from navigation_goal import NavigationClient
    client = NavigationClient()
    client.send_goal(x=2.0, y=1.5, yaw=0.0)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
from action_msgs.msg import GoalStatus


class NavigationClient(Node):
    """
    A ROS 2 node for sending navigation goals to Nav2.

    Provides methods for:
    - Sending single pose goals
    - Sending waypoint sequences
    - Setting initial pose
    - Monitoring navigation status
    """

    def __init__(self):
        super().__init__('navigation_client')

        # Declare parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        # Create callback group for concurrent callbacks
        self._callback_group = ReentrantCallbackGroup()

        # Action clients
        self._navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._callback_group
        )

        self._navigate_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
            callback_group=self._callback_group
        )

        # Publisher for initial pose
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )

        # State tracking
        self._goal_handle = None
        self._result_future = None
        self._feedback = None

        self.get_logger().info('Navigation client initialized')

    def yaw_to_quaternion(self, yaw: float) -> tuple:
        """Convert yaw angle to quaternion (x, y, z, w)."""
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0):
        """
        Set the robot's initial pose for localization.

        Args:
            x: X position in meters
            y: Y position in meters
            yaw: Orientation in radians
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Set covariance (diagonal)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467,
        ]

        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Initial pose set: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})'
        )

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        """
        Send a navigation goal to Nav2.

        Args:
            x: Target X position in meters
            y: Target Y position in meters
            yaw: Target orientation in radians
        """
        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

        # Wait for action server
        while not self._navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigate_to_pose action server...')

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # Send goal
        self._send_goal_future = self._navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def send_waypoints(self, waypoints: list):
        """
        Send a sequence of waypoints to navigate through.

        Args:
            waypoints: List of (x, y, yaw) tuples
        """
        self.get_logger().info(f'Sending {len(waypoints)} waypoints')

        # Wait for action server
        while not self._navigate_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigate_through_poses action server...')

        # Create goal message
        goal_msg = NavigateThroughPoses.Goal()

        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            q = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            goal_msg.poses.append(pose)

        # Send goal
        self._send_goal_future = self._navigate_through_poses_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def cancel_goal(self):
        """Cancel the current navigation goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Canceling current goal')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)

    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        self._feedback = feedback_msg.feedback

        # Log progress
        distance = self._feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f}m')

    def _goal_response_callback(self, future):
        """Handle goal response."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled!')
        else:
            self.get_logger().warn(f'Navigation finished with status: {status}')

    def _cancel_done_callback(self, future):
        """Handle cancel response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().warn('Goal cancel request failed')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    navigator = NavigationClient()

    # Get parameters
    x = navigator.get_parameter('x').value
    y = navigator.get_parameter('y').value
    yaw = navigator.get_parameter('yaw').value

    # Send goal if coordinates provided
    if x != 0.0 or y != 0.0:
        navigator.send_goal(x=x, y=y, yaw=yaw)

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
