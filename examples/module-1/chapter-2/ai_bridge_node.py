"""
AI Bridge Node Example

A ROS 2 node that bridges AI decisions to robot velocity commands.
This demonstrates a common pattern for connecting AI inference
outputs to robot control.

Subscribes to: /ai_decision (std_msgs/String)
Publishes to: /cmd_vel (geometry_msgs/Twist)

Run with: ros2 run <package_name> ai_bridge_node
Test with: ros2 topic pub /ai_decision std_msgs/msg/String "{data: 'forward'}"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class AIBridgeNode(Node):
    """Bridges AI decisions to robot velocity commands."""

    def __init__(self):
        super().__init__('ai_bridge')

        # Subscribe to AI decisions
        self.decision_sub = self.create_subscription(
            String,
            'ai_decision',
            self.decision_callback,
            10
        )

        # Publish velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Movement parameters (can be configured via ROS parameters)
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

        # Valid decisions
        self.valid_decisions = ['forward', 'backward', 'left', 'right', 'stop']

        self.get_logger().info('AI Bridge node ready')
        self.get_logger().info(f'Valid decisions: {self.valid_decisions}')

    def decision_callback(self, msg):
        """Convert AI decision to velocity command."""
        decision = msg.data.lower().strip()
        cmd = Twist()

        if decision == 'forward':
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        elif decision == 'backward':
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0
        elif decision == 'left':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
        elif decision == 'right':
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
        elif decision == 'stop':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            self.get_logger().warn(
                f'Unknown decision: "{decision}". '
                f'Valid options: {self.valid_decisions}'
            )
            return

        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(
            f'Decision "{decision}" -> '
            f'linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}'
        )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = AIBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command on shutdown
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.get_logger().info('Shutting down, sent stop command')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
