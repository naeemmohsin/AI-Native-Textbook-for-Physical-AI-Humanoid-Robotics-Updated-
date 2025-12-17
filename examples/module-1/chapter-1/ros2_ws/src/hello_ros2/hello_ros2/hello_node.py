"""
Hello ROS 2 Node Example

This is a simple ROS 2 node that demonstrates the basic structure
of a Python node using rclpy. It logs a greeting message and then
counts up every second.

Run with: ros2 run hello_ros2 hello_node
"""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """A simple ROS 2 node that counts and logs messages."""

    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')

        # Create a timer that fires every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        """Timer callback that increments and logs the count."""
        self.count += 1
        self.get_logger().info(f'Count: {self.count}')


def main(args=None):
    """Main entry point for the hello_node."""
    rclpy.init(args=args)
    node = HelloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
