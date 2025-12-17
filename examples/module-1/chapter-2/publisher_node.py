"""
Publisher Node Example

A simple ROS 2 publisher node that publishes string messages
to the 'chatter' topic at 1 Hz.

Run with: ros2 run <package_name> publisher_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """A ROS 2 node that publishes string messages."""

    def __init__(self):
        super().__init__('talker')

        # Create publisher: (message_type, topic_name, queue_size)
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

        self.get_logger().info('Talker node started')

    def timer_callback(self):
        """Timer callback that publishes a message."""
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TalkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
