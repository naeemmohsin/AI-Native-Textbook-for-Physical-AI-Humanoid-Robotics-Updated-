"""
Subscriber Node Example

A simple ROS 2 subscriber node that receives string messages
from the 'chatter' topic.

Run with: ros2 run <package_name> subscriber_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """A ROS 2 node that subscribes to string messages."""

    def __init__(self):
        super().__init__('listener')

        # Create subscription: (message_type, topic_name, callback, queue_size)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        self.get_logger().info('Listener node started')

    def listener_callback(self, msg):
        """Callback executed whenever a message is received."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
