"""
Service Server Example

A ROS 2 service server that provides an AddTwoInts service.
The service accepts two integers and returns their sum.

Run with: ros2 run <package_name> service_server
Test with: ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """A ROS 2 node that provides an add_two_ints service."""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service: (service_type, service_name, callback)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        self.get_logger().info('AddTwoInts service ready')

    def add_callback(self, request, response):
        """Handle service request by adding two integers."""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = AddTwoIntsServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
