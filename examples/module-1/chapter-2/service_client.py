"""
Service Client Example

A ROS 2 service client that calls the AddTwoInts service.
Demonstrates synchronous service calling.

Run with: ros2 run <package_name> service_client
Requires: service_server to be running
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """A ROS 2 node that calls the add_two_ints service."""

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.get_logger().info('Service available')

    def send_request(self, a, b):
        """Send request and wait for response (synchronous)."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Asynchronous call followed by spin until complete
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    client = AddTwoIntsClient()

    # Send a test request
    response = client.send_request(3, 5)
    print(f'Result: 3 + 5 = {response.sum}')

    # Send another request
    response = client.send_request(10, 20)
    print(f'Result: 10 + 20 = {response.sum}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
