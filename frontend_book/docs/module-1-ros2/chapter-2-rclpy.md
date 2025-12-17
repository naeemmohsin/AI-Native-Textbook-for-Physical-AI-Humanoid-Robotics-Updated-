---
sidebar_position: 3
title: "Chapter 2: Python Agents with rclpy"
description: Build Python ROS 2 nodes with publishers, subscribers, and services using the rclpy client library
keywords: [ros2, rclpy, python, publisher, subscriber, service, robotics, physical ai]
---

# Chapter 2: Python Agents with rclpy

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Create ROS 2 nodes using the rclpy Python library
- Implement publishers and subscribers for topic-based communication
- Build service servers and clients for request/response patterns
- Understand the execution model (spin, callbacks, executors)
- Bridge AI agents to robot controllers using ROS 2
:::

## Prerequisites

Before starting this chapter, ensure you have:
- Completed [Chapter 1: ROS 2 Fundamentals](./chapter-1-fundamentals.md)
- A working ROS 2 workspace
- Python 3.10 or later

## 1. Introduction to rclpy

**rclpy** is the official Python client library for ROS 2. It provides a Pythonic interface to the ROS 2 middleware, allowing you to write nodes that communicate using topics, services, and actions.

### The Node Class

Every rclpy node inherits from the `Node` class, which provides:

- **Logging**: Built-in logger for output messages
- **Timers**: Callbacks that fire at specified intervals
- **Publishers/Subscribers**: Topic-based communication
- **Services**: Request/response communication
- **Parameters**: Configuration values

```python title="Basic node structure"
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')  # Node name
        self.get_logger().info('Node started!')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Node Lifecycle

A typical rclpy node follows this lifecycle:

1. **Initialize**: `rclpy.init()` - Initialize the ROS 2 client library
2. **Create**: Instantiate your node class
3. **Spin**: `rclpy.spin()` - Process callbacks until shutdown
4. **Cleanup**: `node.destroy_node()` and `rclpy.shutdown()`

## 2. The Execution Model

### 2.1 Spinning and Callbacks

In ROS 2, **spinning** means continuously checking for and executing callbacks. There are several spin variants:

```python
# Spin forever (blocks until shutdown)
rclpy.spin(node)

# Spin once (process one callback, then return)
rclpy.spin_once(node)

# Spin once with timeout
rclpy.spin_once(node, timeout_sec=1.0)

# Spin until a condition is met
while rclpy.ok() and not done:
    rclpy.spin_once(node, timeout_sec=0.1)
```

**Callbacks** are functions that execute in response to events:
- Timer callbacks fire at regular intervals
- Subscription callbacks fire when messages arrive
- Service callbacks fire when requests are received

```text
┌─────────────────────────────────────────────────────┐
│                   Spin Loop                          │
│                                                      │
│   ┌───────────┐    ┌───────────┐    ┌───────────┐  │
│   │   Timer   │    │   Topic   │    │  Service  │  │
│   │ Callback  │    │ Callback  │    │ Callback  │  │
│   └─────┬─────┘    └─────┬─────┘    └─────┬─────┘  │
│         │                │                │         │
│         └────────────────┼────────────────┘         │
│                          ▼                          │
│                    ┌──────────┐                     │
│                    │ Executor │                     │
│                    └──────────┘                     │
└─────────────────────────────────────────────────────┘
```

### 2.2 Executors

**Executors** manage how callbacks are scheduled and executed:

**SingleThreadedExecutor** (default): Executes callbacks one at a time in a single thread. Simple and safe, but callbacks block each other.

```python
from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

**MultiThreadedExecutor**: Can execute callbacks concurrently in multiple threads. Better throughput, but requires thread-safe code.

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

:::tip When to use each executor
- Use **SingleThreadedExecutor** (default) for most cases - it's simpler and avoids threading issues
- Use **MultiThreadedExecutor** when you have long-running callbacks that would block other processing
:::

## 3. Publishers and Subscribers

### 3.1 Creating a Publisher

A **publisher** sends messages to a topic. Here's a complete example of a "talker" node that publishes string messages:

```python title="publisher_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')

        # Create publisher: (message_type, topic_name, queue_size)
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

        self.get_logger().info('Talker node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
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
```

**Key concepts:**
- `create_publisher(msg_type, topic, qos)` - Creates a publisher
- Message types come from packages like `std_msgs`, `geometry_msgs`, `sensor_msgs`
- The **QoS (Quality of Service)** value (10 here) sets the queue depth

### Common Message Types

| Package | Message | Use Case |
|---------|---------|----------|
| `std_msgs` | `String`, `Int32`, `Float64`, `Bool` | Simple data types |
| `geometry_msgs` | `Twist`, `Pose`, `Point`, `Vector3` | Motion and position |
| `sensor_msgs` | `Image`, `LaserScan`, `Imu`, `JointState` | Sensor data |

### 3.2 Creating a Subscriber

A **subscriber** receives messages from a topic. Here's a "listener" node:

```python title="subscriber_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
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
        """Called whenever a message is received."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
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
```

**Key concepts:**
- `create_subscription(msg_type, topic, callback, qos)` - Creates a subscriber
- The callback function receives the message as its argument
- Callbacks execute in the spin thread

## 4. Services

### 4.1 Service Server

A **service server** handles requests and returns responses. Here's an example using the `AddTwoInts` service:

```python title="service_server.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
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
        """Handle service request."""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
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
```

### 4.2 Service Client

A **service client** sends requests and receives responses. There are two approaches:

**Synchronous call** (blocks until response):

```python title="service_client.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        """Send request and wait for response."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Synchronous call
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()

    response = client.send_request(3, 5)
    print(f'Result: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Asynchronous call** (non-blocking with callback):

```python title="service_client_async.py"
def send_request_async(self, a, b):
    """Send request with callback."""
    request = AddTwoInts.Request()
    request.a = a
    request.b = b

    future = self.client.call_async(request)
    future.add_done_callback(self.response_callback)

def response_callback(self, future):
    """Handle response when ready."""
    response = future.result()
    self.get_logger().info(f'Result: {response.sum}')
```

:::warning
Avoid calling synchronous service calls from within callbacks - this can cause deadlocks. Use async calls instead.
:::

## 5. Bridging AI Agents to Robot Controllers

One of the most powerful applications of ROS 2 is connecting AI systems to robot hardware. Here's a pattern for bridging AI inference outputs to robot control commands:

### Architecture Pattern

```text
┌─────────────────┐     /ai_decision     ┌─────────────────┐
│   AI Inference  │ ───────────────────▶ │   Bridge Node   │
│      Node       │                      │                 │
└─────────────────┘                      │  Translates AI  │
                                         │  decisions to   │
                                         │  robot commands │
                                         │                 │
                                         └────────┬────────┘
                                                  │
                                                  ▼ /cmd_vel
                                         ┌─────────────────┐
                                         │  Robot Driver   │
                                         │      Node       │
                                         └─────────────────┘
```

### Example: AI Decision to Velocity Bridge

This node receives AI decisions (as strings like "forward", "left", "stop") and converts them to velocity commands:

```python title="ai_bridge_node.py"
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

        # Movement parameters
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

        self.get_logger().info('AI Bridge node ready')

    def decision_callback(self, msg):
        """Convert AI decision to velocity command."""
        decision = msg.data.lower()
        cmd = Twist()

        if decision == 'forward':
            cmd.linear.x = self.linear_speed
        elif decision == 'backward':
            cmd.linear.x = -self.linear_speed
        elif decision == 'left':
            cmd.angular.z = self.angular_speed
        elif decision == 'right':
            cmd.angular.z = -self.angular_speed
        elif decision == 'stop':
            pass  # All velocities default to 0
        else:
            self.get_logger().warn(f'Unknown decision: {decision}')
            return

        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(
            f'Decision "{decision}" -> linear={cmd.linear.x}, angular={cmd.angular.z}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AIBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This pattern allows you to:
- Keep AI inference separate from robot control
- Test the bridge with simulated AI outputs
- Swap different AI models without changing robot code
- Add safety checks and limits in the bridge

## 6. Error Handling and Debugging

### Exception Handling in Callbacks

Always wrap callback code in try/except to prevent crashes:

```python
def callback(self, msg):
    try:
        # Process message
        result = self.process(msg.data)
    except ValueError as e:
        self.get_logger().error(f'Invalid data: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error: {e}')
```

### Using the Logger

rclpy provides a built-in logger with severity levels:

```python
# Different log levels
self.get_logger().debug('Detailed debugging info')
self.get_logger().info('General information')
self.get_logger().warn('Warning - something unexpected')
self.get_logger().error('Error - something went wrong')
self.get_logger().fatal('Fatal - unrecoverable error')
```

### Common Debugging Techniques

**1. Echo topics** to see what's being published:
```bash
ros2 topic echo /chatter
```

**2. Check topic info** for publishers and subscribers:
```bash
ros2 topic info /chatter -v
```

**3. Use rqt_graph** to visualize the node graph:
```bash
ros2 run rqt_graph rqt_graph
```

**4. Check node parameters**:
```bash
ros2 param list /my_node
ros2 param get /my_node my_parameter
```

## 7. Hands-On Exercise: Build a Publisher-Subscriber Pair

Let's put it all together by creating a complete publisher-subscriber system.

### Step 1: Create the Package

In your workspace's `src` directory:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python pubsub_demo --dependencies rclpy std_msgs
```

### Step 2: Create the Publisher Node

Create `pubsub_demo/talker.py`:

```python title="pubsub_demo/talker.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Message {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```

### Step 3: Create the Subscriber Node

Create `pubsub_demo/listener.py`:

```python title="pubsub_demo/listener.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter', self.callback, 10
        )

    def callback(self, msg):
        self.get_logger().info(f'Heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
```

### Step 4: Update setup.py

Add entry points in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'talker = pubsub_demo.talker:main',
        'listener = pubsub_demo.listener:main',
    ],
},
```

### Step 5: Build and Run

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select pubsub_demo
source install/setup.bash

# Terminal 1: Run the talker
ros2 run pubsub_demo talker

# Terminal 2: Run the listener
ros2 run pubsub_demo listener
```

You should see the talker publishing messages and the listener receiving them!

## Summary

In this chapter, you learned:

- **rclpy** is the Python client library for ROS 2, built around the `Node` class
- **Spinning** processes callbacks - use `spin()` for continuous operation or `spin_once()` for single iterations
- **Executors** control how callbacks are scheduled - `SingleThreadedExecutor` for simple cases, `MultiThreadedExecutor` for concurrent processing
- **Publishers** send messages to topics; **Subscribers** receive them with callbacks
- **Services** provide request/response communication - servers handle requests, clients make them
- **AI bridges** translate AI decisions to robot commands, keeping concerns separated
- **Logging** and **CLI tools** help debug node behavior

## Further Reading

- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Writing a Simple Service and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Understanding Executors](https://docs.ros.org/en/humble/Concepts/About-Executors.html)

---

**Next**: [Chapter 3: Humanoid Description with URDF](./chapter-3-urdf.md) - Learn to define robot structure using URDF for visualization and simulation.
