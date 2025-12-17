---
sidebar_position: 2
title: "Chapter 1: ROS 2 Fundamentals"
description: Learn the core architecture and communication patterns of ROS 2 - the middleware that powers modern humanoid robots
keywords: [ros2, robotics, middleware, nodes, topics, services, actions, physical ai]
---

# Chapter 1: ROS 2 Fundamentals

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Explain what ROS 2 is and why it's essential for Physical AI and humanoid robotics
- Describe the four main communication primitives: nodes, topics, services, and actions
- Set up a ROS 2 workspace and create a basic package
- Use ROS 2 CLI tools to inspect running systems
:::

## Prerequisites

Before starting this chapter, ensure you have:
- ROS 2 Humble installed on Ubuntu 22.04 or WSL2
- Basic familiarity with the Linux command line
- Python 3.10 or later installed

## 1. Introduction to ROS 2

### What is ROS 2?

**ROS 2** (Robot Operating System 2) is not actually an operating system - it's a **middleware framework** that provides the tools, libraries, and conventions needed to build robot applications. Think of it as the "nervous system" that allows different parts of a robot to communicate and coordinate.

In a humanoid robot, you might have:
- Cameras processing visual input
- Motor controllers moving limbs
- AI models making decisions
- Sensors detecting obstacles

ROS 2 provides the infrastructure that allows all these components to exchange data reliably, regardless of whether they're running on the same computer or distributed across multiple machines.

### Why ROS 2 for Physical AI?

For AI engineers entering robotics, ROS 2 offers several key advantages:

1. **Standardized Communication**: Instead of writing custom protocols for every sensor and actuator, ROS 2 provides standard message types and communication patterns.

2. **Language Agnostic**: Write your AI inference in Python, your motor control in C++, and they can still communicate seamlessly.

3. **Hardware Abstraction**: The same code that controls a simulated robot can control a physical one with minimal changes.

4. **Ecosystem**: Thousands of packages for perception, navigation, manipulation, and more are available and actively maintained.

5. **Real-Time Capable**: Unlike ROS 1, ROS 2 is built on DDS (Data Distribution Service) and supports real-time operation critical for robot control.

### ROS 2 vs ROS 1

If you've heard of "ROS" before, you might be wondering about the difference:

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| Middleware | Custom (TCPROS/UDPROS) | DDS (standardized) |
| Real-time | Not supported | Supported |
| Multi-robot | Difficult | Native support |
| Security | None | Built-in DDS security |
| Platforms | Linux only | Linux, Windows, macOS |
| Maintenance | Legacy (EOL 2025) | Active development |

:::tip
Always use ROS 2 for new projects. This book exclusively covers ROS 2 Humble, the current Long-Term Support (LTS) release.
:::

## 2. Core Concepts

ROS 2 provides four main communication patterns. Understanding when to use each is fundamental to designing robot systems.

### 2.1 Nodes

A **node** is the basic building block of a ROS 2 system. Each node is an independent process that performs a specific task:

- A camera node captures images
- A perception node processes images to detect objects
- A planning node decides what actions to take
- A motor node sends commands to actuators

```text
┌─────────────────────────────────────────────────────────────┐
│                      Robot System                            │
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐              │
│  │  Camera  │    │ Detector │    │ Planner  │              │
│  │   Node   │───▶│   Node   │───▶│   Node   │───▶ Actions  │
│  └──────────┘    └──────────┘    └──────────┘              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**Key characteristics of nodes:**
- Each node has a unique name within the system
- Nodes can be started, stopped, and restarted independently
- A single executable can contain multiple nodes
- Nodes communicate through topics, services, or actions

To list all running nodes:

```bash
ros2 node list
```

To get information about a specific node:

```bash
ros2 node info /camera_node
```

### 2.2 Topics (Publish/Subscribe)

**Topics** enable asynchronous, one-to-many communication. A publisher sends messages to a topic, and any number of subscribers can receive those messages.

```text
┌────────────┐         /camera/image         ┌────────────┐
│  Camera    │ ─────────────────────────────▶│  Display   │
│  (Pub)     │              │                │  (Sub)     │
└────────────┘              │                └────────────┘
                            │
                            ▼
                     ┌────────────┐
                     │  Recorder  │
                     │  (Sub)     │
                     └────────────┘
```

**When to use topics:**
- Streaming data (sensor readings, camera frames)
- When you don't need to know if anyone received the message
- When multiple nodes need the same data
- Continuous, time-series data

**Common topic commands:**

```bash
# List all topics
ros2 topic list

# Show messages on a topic
ros2 topic echo /camera/image

# Get topic information (type, publishers, subscribers)
ros2 topic info /camera/image

# Publish a test message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

### 2.3 Services (Request/Response)

**Services** enable synchronous, one-to-one communication. A client sends a request and waits for a response from the server.

```text
┌────────────┐    Request     ┌────────────┐
│   Client   │ ─────────────▶ │   Server   │
│            │                │            │
│            │ ◀───────────── │            │
└────────────┘    Response    └────────────┘
```

**When to use services:**
- Discrete operations with a result (take photo, save file)
- Configuration changes (set parameter, change mode)
- Queries (get current position, check status)
- When you need confirmation that an operation completed

**Common service commands:**

```bash
# List all services
ros2 service list

# Get service type
ros2 service type /spawn

# Call a service
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0}"
```

### 2.4 Actions (Goal/Feedback/Result)

**Actions** are for long-running tasks that need progress feedback and can be canceled. They combine the features of topics and services.

```text
┌────────────┐       Goal        ┌────────────┐
│   Client   │ ─────────────────▶│   Server   │
│            │                   │            │
│            │ ◀──── Feedback ───│            │  (periodic updates)
│            │                   │            │
│            │ ◀──── Result ─────│            │  (when complete)
└────────────┘                   └────────────┘
```

**When to use actions:**
- Navigation to a goal (move to position)
- Arm manipulation (pick up object)
- Any task that takes time and should report progress
- Operations that might need to be canceled mid-execution

**Common action commands:**

```bash
# List all actions
ros2 action list

# Get action information
ros2 action info /navigate_to_pose

# Send a goal
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}"
```

### Communication Pattern Summary

| Pattern | Communication | Use Case | Example |
|---------|---------------|----------|---------|
| Topic | Async, 1-to-many | Streaming data | Camera images |
| Service | Sync, 1-to-1 | Discrete operations | Set parameter |
| Action | Async with feedback | Long-running tasks | Navigate to goal |

## 3. Workspace and Package Structure

### 3.1 Creating a Workspace

A ROS 2 **workspace** is a directory containing your packages. The standard structure is:

```text
ros2_ws/                    # Workspace root
├── src/                    # Source packages go here
│   ├── package_1/
│   └── package_2/
├── build/                  # Build artifacts (generated)
├── install/                # Installed packages (generated)
└── log/                    # Build logs (generated)
```

To create a workspace:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src

# Navigate to workspace
cd ~/ros2_ws

# Build the workspace (even if empty)
colcon build

# Source the workspace
source install/setup.bash
```

:::note
The `colcon` tool is the standard build system for ROS 2. It handles building all packages in your workspace in the correct dependency order.
:::

### 3.2 Creating a Package

A **package** is the organizational unit of ROS 2 code. Each package contains:

```text
my_package/
├── package.xml           # Package metadata and dependencies
├── setup.py              # Python package setup (for Python packages)
├── setup.cfg             # Python setup configuration
├── resource/my_package   # Marker file for ament
├── my_package/           # Python module directory
│   ├── __init__.py
│   └── my_node.py
└── test/                 # Test files
```

**package.xml** declares the package name, version, dependencies, and maintainer:

```xml title="package.xml"
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**setup.py** tells Python how to install the package:

```python title="setup.py"
from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My first ROS 2 package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
        ],
    },
)
```

## 4. Hands-On Exercise: Your First ROS 2 Workspace

Let's create a complete workspace with a simple "Hello ROS 2" package.

### Step 1: Create the Workspace

```bash
# Create workspace structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2: Create a Python Package

```bash
# Use ros2 pkg create to generate package structure
ros2 pkg create --build-type ament_python hello_ros2 --dependencies rclpy std_msgs
```

This creates the following structure:

```text
hello_ros2/
├── hello_ros2/
│   └── __init__.py
├── package.xml
├── resource/
│   └── hello_ros2
├── setup.cfg
├── setup.py
└── test/
```

### Step 3: Create a Simple Node

Create a new file `hello_ros2/hello_node.py`:

```python title="hello_ros2/hello_node.py"
import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')

        # Create a timer that fires every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.count += 1
        self.get_logger().info(f'Count: {self.count}')


def main(args=None):
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
```

### Step 4: Register the Node Entry Point

Edit `setup.py` to add the entry point:

```python title="setup.py (entry_points section)"
entry_points={
    'console_scripts': [
        'hello_node = hello_ros2.hello_node:main',
    ],
},
```

### Step 5: Build and Run

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select hello_ros2

# Source the workspace
source install/setup.bash

# Run the node
ros2 run hello_ros2 hello_node
```

You should see output like:

```text
[INFO] [hello_node]: Hello, ROS 2!
[INFO] [hello_node]: Count: 1
[INFO] [hello_node]: Count: 2
[INFO] [hello_node]: Count: 3
...
```

### Step 6: Inspect Your Node

In a new terminal (with the workspace sourced):

```bash
# List running nodes
ros2 node list

# Output: /hello_node

# Get node info
ros2 node info /hello_node
```

:::tip
Always source your workspace after building: `source install/setup.bash`. Add this to your `~/.bashrc` to do it automatically.
:::

## Summary

In this chapter, you learned:

- **ROS 2** is a middleware framework that provides the communication infrastructure for robot systems, acting as the "nervous system" of Physical AI applications.

- **Nodes** are independent processes that perform specific tasks and communicate through three patterns:
  - **Topics**: Asynchronous, streaming communication (sensors, continuous data)
  - **Services**: Synchronous, request-response communication (discrete operations)
  - **Actions**: Long-running tasks with progress feedback (navigation, manipulation)

- **Workspaces** organize your ROS 2 packages, and **colcon** is the build tool that compiles them.

- **Packages** are the unit of organization, containing nodes, message definitions, and other resources.

## Further Reading

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [Understanding Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Understanding Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Understanding Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

---

**Next**: [Chapter 2: Python Agents with rclpy](./chapter-2-rclpy.md) - Learn to write Python ROS 2 nodes with publishers, subscribers, and services.
