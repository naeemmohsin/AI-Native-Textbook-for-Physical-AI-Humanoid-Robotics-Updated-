# Content Model: Module 1 – The Robotic Nervous System (ROS 2)

**Feature**: 001-ros2-module
**Date**: 2025-12-16
**Purpose**: Define content structure, chapter outlines, and learning flow

## Module Overview

```text
Module 1: The Robotic Nervous System (ROS 2)
├── index.md (Module Introduction)
├── chapter-1-fundamentals.md
├── chapter-2-rclpy.md
└── chapter-3-urdf.md
```

## Content Entities

### Chapter Structure Template

Each chapter follows this consistent structure:

```markdown
---
sidebar_position: N
title: "Chapter N: [Title]"
description: "[One-line summary]"
---

# Chapter N: [Title]

## Learning Objectives
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Prerequisites
- [What reader should know before this chapter]

## [Section 1]
[Content with explanations, diagrams, code examples]

## [Section 2]
[Content...]

## Hands-On Exercise
[Practical exercise for the reader]

## Summary
- [Key takeaway 1]
- [Key takeaway 2]

## Further Reading
- [Official documentation links]
```

---

## Chapter 1: ROS 2 Fundamentals

**File**: `docs/module-1-ros2/chapter-1-fundamentals.md`
**Position**: 1
**Estimated Length**: 2000-2500 words

### Learning Objectives
1. Explain the role of ROS 2 as middleware in Physical AI systems
2. Identify and describe the four core communication patterns (nodes, topics, services, actions)
3. Set up a ROS 2 workspace and create a basic package

### Outline

```text
1. Introduction to ROS 2
   - What is ROS 2?
   - Why ROS 2 for Physical AI and humanoid robots?
   - ROS 2 vs ROS 1 (brief comparison)

2. Core Concepts
   2.1 Nodes
       - Definition and purpose
       - Node lifecycle
       - Diagram: Node as a process

   2.2 Topics (Publish/Subscribe)
       - Asynchronous communication
       - Publishers and subscribers
       - Diagram: Topic message flow
       - When to use topics

   2.3 Services (Request/Response)
       - Synchronous communication
       - Service servers and clients
       - Diagram: Service call flow
       - When to use services

   2.4 Actions (Goal/Feedback/Result)
       - Long-running tasks
       - Action servers and clients
       - Diagram: Action state machine
       - When to use actions

3. Workspace and Package Structure
   3.1 Creating a workspace
       - Directory structure
       - colcon build system

   3.2 Creating a package
       - package.xml
       - setup.py (Python packages)
       - CMakeLists.txt (C++ packages)

4. Hands-On: Your First ROS 2 Workspace
   - Step-by-step workspace creation
   - Create hello_ros2 package
   - Build and source

5. Summary
```

### Code Examples
- `examples/module-1/chapter-1/ros2_ws/` - Complete workspace structure
- Inline: `ros2 run`, `ros2 topic list`, `ros2 node list` commands

### Diagrams
- `ros2-architecture.png` - High-level ROS 2 architecture
- `node-topic-flow.png` - Topic publish/subscribe pattern
- `service-flow.png` - Service request/response pattern
- `action-flow.png` - Action goal/feedback/result pattern

---

## Chapter 2: Python Agents with rclpy

**File**: `docs/module-1-ros2/chapter-2-rclpy.md`
**Position**: 2
**Estimated Length**: 2500-3000 words

### Learning Objectives
1. Understand the rclpy execution model (spinning, callbacks, executors)
2. Implement publishers, subscribers, and services in Python
3. Design node architectures that bridge AI agents to robot controllers

### Outline

```text
1. Introduction to rclpy
   - rclpy as Python client library
   - Node class and lifecycle
   - Execution model overview

2. The Execution Model
   2.1 Spinning and Callbacks
       - spin() vs spin_once()
       - Callback groups
       - Diagram: Callback execution flow

   2.2 Executors
       - SingleThreadedExecutor
       - MultiThreadedExecutor
       - When to use each

3. Publishers and Subscribers
   3.1 Creating a Publisher
       - Full code example: talker node
       - Message types (std_msgs, geometry_msgs)
       - QoS profiles

   3.2 Creating a Subscriber
       - Full code example: listener node
       - Callback handling
       - Message filtering

4. Services
   4.1 Service Server
       - Full code example: add_two_ints server
       - Service definition files (.srv)

   4.2 Service Client
       - Synchronous vs asynchronous calls
       - Full code example: client node

5. Bridging AI Agents to Robot Controllers
   5.1 Architecture Pattern
       - AI inference node → Command publisher
       - Diagram: AI-to-robot bridge

   5.2 Example: Inference-to-Velocity Bridge
       - Receive AI decision
       - Translate to cmd_vel
       - Publish to robot

6. Error Handling and Debugging
   - Exception handling in callbacks
   - rclpy logging (get_logger())
   - Common debugging techniques

7. Hands-On: Build a Publisher-Subscriber Pair
   - Create talker and listener nodes
   - Run and verify communication

8. Summary
```

### Code Examples
- `examples/module-1/chapter-2/publisher_node.py` - Complete publisher
- `examples/module-1/chapter-2/subscriber_node.py` - Complete subscriber
- `examples/module-1/chapter-2/service_server.py` - Service server
- `examples/module-1/chapter-2/service_client.py` - Service client
- `examples/module-1/chapter-2/ai_bridge_node.py` - AI-to-robot bridge pattern

### Diagrams
- `rclpy-execution.png` - Execution model (spin loop)
- `pubsub-architecture.png` - Publisher/subscriber node architecture
- `ai-robot-bridge.png` - AI agent to robot controller bridge

---

## Chapter 3: Humanoid Description with URDF

**File**: `docs/module-1-ros2/chapter-3-urdf.md`
**Position**: 3
**Estimated Length**: 2000-2500 words

### Learning Objectives
1. Understand URDF structure: links, joints, and coordinate frames
2. Specify visual, collision, and inertial properties
3. Load and visualize URDF in ROS 2

### Outline

```text
1. Introduction to URDF
   - What is URDF?
   - Why robot descriptions matter
   - URDF in the ROS 2 ecosystem

2. URDF Structure
   2.1 Links
       - Definition and attributes
       - Visual geometry (mesh, primitive shapes)
       - Collision geometry
       - Inertial properties (mass, inertia tensor)
       - Diagram: Link anatomy

   2.2 Joints
       - Joint types (revolute, prismatic, fixed, continuous)
       - Parent-child relationships
       - Limits (position, velocity, effort)
       - Diagram: Joint types visualization

   2.3 Coordinate Frames
       - TF2 and frame relationships
       - Origin and axis specification
       - Diagram: Frame tree

3. Building a URDF Step-by-Step
   3.1 Simple Link
       - Single link with visual and collision
       - Full XML example

   3.2 Two-Link Robot Arm
       - Adding a joint
       - Parent-child linking
       - Full XML example

   3.3 Humanoid Arm Structure
       - Multiple joints (shoulder, elbow, wrist)
       - Progressive example building

4. Visual, Collision, and Inertial Elements
   4.1 Visual Elements
       - Geometry types (box, cylinder, sphere, mesh)
       - Material and color

   4.2 Collision Elements
       - Simplified geometry for physics
       - Why collision != visual

   4.3 Inertial Elements
       - Mass specification
       - Inertia tensor calculation
       - Table: Common inertia formulas

5. URDF Integration with ROS 2
   5.1 robot_state_publisher
       - Publishing robot description
       - Joint state handling

   5.2 Visualization with RViz2
       - Loading URDF in RViz2
       - Viewing TF frames

   5.3 Launch File Setup
       - Complete launch file example

6. Hands-On: Build and Visualize a Robot Arm
   - Create URDF file
   - Launch robot_state_publisher
   - View in RViz2

7. Summary
```

### Code Examples
- `examples/module-1/chapter-3/simple_link.urdf` - Single link
- `examples/module-1/chapter-3/two_link_arm.urdf` - Two-link robot
- `examples/module-1/chapter-3/humanoid_arm.urdf` - Multi-joint arm
- `examples/module-1/chapter-3/launch/display.launch.py` - Visualization launch file

### Diagrams
- `urdf-structure.png` - URDF element hierarchy
- `link-anatomy.png` - Link components (visual, collision, inertial)
- `joint-types.png` - Joint type visualizations
- `tf-tree.png` - Example TF frame tree

---

## Cross-Cutting Content Requirements

### Frontmatter Standard
All chapter files MUST include:
```yaml
---
sidebar_position: [1-3]
title: "[Chapter Title]"
description: "[SEO-friendly description]"
keywords: [ros2, robotics, ...]
---
```

### Code Block Format
```markdown
```python title="publisher_node.py"
# Code with syntax highlighting and title
```
```

### Admonition Usage
- `:::note` - Additional information
- `:::tip` - Best practices
- `:::warning` - Common pitfalls
- `:::info` - Prerequisites or context

### Internal Linking
Use relative links between chapters:
```markdown
As we learned in [Chapter 1](./chapter-1-fundamentals.md#topics)...
```
