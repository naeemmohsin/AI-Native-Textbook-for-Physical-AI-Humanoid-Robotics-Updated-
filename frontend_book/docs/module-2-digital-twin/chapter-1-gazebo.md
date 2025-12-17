---
sidebar_position: 2
title: "Chapter 1: Physics Simulation with Gazebo"
description: "Learn digital twin concepts and physics-based simulation using Gazebo Harmonic"
---

# Chapter 1: Physics Simulation with Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain what a digital twin is and its value in robotics development
- Understand physics engine fundamentals: rigid body dynamics, gravity, and collisions
- Import URDF humanoid robots into Gazebo Harmonic
- Create and configure Gazebo world files with physics properties
- Run physics simulations and observe realistic robot behavior

## Prerequisites

Before starting this chapter, ensure you have:

- Completed Module 1 (ROS 2 Fundamentals, rclpy, URDF)
- ROS 2 Humble installed with Gazebo Harmonic (`ros-humble-ros-gz`)
- A URDF humanoid robot model (from Module 1 Chapter 3)
- GPU with OpenGL 3.3+ support

---

## Introduction to Digital Twins

A **digital twin** is a virtual representation of a physical system that mirrors its real-world counterpart in behavior, properties, and state. In robotics, digital twins serve as the bridge between software development and physical deployment, enabling engineers to test algorithms, validate designs, and train AI systems without risking expensive hardware.

### Why Digital Twins Matter

Traditional robotics development follows a risky pattern: write code, deploy to hardware, discover problems, repeat. This cycle is slow, expensive, and dangerous. Digital twins transform this workflow:

| Traditional Approach | Digital Twin Approach |
|---------------------|----------------------|
| Deploy untested code to hardware | Test in simulation first |
| Risk hardware damage | Safe virtual testing |
| Limited test scenarios | Unlimited test variations |
| Expensive real-world testing | Low-cost simulation |
| Time-consuming iteration | Rapid iteration cycles |

### The Simulation-Reality Gap

While digital twins are powerful, they're not perfect. The **sim-to-real gap** describes differences between simulated and real-world behavior:

- **Physics approximations**: Simulations simplify real physics
- **Sensor noise**: Real sensors have imperfections not always modeled
- **Environmental factors**: Temperature, humidity, lighting variations
- **Manufacturing tolerances**: Real robots vary from CAD models

Throughout this module, we'll address these gaps by using realistic physics engines, adding sensor noise, and validating simulation against real-world data.

:::tip Digital Twin Use Cases
- **Algorithm development**: Test control algorithms before hardware deployment
- **Training data generation**: Create synthetic data for machine learning
- **Failure analysis**: Simulate dangerous scenarios safely
- **Design validation**: Test robot designs before manufacturing
:::

---

## Physics Engine Fundamentals

Physics engines are the computational heart of any robotics simulation. They calculate how objects move, interact, and respond to forces in the virtual world. Understanding these fundamentals helps you configure simulations that behave realistically.

### Rigid Body Dynamics

In simulation, robots are typically modeled as **rigid bodies**—objects that don't deform under forces. Each rigid body has:

- **Mass**: Affects how the body responds to forces (heavier = harder to accelerate)
- **Inertia tensor**: Describes resistance to rotational motion
- **Center of mass**: The balance point of the body
- **Pose**: Position (x, y, z) and orientation (roll, pitch, yaw)

The physics engine solves Newton's equations of motion to determine how these properties change over time:

```
F = ma (Force = mass × acceleration)
τ = Iα (Torque = inertia × angular acceleration)
```

### Gravity

Gravity is the constant downward force acting on all objects. In Gazebo, Earth's gravity is configured as:

```xml
<gravity>0 0 -9.81</gravity>
```

This vector applies a force of 9.81 m/s² in the negative Z direction (downward). When you spawn a robot, gravity immediately pulls it toward the ground—if there's no ground plane, it falls forever.

### Collisions and Contact Dynamics

When objects touch, the physics engine must:

1. **Detect the collision**: Determine which surfaces are in contact
2. **Calculate contact forces**: Prevent objects from passing through each other
3. **Apply friction**: Resist sliding motion between surfaces
4. **Handle bouncing**: Apply restitution for elastic collisions

Collision properties are crucial for realistic humanoid simulation:

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>      <!-- Static friction coefficient -->
      <mu2>1.0</mu2>    <!-- Dynamic friction coefficient -->
    </ode>
  </friction>
  <bounce>
    <restitution_coefficient>0.0</restitution_coefficient>
  </bounce>
</surface>
```

:::note Friction Coefficients
- `mu = 0.0`: Frictionless (ice-like)
- `mu = 0.5`: Low friction (polished wood)
- `mu = 1.0`: High friction (rubber on concrete)
- Values > 1.0: Very high friction (specialized surfaces)
:::

### Physics Engines in Gazebo

Gazebo Harmonic supports multiple physics engines, each with trade-offs:

| Engine | Strengths | Best For |
|--------|-----------|----------|
| **DART** | Accurate contact dynamics | Humanoid robots, manipulation |
| **Bullet** | Fast, stable | Mobile robots, large environments |
| **ODE** | Widely tested | Legacy compatibility |

For humanoid robots, we recommend **DART** due to its superior handling of complex contact scenarios like walking and grasping.

---

## Gazebo Harmonic Overview

Gazebo Sim (formerly Ignition Gazebo) is the next-generation robotics simulator, with Gazebo Harmonic being the latest LTS release paired with ROS 2 Humble.

### Evolution from Gazebo Classic

If you've used the older "Gazebo Classic" (versions up to 11), you'll notice significant changes:

| Feature | Gazebo Classic | Gazebo Harmonic |
|---------|---------------|-----------------|
| Architecture | Monolithic | Plugin-based |
| ROS Integration | gazebo_ros | ros_gz |
| File Format | SDF 1.6 | SDF 1.9+ |
| Command Line | `gazebo` | `gz sim` |
| Physics | ODE default | DART default |

### Key Features

Gazebo Harmonic provides:

- **Modular plugin architecture**: Load only what you need
- **Native ROS 2 integration**: Via `ros_gz` packages
- **Modern rendering**: Ogre 2 with PBR materials
- **Improved sensors**: GPU-accelerated LiDAR, cameras
- **Better physics**: DART engine for contact-rich scenarios

### ros_gz Integration

The `ros_gz` packages bridge Gazebo and ROS 2:

- **ros_gz_bridge**: Bidirectional topic/service translation
- **ros_gz_sim**: Launch Gazebo from ROS 2 launch files
- **ros_gz_image**: Camera image transport

---

## Setting Up Gazebo Environment

Let's verify your Gazebo installation and explore the interface.

### Verify Installation

First, check that Gazebo Harmonic is installed:

```bash
# Check Gazebo version
gz sim --version
# Expected output: Gazebo Sim, version 8.x.x

# Verify ros_gz packages
ros2 pkg list | grep ros_gz
# Expected: ros_gz_bridge, ros_gz_sim, ros_gz_image, etc.
```

If packages are missing, install them:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz
```

### Launching Gazebo

Start Gazebo with an empty world:

```bash
gz sim empty.sdf
```

Or with a sample world containing shapes:

```bash
gz sim shapes.sdf
```

### Understanding the GUI

The Gazebo GUI consists of several panels:

- **3D Scene**: Main viewport showing the simulation world
- **Entity Tree**: Hierarchical view of all models and links
- **Component Inspector**: Properties of selected entities
- **Playback Controls**: Play, pause, step, reset simulation

**Key Controls:**
- **Left-click + drag**: Rotate view
- **Right-click + drag**: Pan view
- **Scroll wheel**: Zoom
- **Space**: Pause/resume simulation

### Command-Line Tools

Gazebo provides powerful CLI tools:

```bash
# List running simulations
gz sim -l

# Spawn a model
gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf: "<model name=\"box\">...</model>"'

# Get simulation info
gz topic -l  # List topics
gz topic -e -t /clock  # Echo clock topic
```

---

## Importing URDF Humanoids

Now let's bring your Module 1 URDF humanoid into Gazebo. This involves converting URDF to SDF format and spawning the robot.

### URDF vs. SDF

While ROS uses URDF (Unified Robot Description Format), Gazebo prefers SDF (Simulation Description Format):

| Aspect | URDF | SDF |
|--------|------|-----|
| Purpose | Robot description | Full world description |
| Physics | Basic | Advanced (friction, damping) |
| Sensors | Limited | Comprehensive |
| World features | None | Lights, physics settings |

The solution: Keep URDF as your robot source, convert to SDF for Gazebo.

### Converting URDF to SDF

Use the `gz sdf` tool for conversion:

```bash
# Convert URDF to SDF
gz sdf -p humanoid.urdf > humanoid.sdf

# Validate the result
gz sdf -k humanoid.sdf
```

We've provided a helper script in the examples:

```bash title="examples/module-2/chapter-1/humanoid_gazebo/urdf_to_sdf.sh"
#!/bin/bash
# Usage: ./urdf_to_sdf.sh humanoid.urdf humanoid.sdf

gz sdf -p "$1" > "$2"
gz sdf -k "$2"  # Validate
```

### Spawning via ROS 2 Launch

The recommended approach is spawning through a ROS 2 launch file:

```python title="examples/module-2/chapter-1/humanoid_gazebo/launch/spawn_humanoid.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': '-r humanoid_world.sdf'}.items()
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'humanoid',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '1.0'
            ],
        ),

        # Bridge clock to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),
    ])
```

### Verifying the Import

After spawning, verify your robot:

1. **Visual check**: Robot appears in Gazebo with correct structure
2. **TF tree**: `ros2 run tf2_tools view_frames` shows all links
3. **Joint states**: `ros2 topic echo /joint_states` shows joint positions

Common import issues:
- **Missing meshes**: Check file paths in URDF
- **Incorrect inertia**: Robot behaves strangely under gravity
- **Missing collisions**: Robot passes through objects

---

## Creating World Files

World files define the complete simulation environment: ground, obstacles, lighting, and physics configuration.

### SDF World Structure

A world file follows this structure:

```xml title="examples/module-2/chapter-1/humanoid_gazebo/worlds/humanoid_world.sdf"
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="humanoid_world">

    <!-- Physics configuration -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Required plugins -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><size>100 100</size></plane></geometry>
        </visual>
      </link>
    </model>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

  </world>
</sdf>
```

### Adding Obstacles

Create interesting environments with static obstacles:

```xml
<!-- Static box obstacle -->
<model name="obstacle_box">
  <static>true</static>
  <pose>2 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
      <material>
        <ambient>0.3 0.3 0.8 1</ambient>
      </material>
    </visual>
  </link>
</model>
```

### Configuring Physics Properties

Physics settings affect simulation accuracy and speed:

```xml
<physics name="dart_physics" type="dart">
  <!-- Simulation step size (seconds) -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor: 1.0 = real-time, 0.5 = half speed -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Updates per second -->
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- DART-specific settings -->
  <dart>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

:::warning Step Size Trade-off
- **Smaller step size** (0.001s): More accurate, but slower
- **Larger step size** (0.01s): Faster, but may miss collisions

For humanoids, use 0.001s or smaller for stable contact dynamics.
:::

---

## Running Simulations

With your world and robot ready, let's run simulations and interact with ROS 2.

### Starting the Simulation

Use the launch file to start everything:

```bash
# From your workspace
ros2 launch humanoid_gazebo spawn_humanoid.launch.py
```

### Simulation Controls

Control the simulation via GUI or command line:

```bash
# Pause simulation
gz service -s /world/humanoid_world/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --req 'pause: true'

# Step forward
gz service -s /world/humanoid_world/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --req 'step: true'
```

### ROS 2 Integration

Bridge topics between Gazebo and ROS 2:

```bash
# Bridge clock
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock

# Bridge joint states
ros2 run ros_gz_bridge parameter_bridge \
  /world/humanoid_world/model/humanoid/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model

# List all available topics
ros2 topic list
```

### Observing Physics Behavior

With the simulation running, observe:

1. **Gravity effect**: Robot falls if spawned above ground
2. **Collision response**: Robot stops at ground plane
3. **Joint dynamics**: Joints respond to gravity load

Try applying forces:

```bash
# Apply a force to a link
gz service -s /world/humanoid_world/wrench \
  --reqtype gz.msgs.EntityWrench \
  --reptype gz.msgs.Boolean \
  --req 'entity: {name: "humanoid::base_link"}, wrench: {force: {x: 100}}'
```

---

## Hands-On Exercise

Now it's your turn to apply what you've learned.

### Exercise: Create a Custom Simulation Environment

**Objective**: Import your Module 1 humanoid URDF into a custom Gazebo world with obstacles and verify physics behavior.

### Step 1: Prepare Your URDF

1. Locate your humanoid URDF from Module 1
2. Verify it has proper inertia values for all links
3. Ensure collision geometries are defined

### Step 2: Convert to SDF

```bash
cd ~/ros2_ws/src/humanoid_description/urdf
gz sdf -p humanoid.urdf > humanoid.sdf
gz sdf -k humanoid.sdf  # Validate
```

### Step 3: Create a World File

Create a new world file `my_world.sdf` with:
- Ground plane with high friction (mu = 1.0)
- At least two static obstacles (boxes or cylinders)
- One dynamic object (ball that can be pushed)
- DART physics engine with 0.001s step size

### Step 4: Create a Launch File

Write a ROS 2 launch file that:
1. Starts Gazebo with your world
2. Spawns your humanoid robot
3. Bridges `/clock` and `/joint_states` to ROS 2

### Step 5: Verify Physics Behavior

Run your simulation and verify:
- [ ] Robot spawns correctly in Gazebo
- [ ] Robot falls under gravity and lands on ground
- [ ] Robot doesn't pass through obstacles
- [ ] Dynamic ball can be pushed by external forces
- [ ] Joint states appear in ROS 2 (`ros2 topic echo /joint_states`)

### Expected Outcome

Your simulation should show a humanoid robot standing on a ground plane surrounded by obstacles, with all physics behaving realistically.

---

## Key Takeaways

- **Digital twins** enable safe, rapid iteration in robotics development
- **Physics engines** simulate rigid body dynamics, gravity, and collisions
- **Gazebo Harmonic** is the modern simulator for ROS 2 with improved physics
- **URDF converts to SDF** for Gazebo, preserving robot structure
- **World files** define the complete simulation environment
- **ros_gz_bridge** connects Gazebo topics to ROS 2

## What's Next

In [Chapter 2: Digital Twins & HRI in Unity](./chapter-2-unity), you'll learn to:
- Create high-fidelity visual environments in Unity
- Set up ROS-Unity integration for bidirectional communication
- Build human-robot interaction scenarios

## Additional Resources

- [Gazebo Sim Documentation](https://gazebosim.org/docs/harmonic)
- [ros_gz GitHub Repository](https://github.com/gazebosim/ros_gz)
- [SDF Specification](http://sdformat.org/spec)
- [DART Physics Engine](https://dartsim.github.io/)
