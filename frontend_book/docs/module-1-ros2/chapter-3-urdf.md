---
sidebar_position: 4
title: "Chapter 3: Humanoid Description with URDF"
description: Define robot structure using URDF with visual, collision, and inertial properties for simulation and visualization
keywords: [ros2, urdf, robot description, links, joints, rviz2, tf2, robotics]
---

# Chapter 3: Humanoid Description with URDF

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Explain the purpose and structure of URDF (Unified Robot Description Format)
- Create links with visual, collision, and inertial properties
- Define joints connecting links with appropriate types and limits
- Understand coordinate frames and the TF2 transform system
- Build and visualize a robot arm in RViz2
:::

## Prerequisites

Before starting this chapter, ensure you have:
- Completed [Chapter 1: ROS 2 Fundamentals](./chapter-1-fundamentals.md)
- ROS 2 Humble installed with RViz2
- Basic understanding of 3D coordinate systems

## 1. Introduction to URDF

### What is URDF?

**URDF** (Unified Robot Description Format) is an XML format for describing a robot's physical structure. It defines:

- **Links**: The rigid bodies of the robot (arms, wheels, sensors)
- **Joints**: The connections between links (how they move relative to each other)
- **Visual properties**: How the robot looks (meshes, colors)
- **Collision properties**: Simplified geometry for physics simulation
- **Inertial properties**: Mass and inertia for dynamics simulation

### Why Robot Descriptions Matter

A proper robot description enables:

1. **Visualization**: See your robot in RViz2 and other tools
2. **Simulation**: Test code in Gazebo before deploying to hardware
3. **Motion Planning**: MoveIt and other planners need to know robot geometry
4. **Transform Broadcasting**: TF2 publishes coordinate frame relationships
5. **Collision Detection**: Prevent the robot from hitting itself or obstacles

### URDF in the ROS 2 Ecosystem

```text
┌─────────────┐
│  URDF File  │
└──────┬──────┘
       │
       ▼
┌──────────────────────┐
│ robot_state_publisher │──────▶ /robot_description (topic)
└──────────────────────┘         /tf, /tf_static (transforms)
       │
       ▼
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│    RViz2     │   │   Gazebo     │   │   MoveIt     │
└──────────────┘   └──────────────┘   └──────────────┘
```

## 2. URDF Structure

### 2.1 Links

A **link** represents a rigid body in the robot. Each link can have three types of elements:

```xml title="Link structure"
<link name="my_link">
  <visual>
    <!-- How the link appears (for visualization) -->
  </visual>
  <collision>
    <!-- Simplified geometry for physics -->
  </collision>
  <inertial>
    <!-- Mass and inertia properties -->
  </inertial>
</link>
```

**Link Anatomy:**

```text
┌────────────────────────────────────────────────────┐
│                      Link                           │
│                                                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐│
│  │   Visual    │  │  Collision  │  │  Inertial   ││
│  │             │  │             │  │             ││
│  │ - geometry  │  │ - geometry  │  │ - mass      ││
│  │ - material  │  │ (simplified)│  │ - inertia   ││
│  │ - origin    │  │ - origin    │  │ - origin    ││
│  └─────────────┘  └─────────────┘  └─────────────┘│
└────────────────────────────────────────────────────┘
```

### 2.2 Joints

A **joint** connects two links and defines how they move relative to each other.

```xml title="Joint structure"
<joint name="my_joint" type="revolute">
  <parent link="link_a"/>
  <child link="link_b"/>
  <origin xyz="0 0 1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

**Joint Types:**

| Type | Motion | Use Case |
|------|--------|----------|
| `revolute` | Rotation with limits | Elbow, shoulder |
| `continuous` | Unlimited rotation | Wheel, propeller |
| `prismatic` | Linear sliding | Elevator, telescope |
| `fixed` | No motion | Sensor mount |
| `floating` | 6-DOF (simulation) | Mobile base |
| `planar` | 2D translation | XY table |

```text
Joint Types Visualization:

revolute              continuous           prismatic            fixed
    │                     │                    │                  │
    ○───────────┐         ○                    ║                  ═══
    │ (limited) │         │ (unlimited)        ║ (sliding)       (rigid)
    └───────────┘         ↻                    ▼
```

### 2.3 Coordinate Frames

Every link has an associated **coordinate frame**. The **TF2** library tracks relationships between frames.

**Key concepts:**
- The `origin` element specifies the transform from parent to child
- `xyz` is the translation (x, y, z in meters)
- `rpy` is the rotation (roll, pitch, yaw in radians)
- `axis` defines the joint's axis of rotation/translation

```xml
<!-- Joint at 0.5m above parent, rotating around Z axis -->
<joint name="shoulder" type="revolute">
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

**Frame Tree Example:**

```text
world
  └── base_link
        ├── shoulder_link
        │     └── upper_arm_link
        │           └── elbow_link
        │                 └── forearm_link
        └── camera_link
```

## 3. Building a URDF Step-by-Step

### 3.1 Simple Link

Let's start with a single link - a box representing a robot base:

```xml title="simple_link.urdf"
<?xml version="1.0"?>
<robot name="simple_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0"
               izz="0.01"/>
    </inertial>
  </link>

</robot>
```

### 3.2 Two-Link Robot Arm

Now let's add a second link connected by a revolute joint:

```xml title="two_link_arm.urdf"
<?xml version="1.0"?>
<robot name="two_link_arm">

  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- First arm link -->
  <link name="link_1">
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.25"/>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base to link_1 -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <!-- Second arm link -->
  <link name="link_2">
    <visual>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <box size="0.04 0.04 0.4"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <box size="0.04 0.04 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.2"/>
      <mass value="0.3"/>
      <inertia ixx="0.005" ixy="0" ixz="0"
               iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Joint connecting link_1 to link_2 -->
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

</robot>
```

### 3.3 Humanoid Arm Structure

A more complete arm with shoulder, elbow, and wrist:

```xml title="humanoid_arm.urdf"
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <!-- Base/Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Shoulder link -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder joint (rotation around Y) -->
  <joint name="shoulder_pan" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0.15 0.1 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Upper arm -->
  <link name="upper_arm_link">
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.5 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder lift joint -->
  <joint name="shoulder_lift" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="1.0"/>
  </joint>

  <!-- Elbow link -->
  <link name="elbow_link">
    <visual>
      <geometry>
        <sphere radius="0.035"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0" ixz="0"
               iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="50" velocity="1.0"/>
  </joint>

  <!-- Forearm -->
  <link name="forearm_link">
    <visual>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 0.8 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125"/>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.125"/>
      <mass value="0.5"/>
      <inertia ixx="0.003" ixy="0" ixz="0"
               iyy="0.003" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Forearm joint -->
  <joint name="forearm_roll" type="continuous">
    <parent link="elbow_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Wrist/Hand -->
  <link name="hand_link">
    <visual>
      <geometry>
        <box size="0.08 0.06 0.02"/>
      </geometry>
      <material name="gray">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.06 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Wrist joint -->
  <joint name="wrist" type="revolute">
    <parent link="forearm_link"/>
    <child link="hand_link"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="20" velocity="1.5"/>
  </joint>

</robot>
```

## 4. Visual, Collision, and Inertial Elements

### 4.1 Visual Elements

Visual elements define how the robot appears:

**Geometry types:**

```xml
<!-- Primitive shapes -->
<geometry>
  <box size="1.0 0.5 0.2"/>           <!-- x, y, z dimensions -->
  <cylinder radius="0.1" length="0.5"/> <!-- radius and height -->
  <sphere radius="0.1"/>               <!-- radius -->
</geometry>

<!-- Mesh files (for detailed models) -->
<geometry>
  <mesh filename="package://my_robot/meshes/arm.stl" scale="1 1 1"/>
</geometry>
```

**Materials and colors:**

```xml
<material name="red">
  <color rgba="1.0 0.0 0.0 1.0"/>  <!-- red, green, blue, alpha -->
</material>

<!-- Or reference a texture -->
<material name="wood">
  <texture filename="package://my_robot/textures/wood.png"/>
</material>
```

### 4.2 Collision Elements

Collision elements are used for physics simulation. Use simplified geometry:

```xml
<!-- Visual: detailed mesh -->
<visual>
  <geometry>
    <mesh filename="package://robot/meshes/detailed_arm.dae"/>
  </geometry>
</visual>

<!-- Collision: simplified box (faster physics) -->
<collision>
  <geometry>
    <box size="0.1 0.1 0.5"/>
  </geometry>
</collision>
```

:::tip
Always use simpler geometry for collision than visual. A complex mesh with thousands of triangles will slow down physics simulation significantly.
:::

### 4.3 Inertial Elements

Inertial properties are required for dynamics simulation:

```xml
<inertial>
  <origin xyz="0 0 0.1"/>  <!-- Center of mass -->
  <mass value="1.0"/>       <!-- Mass in kg -->
  <inertia ixx="0.01" ixy="0" ixz="0"
           iyy="0.01" iyz="0"
           izz="0.01"/>     <!-- Inertia tensor -->
</inertial>
```

**Common Inertia Formulas:**

| Shape | Formula (about center) |
|-------|------------------------|
| Box | Ixx = m(y² + z²)/12, Iyy = m(x² + z²)/12, Izz = m(x² + y²)/12 |
| Cylinder (Z-axis) | Ixx = Iyy = m(3r² + h²)/12, Izz = mr²/2 |
| Sphere | Ixx = Iyy = Izz = 2mr²/5 |

## 5. URDF Integration with ROS 2

### 5.1 robot_state_publisher

The `robot_state_publisher` node reads your URDF and:
- Publishes it to `/robot_description`
- Broadcasts TF transforms for all links

```python title="display.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'robot.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Joint state publisher (for manually setting joint angles)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_robot'),
                'rviz',
                'display.rviz'
            )],
        ),
    ])
```

### 5.2 Visualization with RViz2

To view your robot in RViz2:

1. Launch `robot_state_publisher` with your URDF
2. Open RViz2
3. Add a "RobotModel" display
4. Set "Description Topic" to `/robot_description`
5. Add a "TF" display to see coordinate frames

### 5.3 Verifying Your URDF

Use `check_urdf` to validate URDF syntax:

```bash
# Install urdf parser
sudo apt install liburdfdom-tools

# Check URDF file
check_urdf my_robot.urdf
```

View the kinematic tree:

```bash
urdf_to_graphiz my_robot.urdf
# This creates my_robot.pdf with a visual tree
```

## 6. Hands-On Exercise: Build and Visualize a Robot Arm

### Step 1: Create a Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_arm_description --dependencies rclpy
```

### Step 2: Add URDF Directory

```bash
mkdir -p robot_arm_description/urdf
mkdir -p robot_arm_description/launch
mkdir -p robot_arm_description/rviz
```

### Step 3: Create the URDF

Copy the `two_link_arm.urdf` from Section 3.2 to `robot_arm_description/urdf/robot.urdf`.

### Step 4: Create Launch File

Create `robot_arm_description/launch/display.launch.py`:

```python title="display.launch.py"
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_arm_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
        ),
    ])
```

### Step 5: Update setup.py

Add data files to `setup.py`:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'robot_arm_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robot arm URDF description',
    license='Apache-2.0',
)
```

### Step 6: Build and Launch

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select robot_arm_description
source install/setup.bash

# Launch
ros2 launch robot_arm_description display.launch.py
```

### Step 7: Configure RViz2

In RViz2:
1. Set "Fixed Frame" to `base_link`
2. Click "Add" → "RobotModel"
3. Click "Add" → "TF"
4. Use the Joint State Publisher GUI to move the joints!

## Summary

In this chapter, you learned:

- **URDF** is an XML format for describing robot physical structure
- **Links** are rigid bodies with visual, collision, and inertial properties
- **Joints** connect links and define their relative motion (revolute, prismatic, fixed, etc.)
- **Coordinate frames** are tracked by TF2, enabling transforms between robot parts
- **robot_state_publisher** reads URDF and broadcasts transforms
- **RViz2** visualizes robots using the published description

## Further Reading

- [URDF Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Building a Visual Robot Model](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
- [TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [robot_state_publisher](https://github.com/ros/robot_state_publisher)

---

**Congratulations!** You've completed Module 1: The Robotic Nervous System. You now understand ROS 2 architecture, can write Python nodes with rclpy, and can describe robots using URDF.

**Next Steps:**
- Practice by building more complex URDF models
- Explore Gazebo simulation with your robot descriptions
- Learn about MoveIt for motion planning
