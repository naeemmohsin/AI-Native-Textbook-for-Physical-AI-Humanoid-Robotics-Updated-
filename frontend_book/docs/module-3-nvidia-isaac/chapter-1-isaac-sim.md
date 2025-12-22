---
sidebar_position: 1
title: "Chapter 1: NVIDIA Isaac Sim"
description: "Set up photorealistic simulation and synthetic data generation with NVIDIA Isaac Sim"
---

# Chapter 1: NVIDIA Isaac Sim

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the role of Isaac Sim in the NVIDIA robotics ecosystem
2. Install and configure Isaac Sim with ROS 2 Humble
3. Load humanoid robot models (URDF/USD) with realistic physics
4. Configure sensors and ROS 2 bridge using action graphs
5. Generate synthetic training data with Omniverse Replicator

## Prerequisites

Before starting this chapter, ensure you have:

- Completed Module 1 (ROS 2 Fundamentals) and Module 2 (Digital Twin)
- An NVIDIA RTX GPU (2070 or higher) with 8+ GB VRAM
- Ubuntu 22.04 LTS with NVIDIA drivers 525+
- At least 50 GB of free disk space

:::caution Hardware Requirements
Isaac Sim requires significant GPU resources. Running on systems below minimum specifications will result in poor performance or failure to launch.
:::

## Introduction to Isaac Sim

NVIDIA Isaac Sim is a photorealistic, physically accurate robotics simulation platform built on NVIDIA Omniverse. While Gazebo provides excellent real-time simulation for algorithm development, Isaac Sim adds capabilities essential for perception and AI training workflows.

### The Omniverse Platform

Isaac Sim runs on NVIDIA Omniverse, a platform for building and connecting 3D applications. Omniverse provides:

- **RTX Rendering**: Ray-traced lighting and materials for photorealism
- **PhysX 5**: High-fidelity physics simulation with GPU acceleration
- **USD Format**: Universal Scene Description for asset interoperability
- **Connectors**: Integration with DCC tools like Blender and Maya

### Isaac Sim vs. Gazebo

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| Rendering | OpenGL/Ogre | RTX ray tracing |
| Physics | ODE/Bullet/DART | PhysX 5 |
| Synthetic Data | Limited | Full pipeline |
| GPU Acceleration | Partial | Native |
| ROS 2 Integration | Native | Via bridge |
| Learning Curve | Moderate | Steeper |

**When to use Isaac Sim**:
- Training perception models requiring photorealistic data
- Testing vision-based navigation algorithms
- Generating diverse synthetic training datasets
- Sim-to-real transfer for deep learning

**When to use Gazebo**:
- Rapid algorithm prototyping
- Control system development
- Lower hardware requirements
- Native ROS 2 workflows

## Installation and Setup

### System Requirements Verification

First, verify your system meets the requirements:

```bash
# Check NVIDIA driver version (should be 525+)
nvidia-smi

# Check available GPU memory
nvidia-smi --query-gpu=memory.total --format=csv

# Verify Ubuntu version
lsb_release -a
```

### Installing Omniverse Launcher

1. Download Omniverse Launcher from [NVIDIA's website](https://www.nvidia.com/en-us/omniverse/)

2. Install the launcher:

```bash
# Make the AppImage executable
chmod +x omniverse-launcher-linux.AppImage

# Run the launcher
./omniverse-launcher-linux.AppImage
```

3. Sign in with your NVIDIA Developer account

4. Navigate to the **Exchange** tab and install:
   - **Cache** (for asset caching)
   - **Nucleus** (for collaboration server)

### Installing Isaac Sim

1. In Omniverse Launcher, go to **Exchange** → **Apps**

2. Search for "Isaac Sim" and click **Install**

3. Select version **4.0.0** or later for ROS 2 Humble compatibility

4. Wait for download and installation (approximately 15-20 GB)

5. Launch Isaac Sim from the **Library** tab

### Nucleus Server Configuration

Nucleus provides asset management and collaboration. For local development:

1. Open Omniverse Launcher → **Nucleus** tab

2. Click **Add Local Nucleus Service**

3. Create a local server with default settings

4. Note the connection URL (typically `omniverse://localhost/`)

:::tip First Launch
The first launch takes several minutes as shaders compile. Subsequent launches are faster.
:::

## USD Scene Fundamentals

Isaac Sim uses Universal Scene Description (USD), developed by Pixar, as its native scene format. Understanding USD is essential for creating and modifying simulation environments.

### What is USD?

USD is a framework for describing, composing, and reading hierarchical scene data. Key concepts:

- **Stage**: The root container for a scene
- **Prims**: The basic building blocks (meshes, lights, cameras)
- **Properties**: Attributes and relationships on prims
- **Layers**: Files that can be composed together

### Stage Hierarchy

A typical Isaac Sim scene has this hierarchy:

```text
/World
├── /Environment
│   ├── /Ground
│   ├── /Lights
│   └── /Props
├── /Robots
│   └── /Humanoid
│       ├── /base_link
│       ├── /torso
│       └── /sensors
└── /Cameras
    ├── /MainCamera
    └── /RobotCamera
```

### Creating a Basic Scene

In Isaac Sim's Script Editor (Window → Script Editor):

```python
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core import World

# Create a new world
world = World(stage_units_in_meters=1.0)

# Add a ground plane
world.scene.add_default_ground_plane()

# Set physics parameters
physics_context = world.get_physics_context()
physics_context.set_gravity(-9.81)

# Initialize the world
await world.initialize_async()
```

### Physics Configuration

Configure physics for humanoid simulation:

```python
from pxr import UsdPhysics, PhysxSchema

# Get the physics scene
physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

# Set simulation parameters
physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
physx_scene.CreateTimeStepsPerSecondAttr(120)  # 120 Hz physics
physx_scene.CreateEnableGPUDynamicsAttr(True)
physx_scene.CreateEnableCCDAttr(True)  # Continuous collision detection
```

## Loading Humanoid Robots

Isaac Sim supports loading robots from URDF or native USD format. For humanoids, USD provides better articulation control.

### URDF to USD Conversion

Convert your existing URDF models using Isaac Sim's converter:

```python
from omni.isaac.urdf import _urdf

# Configure the URDF importer
urdf_interface = _urdf.acquire_urdf_interface()

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False  # Humanoid needs free base
import_config.make_default_prim = True
import_config.create_physics_scene = True

# Import the URDF
result = urdf_interface.parse_urdf(
    urdf_path="/path/to/humanoid.urdf",
    import_config=import_config
)

# Save as USD
urdf_interface.import_robot(
    urdf_path="/path/to/humanoid.urdf",
    import_config=import_config,
    dest_path="/World/Robots/Humanoid"
)
```

### Articulation Configuration

Configure the robot's articulation for control:

```python
from omni.isaac.core.articulations import Articulation

# Get the robot articulation
humanoid = Articulation(prim_path="/World/Robots/Humanoid")
humanoid.initialize()

# Get joint information
joint_names = humanoid.dof_names
num_dof = humanoid.num_dof

print(f"Robot has {num_dof} degrees of freedom")
print(f"Joints: {joint_names}")

# Set joint positions
import numpy as np
default_positions = np.zeros(num_dof)
humanoid.set_joint_positions(default_positions)
```

### Adding Sensors

Attach sensors to your humanoid:

```python
from omni.isaac.sensor import Camera, IMUSensor, ContactSensor

# Add RGB-D camera to head
camera = Camera(
    prim_path="/World/Robots/Humanoid/head/Camera",
    frequency=30,
    resolution=(640, 480)
)

# Add IMU to torso
imu = IMUSensor(
    prim_path="/World/Robots/Humanoid/torso/IMU",
    name="torso_imu",
    frequency=200
)

# Add contact sensors to feet
left_foot_contact = ContactSensor(
    prim_path="/World/Robots/Humanoid/left_foot/ContactSensor",
    name="left_foot_contact",
    frequency=120
)
```

### Material Assignment

Apply realistic materials for perception training:

```python
from omni.isaac.core.materials import PreviewSurface

# Create a metallic material for robot body
robot_material = PreviewSurface(
    prim_path="/World/Looks/RobotMetal",
    color=np.array([0.7, 0.7, 0.8]),
    metallic=0.9,
    roughness=0.3
)

# Apply to robot mesh
robot_mesh = stage.GetPrimAtPath("/World/Robots/Humanoid/torso/mesh")
robot_material.apply_material(robot_mesh)
```

## ROS 2 Bridge Configuration

Isaac Sim communicates with ROS 2 through Action Graphs, a visual programming system for defining data flow.

### Action Graph Fundamentals

Action Graphs define:
- **Event triggers** (on playback tick, on physics step)
- **Data sources** (sensor readings, joint states)
- **ROS 2 publishers/subscribers**
- **Data transformations**

### Enabling the ROS 2 Bridge

1. Open Window → Extensions

2. Search for "ROS2 Bridge" and enable it

3. The extension adds ROS 2 nodes to the Action Graph palette

### Creating a ROS 2 Bridge Graph

Build an action graph for sensor publishing:

```python
import omni.graph.core as og

# Create action graph
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
            ("ReadCameraInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("PublishImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
            ("PublishCameraInfo", "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"),
            ("ReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
            ("PublishIMU", "omni.isaac.ros2_bridge.ROS2PublishImu"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ReadCameraInfo.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishImage.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("ReadCameraInfo.inputs:cameraPrim", "/World/Robots/Humanoid/head/Camera"),
            ("PublishImage.inputs:topicName", "/camera/image_raw"),
            ("PublishCameraInfo.inputs:topicName", "/camera/camera_info"),
            ("PublishIMU.inputs:topicName", "/imu/data"),
        ],
    }
)
```

### Publishing Sensor Data

Configure publishers for common sensor types:

| Sensor | ROS 2 Topic | Message Type |
|--------|-------------|--------------|
| RGB Camera | `/camera/image_raw` | sensor_msgs/Image |
| Depth Camera | `/camera/depth` | sensor_msgs/Image |
| Camera Info | `/camera/camera_info` | sensor_msgs/CameraInfo |
| LiDAR | `/scan` | sensor_msgs/LaserScan |
| IMU | `/imu/data` | sensor_msgs/Imu |
| Joint States | `/joint_states` | sensor_msgs/JointState |

### Subscribing to Joint Commands

Receive commands from your ROS 2 controller:

```python
# Add subscriber node to action graph
og.Controller.edit(
    {"graph_path": "/World/ActionGraph"},
    {
        keys.CREATE_NODES: [
            ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ],
        keys.SET_VALUES: [
            ("SubscribeJointState.inputs:topicName", "/joint_commands"),
            ("ArticulationController.inputs:robotPath", "/World/Robots/Humanoid"),
        ],
    }
)
```

## Synthetic Data Generation

Omniverse Replicator enables generating diverse training data with automatic annotations.

### Replicator Overview

Replicator provides:
- **Domain randomization**: Vary lighting, textures, poses
- **Ground truth**: Segmentation, depth, bounding boxes, poses
- **Orchestration**: Script-based data generation pipelines
- **Export**: Multiple formats for ML frameworks

### Basic Replicator Pipeline

Create a synthetic data generation script:

```python
import omni.replicator.core as rep

# Define camera for data capture
camera = rep.create.camera(
    position=(2.0, 2.0, 1.5),
    look_at=(0, 0, 0.5)
)

# Create render product
render_product = rep.create.render_product(camera, (1280, 720))

# Configure output writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/data/synthetic_dataset",
    rgb=True,
    semantic_segmentation=True,
    instance_segmentation=True,
    distance_to_camera=True,
    bounding_box_2d_tight=True,
)
writer.attach([render_product])

# Define randomization
with rep.trigger.on_frame(num_frames=1000):
    # Randomize lighting
    with rep.create.light(light_type="dome"):
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 2000))
        rep.modify.attribute("color", rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1)))

    # Randomize robot pose
    with rep.get.prims(path_pattern="/World/Robots/Humanoid"):
        rep.modify.pose(
            rotation=rep.distribution.uniform((-10, -180, -10), (10, 180, 10))
        )
```

### Domain Randomization

Apply randomization for robust training:

```python
# Texture randomization
def randomize_textures():
    materials = rep.get.prims(path_pattern="/World/Looks/*")
    with materials:
        rep.modify.attribute(
            "inputs:diffuse_color_constant",
            rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9))
        )

# Background randomization
def randomize_background():
    backgrounds = [
        "omniverse://localhost/NVIDIA/Assets/Skies/Clear/noon_grass_4k.hdr",
        "omniverse://localhost/NVIDIA/Assets/Skies/Cloudy/evening_road_4k.hdr",
    ]
    rep.modify.attribute("inputs:texture", rep.distribution.choice(backgrounds))

rep.randomizer.register(randomize_textures)
rep.randomizer.register(randomize_background)
```

### Ground Truth Export

Configure annotation export:

```python
# Enable semantic labels
rep.modify.semantics([
    ("class", "robot"),
    ("class", "floor"),
    ("class", "obstacle"),
])

# Export formats
writer = rep.WriterRegistry.get("KittiWriter")  # For object detection
# or
writer = rep.WriterRegistry.get("CocoWriter")   # For segmentation
```

## Hands-On Exercise

Create a complete Isaac Sim scene with a humanoid robot and ROS 2 integration.

### Step 1: Create the Scene

1. Launch Isaac Sim from Omniverse Launcher

2. Create a new stage: File → New

3. Add a ground plane: Create → Physics → Ground Plane

4. Save the stage as `humanoid_scene.usd`

### Step 2: Load the Humanoid

1. Open the Script Editor: Window → Script Editor

2. Run the URDF import script from the Loading Humanoid Robots section

3. Alternatively, load a sample robot: Isaac Examples → Robots → Humanoid

### Step 3: Configure Sensors

1. Add cameras to the robot's head following the sensor configuration examples

2. Add an IMU sensor to the torso

3. Configure sensor frequencies and resolutions

### Step 4: Set Up ROS 2 Bridge

1. Enable the ROS 2 Bridge extension

2. Create an Action Graph with the ROS 2 bridge configuration

3. Configure publishers for camera images, IMU data, and joint states

### Step 5: Verify Data Flow

1. Start simulation: Play button or `Space`

2. In a terminal, check ROS 2 topics:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# List available topics
ros2 topic list

# Verify camera data
ros2 topic hz /camera/image_raw

# Check IMU data
ros2 topic echo /imu/data --once
```

### Success Criteria

You have successfully completed this exercise when:

- [ ] Isaac Sim launches without errors
- [ ] Humanoid robot is visible in the viewport
- [ ] Simulation runs at stable frame rate (30+ FPS)
- [ ] ROS 2 topics are publishing sensor data
- [ ] Camera images are viewable in RViz2

## Key Takeaways

1. **Isaac Sim extends Gazebo** capabilities with photorealistic rendering and synthetic data generation for perception training

2. **USD format** provides powerful scene composition and asset management for complex robotics simulations

3. **Action Graphs** connect Isaac Sim to ROS 2, enabling bidirectional communication for sensors and actuators

4. **Omniverse Replicator** automates synthetic data generation with domain randomization for robust ML training

5. **GPU acceleration** is essential - Isaac Sim requires NVIDIA RTX hardware for real-time performance

## Next Steps

In Chapter 2, you'll learn to:
- Set up Isaac ROS for GPU-accelerated perception
- Implement Visual SLAM using cuVSLAM
- Build perception pipelines for object detection
- Integrate perception outputs with your simulation

[Continue to Chapter 2: Isaac ROS for Perception →](./chapter-2-isaac-ros)
