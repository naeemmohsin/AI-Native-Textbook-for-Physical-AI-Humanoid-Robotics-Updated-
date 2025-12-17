# Research: Module 2 – The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-module
**Date**: 2025-12-17
**Purpose**: Resolve technical decisions and document best practices for simulation chapter content

## Technology Decisions

### 1. Gazebo Version Selection

**Decision**: Gazebo Harmonic (Gazebo Sim 8.x)
**Rationale**:
- Latest LTS release paired with ROS 2 Humble
- Active development and community support
- Native ros_gz integration packages
- Modern plugin architecture for sensors
**Alternatives Considered**:
- Gazebo Classic (11.x): End-of-life, limited ROS 2 support
- Gazebo Fortress: Previous LTS, fewer features than Harmonic
- Gazebo Garden: Non-LTS, less stable for educational content

**Official Sources**:
- [Gazebo Releases](https://gazebosim.org/docs/harmonic/releases)
- [ros_gz Integration](https://github.com/gazebosim/ros_gz)

### 2. Unity Version and Robotics Packages

**Decision**: Unity 2022.3 LTS with Unity Robotics Hub
**Rationale**:
- Long-term support ensures stability for readers
- Unity Robotics Hub is official solution for ROS integration
- URDF Importer included for robot model loading
- Built-in support for ROS 2 message types
**Alternatives Considered**:
- Unity 2021 LTS: Older, fewer robotics features
- Unity 6: Too new, limited documentation
- Unreal Engine: Steeper learning curve, less ROS tooling

**Official Sources**:
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer)

### 3. ROS-Gazebo Bridge Architecture

**Decision**: Use ros_gz_bridge for ROS 2 ↔ Gazebo Sim communication
**Rationale**:
- Official bridge maintained by Open Robotics
- Supports all standard ROS 2 message types
- Bidirectional topic/service mapping
- Automatic message type conversion
**Implementation Pattern**:
```bash
# Bridge configuration
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model
```

**Official Sources**:
- [ros_gz_bridge Documentation](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)

### 4. Sensor Simulation Approach

**Decision**: Use gz-sensors library with noise configuration
**Rationale**:
- Native Gazebo Sim sensor implementations
- Configurable noise models (Gaussian, bias, drift)
- Standard ROS 2 message output
- GPU-accelerated where applicable (LiDAR, camera)

**Sensor Configuration Patterns**:

| Sensor | Gazebo Plugin | ROS Message Type | Key Parameters |
|--------|---------------|------------------|----------------|
| LiDAR | gz-sensors-lidar | sensor_msgs/PointCloud2 | range, samples, noise |
| Depth Camera | gz-sensors-rgbd_camera | sensor_msgs/Image | resolution, FOV, depth range |
| IMU | gz-sensors-imu | sensor_msgs/Imu | rate, noise_density, bias |

**Official Sources**:
- [gz-sensors Documentation](https://gazebosim.org/api/sensors/8/index.html)
- [Sensor Noise Models](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1Noise.html)

### 5. URDF-to-SDF Conversion

**Decision**: Use gz sdf tool for URDF conversion, maintain URDF as source
**Rationale**:
- URDF is ROS ecosystem standard for robot descriptions
- SDF required for Gazebo world features (physics, plugins)
- Automatic conversion preserves compatibility
- Module 1 humanoid URDF can be directly imported
**Conversion Command**:
```bash
gz sdf -p robot.urdf > robot.sdf
```

**Official Sources**:
- [URDF to SDF Migration](https://gazebosim.org/api/sdformat/14/sdf_urdf.html)

### 6. Unity-ROS Communication Protocol

**Decision**: TCP-based communication via ROS-TCP-Connector
**Rationale**:
- Lower latency than WebSocket alternatives
- Native binary serialization
- Supports large messages (images, point clouds)
- Works across network boundaries
**Architecture**:
```text
Unity App ←→ ROS-TCP-Connector ←→ ROS-TCP-Endpoint ←→ ROS 2 Network
```

**Official Sources**:
- [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

## Best Practices

### Gazebo World Design

1. **Modular World Files**: Separate robot models from world definition
2. **Physics Settings**: Use DART engine for humanoid contact dynamics
3. **Performance**: Limit polygon count, use simplified collision meshes
4. **Reproducibility**: Pin random seeds for deterministic simulation

### Unity Scene Organization

1. **Hierarchy**: Robot → Links → Joints (mirror URDF structure)
2. **Materials**: Use Standard Shader for broad compatibility
3. **Lighting**: Baked lighting for static scenes, real-time for dynamic
4. **Frame Rate**: Target 30 FPS minimum for smooth HRI visualization

### Sensor Configuration

1. **Noise Modeling**: Always include noise for realism
2. **Update Rates**: Match real sensor specifications
3. **Coordinate Frames**: Verify TF tree matches physical sensor placement
4. **Validation**: Compare simulated output to real sensor data sheets

## Dependencies Summary

| Component | Version | Installation |
|-----------|---------|--------------|
| ROS 2 | Humble | ros-humble-desktop |
| Gazebo Sim | Harmonic (8.x) | ros-humble-ros-gz |
| Unity | 2022.3 LTS | Unity Hub |
| Unity Robotics Hub | Latest | Unity Package Manager |
| ros_gz_bridge | Humble | ros-humble-ros-gz-bridge |
| gz-sensors | 8.x | Included with Gazebo Harmonic |

## Hardware Requirements

| Requirement | Minimum | Recommended |
|-------------|---------|-------------|
| RAM | 8 GB | 16 GB |
| GPU | OpenGL 3.3 | OpenGL 4.5 / Vulkan |
| Storage | 20 GB | 50 GB |
| CPU | 4 cores | 8 cores |
| GPU VRAM | 2 GB | 4 GB |
