# Research: Module 3 – The AI-Robot Brain (NVIDIA Isaac™)

**Feature**: 003-nvidia-isaac-module
**Date**: 2025-12-21
**Purpose**: Technology decisions and best practices for NVIDIA Isaac documentation

## Technology Decisions

### TD-001: Isaac Sim Version Selection

**Decision**: NVIDIA Isaac Sim 4.0+ (2024.1.0 or later) with Omniverse

**Rationale**:
- Latest LTS release with improved ROS 2 Humble support
- Native USD (Universal Scene Description) format for assets
- Enhanced synthetic data generation (Replicator)
- Better performance on RTX GPUs
- Official NVIDIA documentation and tutorials available

**Alternatives Considered**:
- Isaac Sim 2023.x: Older version, less features
- Gazebo alone: Lacks photorealistic rendering and synthetic data capabilities

### TD-002: Isaac ROS Version Selection

**Decision**: Isaac ROS 3.0 (DP3 or later) for ROS 2 Humble

**Rationale**:
- Hardware-accelerated computer vision pipelines
- Native NVIDIA GPU integration (CUDA, TensorRT)
- Pre-built packages for VSLAM, AprilTag detection, DNN inference
- Jetson and x86 support
- Active development and community support

**Alternatives Considered**:
- Standard ROS 2 perception stack: No GPU acceleration
- Custom CUDA implementations: Higher development effort

### TD-003: Visual SLAM Implementation

**Decision**: Isaac ROS Visual SLAM (cuVSLAM)

**Rationale**:
- GPU-accelerated stereo visual odometry
- Real-time performance (30+ FPS on Jetson)
- Loop closure and relocalization support
- Integration with Isaac Sim for testing
- Official NVIDIA support and documentation

**Alternatives Considered**:
- ORB-SLAM3: CPU-based, slower on embedded platforms
- RTAB-Map: Good but heavier resource usage
- Kimera-VIO: Academic, less production support

### TD-004: Navigation Stack

**Decision**: Nav2 with Isaac ROS integration

**Rationale**:
- Industry-standard ROS 2 navigation stack
- Modular architecture (planners, controllers, behaviors)
- Active community and extensive documentation
- Works with Isaac ROS perception outputs
- Configurable for different robot types including humanoids

**Alternatives Considered**:
- Custom navigation: High development effort
- move_base (ROS 1): Not compatible with ROS 2

### TD-005: Synthetic Data Generation

**Decision**: NVIDIA Omniverse Replicator

**Rationale**:
- Integrated with Isaac Sim
- Domain randomization for training data
- Ground truth annotation (segmentation, depth, bounding boxes)
- Scriptable data generation pipelines
- High-quality photorealistic output

**Alternatives Considered**:
- Custom Unity pipeline: More setup, less NVIDIA integration
- Gazebo synthetic data: Lower visual quality

## Hardware Requirements

### Minimum Requirements

| Component | Specification |
|-----------|---------------|
| GPU | NVIDIA RTX 2070 or higher |
| VRAM | 8 GB minimum |
| CPU | 8-core, 3.0 GHz |
| RAM | 32 GB |
| Storage | 50 GB SSD |
| OS | Ubuntu 22.04 LTS |

### Recommended Requirements

| Component | Specification |
|-----------|---------------|
| GPU | NVIDIA RTX 3080 or higher |
| VRAM | 12 GB or more |
| CPU | 12-core, 3.5 GHz |
| RAM | 64 GB |
| Storage | 100 GB NVMe SSD |
| OS | Ubuntu 22.04 LTS |

### Jetson Platform Support

| Platform | Isaac Sim | Isaac ROS | Nav2 |
|----------|-----------|-----------|------|
| Jetson AGX Orin | Via remote | Yes | Yes |
| Jetson Orin Nano | No | Yes | Yes |
| Jetson AGX Xavier | Via remote | Yes | Yes |

## Best Practices

### BP-001: Isaac Sim Scene Organization

- Use USD stage hierarchy for logical grouping
- Separate robot, environment, and sensor assets
- Use action graphs for ROS 2 bridge configuration
- Enable physics at appropriate simulation rates (60-120 Hz)

### BP-002: Isaac ROS Pipeline Design

- Use Nitros for zero-copy GPU message passing
- Chain nodes to minimize CPU-GPU transfers
- Profile with nvprof/nsight for bottlenecks
- Use managed nodes for lifecycle control

### BP-003: Nav2 Configuration for Humanoids

- Configure footprint for bipedal gait patterns
- Adjust costmap inflation for humanoid dimensions
- Use velocity smoothing for stable locomotion
- Consider balance constraints in local planner

### BP-004: Synthetic Data Quality

- Vary lighting conditions for robustness
- Include domain randomization (textures, colors, positions)
- Generate diverse camera viewpoints
- Include edge cases (occlusion, motion blur)

## Integration Patterns

### Isaac Sim to ROS 2

```text
Isaac Sim (Omniverse)
    │
    ├── OmniGraph Action Graph
    │       │
    │       ├── ROS2 Camera Helper
    │       ├── ROS2 Lidar Helper
    │       └── ROS2 Clock Publisher
    │
    └── ROS2 Bridge
            │
            └── ROS 2 Topics (/camera/*, /scan, /clock)
```

### Isaac ROS Perception Pipeline

```text
Camera Input (/image_raw)
    │
    ├── Isaac ROS Image Pipeline
    │       ├── Rectification
    │       └── Resize
    │
    ├── Isaac ROS Visual SLAM
    │       └── /visual_slam/tracking/odometry
    │
    └── Isaac ROS DNN Inference
            └── /detections
```

### Nav2 Integration

```text
Perception
    ├── /visual_slam/tracking/odometry
    ├── /scan (LiDAR)
    └── /camera/depth (depth image)
            │
            └── Nav2 Stack
                    ├── AMCL / SLAM Toolbox
                    ├── Global Planner
                    ├── Local Planner
                    └── Behavior Server
                            │
                            └── /cmd_vel → Robot
```

## Dependencies

### Software Stack

| Package | Version | Purpose |
|---------|---------|---------|
| NVIDIA Isaac Sim | 4.0+ | Photorealistic simulation |
| NVIDIA Omniverse | Latest | USD runtime, rendering |
| Isaac ROS | 3.0 DP3+ | GPU-accelerated perception |
| ROS 2 Humble | Humble Hawksbill | Robot middleware |
| Nav2 | Humble release | Navigation stack |
| cuDNN | 8.9+ | Deep learning inference |
| TensorRT | 8.6+ | Inference optimization |

### Python Dependencies

- numpy >= 1.24
- opencv-python >= 4.8
- torch (for custom models)
- omni.isaac.core (Isaac Sim Python API)

## Open Questions (Resolved)

| Question | Resolution |
|----------|------------|
| Which Isaac Sim version? | 4.0+ for best ROS 2 Humble support |
| VSLAM or LiDAR SLAM? | Both covered; VSLAM primary, LiDAR optional |
| Jetson or x86 focus? | Both supported; x86 for development, Jetson for deployment |
| Nav2 planner selection? | Document multiple options (Navfn, Smac, etc.) |
