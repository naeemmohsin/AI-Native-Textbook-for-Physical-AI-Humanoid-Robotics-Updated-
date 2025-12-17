# Quickstart: Module 2 Environment Setup

**Feature**: 002-digital-twin-module
**Date**: 2025-12-17
**Purpose**: Step-by-step environment setup for Gazebo and Unity digital twin development

## Prerequisites

Before starting Module 2, ensure you have:

- [x] Completed Module 1 (ROS 2 Fundamentals)
- [x] Ubuntu 22.04 LTS (native or WSL2)
- [x] ROS 2 Humble installed and sourced
- [x] Minimum 8 GB RAM (16 GB recommended)
- [x] GPU with OpenGL 3.3+ support (OpenGL 4.5 / Vulkan recommended)
- [x] 20 GB free disk space (50 GB recommended)

## Part 1: Gazebo Harmonic Installation

### 1.1 Install Gazebo Harmonic with ROS 2 Integration

```bash
# Update package index
sudo apt update

# Install Gazebo Harmonic via ROS 2 packages
sudo apt install ros-humble-ros-gz

# This installs:
# - Gazebo Harmonic (gz-sim8)
# - ros_gz_bridge
# - ros_gz_sim
# - ros_gz_image
```

### 1.2 Verify Gazebo Installation

```bash
# Check Gazebo version
gz sim --version
# Expected: Gazebo Sim, version 8.x.x

# Launch Gazebo with an empty world
gz sim empty.sdf

# In a new terminal, verify ROS-Gazebo bridge is available
ros2 pkg list | grep ros_gz
# Expected: ros_gz_bridge, ros_gz_sim, etc.
```

### 1.3 Test ROS-Gazebo Bridge

```bash
# Terminal 1: Start Gazebo with a sample world
gz sim shapes.sdf

# Terminal 2: List available Gazebo topics
gz topic -l

# Terminal 3: Bridge a topic to ROS 2
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock

# Terminal 4: Verify ROS 2 receives the clock
ros2 topic echo /clock
```

## Part 2: Unity Installation

### 2.1 Install Unity Hub

**On Windows/macOS:**
1. Download Unity Hub from [unity.com/download](https://unity.com/download)
2. Install and launch Unity Hub
3. Sign in with a Unity account (Personal license is free)

**On Ubuntu (for visualization only):**
```bash
# Add Unity Hub repository
wget -qO - https://hub.unity3d.com/linux/keys/public | sudo gpg --dearmor -o /usr/share/keyrings/unity-hub.gpg
echo "deb [signed-by=/usr/share/keyrings/unity-hub.gpg] https://hub.unity3d.com/linux/repos/deb stable main" | sudo tee /etc/apt/sources.list.d/unityhub.list

# Install Unity Hub
sudo apt update
sudo apt install unityhub
```

### 2.2 Install Unity 2022.3 LTS

1. Open Unity Hub
2. Go to **Installs** → **Install Editor**
3. Select **Unity 2022.3 LTS** (latest 2022.3.x version)
4. Add modules:
   - Linux Build Support (if on Ubuntu)
   - Documentation (optional)
5. Click **Install**

### 2.3 Create Robotics Project

1. In Unity Hub, click **New Project**
2. Select **3D (Built-in Render Pipeline)**
3. Name: `HumanoidDigitalTwin`
4. Create project

### 2.4 Install Unity Robotics Packages

1. Open **Window** → **Package Manager**
2. Click **+** → **Add package from git URL**
3. Add these packages one by one:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

```
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

4. Wait for packages to install

### 2.5 Configure ROS Connection

1. Go to **Robotics** → **ROS Settings**
2. Set **ROS IP Address**: `127.0.0.1` (or your ROS machine IP)
3. Set **ROS Port**: `10000`
4. Set **Protocol**: ROS2

## Part 3: ROS-Unity Bridge Setup

### 3.1 Install ROS TCP Endpoint

```bash
# Clone the ROS TCP Endpoint package
cd ~/ros2_ws/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

### 3.2 Launch ROS TCP Endpoint

```bash
# Start the endpoint server
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### 3.3 Test Unity-ROS Connection

1. In Unity, enter Play mode
2. Check the Console for connection status
3. In a ROS 2 terminal:

```bash
# List topics - should see Unity-related topics
ros2 topic list
```

## Part 4: Load Humanoid URDF

### 4.1 Prepare URDF from Module 1

```bash
# Ensure your humanoid URDF is accessible
# Example path: ~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf

# Verify URDF validity
check_urdf ~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf
```

### 4.2 Load URDF in Gazebo

```bash
# Convert URDF to SDF (if needed)
gz sdf -p humanoid.urdf > humanoid.sdf

# Create a launch file to spawn the robot
# See examples/module-2/chapter-1/humanoid_gazebo/launch/spawn_humanoid.launch.py
```

### 4.3 Load URDF in Unity

1. Copy your URDF file into Unity's `Assets/URDF/` folder
2. Right-click the URDF file → **Import Robot from URDF**
3. Configure import settings:
   - Axis Type: Y Axis
   - Mesh Decomposition: VHACD
4. Click **Import URDF**

## Part 5: Verification Checklist

### Gazebo Verification

- [ ] `gz sim --version` shows version 8.x.x
- [ ] `gz sim empty.sdf` launches without errors
- [ ] `ros2 pkg list | grep ros_gz` shows bridge packages
- [ ] Bridge successfully connects Gazebo and ROS 2 topics

### Unity Verification

- [ ] Unity 2022.3 LTS installed
- [ ] ROS-TCP-Connector package installed
- [ ] URDF-Importer package installed
- [ ] ROS Settings configured with correct IP/port

### Integration Verification

- [ ] ROS TCP Endpoint running
- [ ] Unity connects to ROS 2 (check Console logs)
- [ ] Topics visible in `ros2 topic list` from Unity
- [ ] URDF imports successfully in both Gazebo and Unity

## Troubleshooting

### Gazebo Issues

**Problem**: Gazebo crashes on startup
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
# Ensure version 3.3 or higher

# Try software rendering (slower but works)
export LIBGL_ALWAYS_SOFTWARE=1
gz sim empty.sdf
```

**Problem**: ros_gz packages not found
```bash
# Ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Reinstall ros_gz
sudo apt install --reinstall ros-humble-ros-gz
```

### Unity Issues

**Problem**: Package import fails
- Ensure Unity 2022.3 LTS is used
- Check internet connection for git packages
- Try importing packages from local clone

**Problem**: ROS connection fails
- Verify ROS TCP Endpoint is running
- Check firewall settings
- Ensure IP addresses match

### WSL2 Issues

**Problem**: Gazebo GUI doesn't display
```bash
# Install WSLg support (Windows 11)
wsl --update

# Or use VcXsrv with proper DISPLAY export
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

## Hardware Requirements Summary

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| RAM | 8 GB | 16 GB |
| GPU | OpenGL 3.3 | OpenGL 4.5 / Vulkan |
| GPU VRAM | 2 GB | 4 GB |
| CPU | 4 cores | 8 cores |
| Storage | 20 GB | 50 GB |
| OS | Ubuntu 22.04 / WSL2 | Ubuntu 22.04 native |

## Next Steps

After completing this quickstart:

1. **Chapter 1**: Learn physics simulation fundamentals with Gazebo
2. **Chapter 2**: Build high-fidelity digital twins with Unity
3. **Chapter 3**: Configure and validate sensor simulation

---

*Environment setup complete. Proceed to Chapter 1: Physics Simulation with Gazebo.*
