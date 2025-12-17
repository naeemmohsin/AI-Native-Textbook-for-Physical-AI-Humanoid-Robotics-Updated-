---
sidebar_position: 3
title: "Chapter 2: Digital Twins & HRI in Unity"
description: "Build high-fidelity digital twins and human-robot interaction scenarios with Unity and ROS 2"
---

# Chapter 2: Digital Twins & HRI in Unity

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain when to use Unity vs. Gazebo for robotics simulation
- Set up Unity 2022 LTS with ROS 2 integration using Unity Robotics Hub
- Import and render robot models using the URDF Importer
- Implement bidirectional ROS-Unity communication
- Create basic human-robot interaction scenarios

## Prerequisites

Before starting this chapter, ensure you have:

- Completed Chapter 1 (Gazebo physics simulation)
- Unity Hub installed with Unity 2022.3 LTS
- ROS 2 Humble running (Ubuntu or WSL2)
- Basic understanding of C# programming

---

## Unity for Robotics

While Gazebo excels at physics simulation, Unity brings photorealistic rendering and advanced human-robot interaction (HRI) capabilities to robotics development. Understanding when to use each platform is key to efficient development.

### When to Choose Unity

Unity is the better choice when you need:

| Requirement | Why Unity |
|-------------|-----------|
| **Photorealistic visuals** | Advanced lighting, PBR materials, post-processing |
| **Human-robot interaction** | Human avatars, animation, realistic environments |
| **Training data generation** | Domain randomization, synthetic datasets |
| **VR/AR integration** | Native XR support |
| **Stakeholder demos** | Visual appeal for non-technical audiences |

### When to Stay with Gazebo

Gazebo remains the better choice for:

- Pure physics testing and validation
- Sensor simulation with accurate noise models
- Control algorithm development
- Real-time factor simulation (matching real-world time)

### Using Both Together

Many teams use both platforms:
1. **Gazebo** for control algorithm development and sensor simulation
2. **Unity** for visualization, HRI testing, and demonstrations
3. **ROS 2** as the common communication layer

---

## Setting Up Unity Environment

### Installing Unity Hub

1. Download Unity Hub from [unity.com/download](https://unity.com/download)
2. Install and sign in (Personal license is free for learning)
3. Navigate to **Installs** → **Install Editor**
4. Select **Unity 2022.3 LTS** (latest 2022.3.x version)

### Creating a Robotics Project

1. In Unity Hub, click **New Project**
2. Select **3D (Built-in Render Pipeline)**
3. Name your project: `HumanoidDigitalTwin`
4. Click **Create Project**

### Installing Unity Robotics Packages

The Unity Robotics Hub provides official packages for ROS integration:

1. Open **Window** → **Package Manager**
2. Click **+** → **Add package from git URL**
3. Add the ROS-TCP-Connector:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

4. Add the URDF Importer:

```
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

5. Wait for packages to install (check Console for errors)

### Verifying Installation

After installation:
- **Robotics** menu appears in the menu bar
- **URDF Import Settings** window available
- **ROS Settings** accessible via Robotics menu

---

## ROS-Unity Integration

The ROS-TCP-Connector enables bidirectional communication between Unity and ROS 2. Messages flow through a TCP connection managed by the ROS-TCP-Endpoint on the ROS side.

### Architecture Overview

```text
┌─────────────────┐         TCP          ┌─────────────────┐
│                 │  ←───────────────→   │                 │
│  Unity App      │    Port 10000        │  ROS 2 Network  │
│  (C# Scripts)   │                      │  (Python/C++)   │
│                 │                      │                 │
└─────────────────┘                      └─────────────────┘
        ↓                                        ↓
  ROS-TCP-Connector                      ROS-TCP-Endpoint
   (Unity Package)                        (ROS 2 Package)
```

### Setting Up ROS-TCP-Endpoint

On your ROS 2 system:

```bash
# Clone the ROS-TCP-Endpoint package
cd ~/ros2_ws/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

# Launch the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Configuring Unity ROS Settings

1. Open **Robotics** → **ROS Settings**
2. Configure connection:
   - **Protocol**: ROS2
   - **ROS IP Address**: `127.0.0.1` (or your ROS machine IP)
   - **ROS Port**: `10000`

### Creating a ROS Connection Script

```csharp title="examples/module-2/chapter-2/unity_project/Scripts/ROSConnection.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSConnectionManager : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        // Get the ROS connection singleton
        ros = ROSConnection.GetOrCreateInstance();

        // Register publisher for a test topic
        ros.RegisterPublisher<StringMsg>("/unity/status");

        // Subscribe to receive messages
        ros.Subscribe<StringMsg>("/ros/commands", OnCommandReceived);

        // Publish initial status
        ros.Publish("/unity/status", new StringMsg("Unity connected"));
        Debug.Log("ROS connection established");
    }

    void OnCommandReceived(StringMsg msg)
    {
        Debug.Log($"Received from ROS: {msg.data}");
    }
}
```

---

## Importing Robot Models

The URDF Importer converts URDF robot descriptions into Unity GameObjects with proper joint hierarchies and physics components.

### Using the URDF Importer

1. Copy your URDF file and meshes into `Assets/URDF/`
2. In Unity, navigate to the URDF file
3. Right-click → **Import Robot from URDF**
4. Configure import settings:

| Setting | Recommended Value |
|---------|-------------------|
| **Axis Type** | Y Axis |
| **Mesh Decomposition** | VHACD |
| **Convex Mesh** | On (for collisions) |

5. Click **Import URDF**

### Understanding the Import Result

The importer creates:
- **GameObject hierarchy** matching URDF link structure
- **ArticulationBody** components for physics
- **Mesh renderers** for visuals
- **Colliders** for physics interactions

### Adding Materials

Default imported models may appear grey. To add materials:

1. Create materials: **Assets** → **Create** → **Material**
2. Configure material properties (color, texture, metallic)
3. Assign materials to link renderers

```csharp title="examples/module-2/chapter-2/unity_project/Scripts/RobotController.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotController : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Get all articulation bodies (joints)
        joints = GetComponentsInChildren<ArticulationBody>();

        // Subscribe to joint state commands
        ros.Subscribe<JointStateMsg>("/joint_commands", OnJointCommand);

        Debug.Log($"Robot initialized with {joints.Length} joints");
    }

    void OnJointCommand(JointStateMsg msg)
    {
        // Apply joint positions from ROS message
        for (int i = 0; i < msg.position.Length && i < joints.Length; i++)
        {
            var drive = joints[i].xDrive;
            drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
            joints[i].xDrive = drive;
        }
    }
}
```

---

## Creating Realistic Environments

Unity's strength lies in creating visually appealing environments that enhance HRI testing and demonstrations.

### Scene Setup Basics

1. **Lighting**: Use a directional light as the sun
2. **Skybox**: Add atmosphere with Window → Rendering → Lighting
3. **Ground plane**: Create a plane with appropriate material
4. **Post-processing**: Add bloom, ambient occlusion for realism

### Materials and Textures

For realistic materials:
- Use **Standard Shader** for broad compatibility
- Set **Metallic** and **Smoothness** for PBR effects
- Add **Normal maps** for surface detail

### Performance Considerations

For smooth HRI testing:
- Target 30+ FPS minimum
- Use **LOD Groups** for complex models
- Limit real-time shadows
- Use **occlusion culling** for large scenes

---

## Bidirectional Communication

Effective digital twins require synchronized state between Unity and ROS 2.

### Publishing from Unity

```csharp
// Publish robot state to ROS
void PublishRobotState()
{
    var jointState = new JointStateMsg
    {
        name = jointNames,
        position = GetJointPositions(),
        velocity = GetJointVelocities()
    };

    ros.Publish("/unity/joint_states", jointState);
}
```

### Subscribing in Unity

```csharp
// Subscribe and handle incoming messages
ros.Subscribe<TwistMsg>("/cmd_vel", (msg) =>
{
    // Apply velocity commands to robot
    ApplyVelocity(msg.linear, msg.angular);
});
```

### Service Calls

```csharp
// Call a ROS service from Unity
ros.SendServiceMessage<GetPlanRequest, GetPlanResponse>(
    "/get_plan",
    new GetPlanRequest { goal = targetPose },
    (response) =>
    {
        Debug.Log($"Received plan with {response.plan.poses.Length} waypoints");
    }
);
```

---

## Human-Robot Interaction Basics

HRI scenarios require humans and robots to interact in shared spaces safely and effectively.

### Adding Human Avatars

1. Import a humanoid avatar (Asset Store or custom)
2. Add **Animator** component with humanoid rig
3. Create animation states for walking, gesturing
4. Control animations via scripts or ROS messages

### Proximity Detection

```csharp title="examples/module-2/chapter-2/unity_project/Scripts/HRIManager.cs"
using UnityEngine;

public class HRIManager : MonoBehaviour
{
    public Transform robot;
    public Transform human;
    public float safetyDistance = 1.5f;

    private bool isInProximity = false;

    void Update()
    {
        float distance = Vector3.Distance(robot.position, human.position);

        if (distance < safetyDistance && !isInProximity)
        {
            isInProximity = true;
            OnHumanApproach();
        }
        else if (distance >= safetyDistance && isInProximity)
        {
            isInProximity = false;
            OnHumanDepart();
        }
    }

    void OnHumanApproach()
    {
        Debug.Log("Human entered safety zone");
        // Slow down robot, trigger safety behavior
    }

    void OnHumanDepart()
    {
        Debug.Log("Human left safety zone");
        // Resume normal operation
    }
}
```

### Safety Zone Visualization

Create visible safety boundaries:

```csharp
void OnDrawGizmos()
{
    if (robot != null)
    {
        Gizmos.color = isInProximity ? Color.red : Color.green;
        Gizmos.DrawWireSphere(robot.position, safetyDistance);
    }
}
```

---

## Hands-On Exercise

### Exercise: Create an Interactive Digital Twin

**Objective**: Build a Unity scene with your humanoid robot that responds to ROS 2 commands and publishes its state.

### Step 1: Project Setup

1. Create a new Unity 2022.3 LTS project
2. Install ROS-TCP-Connector and URDF-Importer packages
3. Import your humanoid URDF

### Step 2: Environment Creation

1. Add a ground plane with material
2. Set up directional lighting
3. Position the robot in the scene

### Step 3: ROS Integration

1. Create and attach `ROSConnection.cs` to an empty GameObject
2. Create and attach `RobotController.cs` to the robot
3. Configure ROS Settings (IP, port)

### Step 4: Test Communication

1. Start ROS-TCP-Endpoint on ROS 2 machine
2. Enter Play mode in Unity
3. Verify connection in Console
4. Test with: `ros2 topic pub /joint_commands sensor_msgs/msg/JointState "..."`

### Expected Outcome

Your Unity digital twin should:
- [ ] Display the humanoid robot correctly
- [ ] Connect to ROS 2 via TCP endpoint
- [ ] Receive and apply joint commands
- [ ] Publish state updates back to ROS 2

---

## Key Takeaways

- **Unity complements Gazebo** for high-fidelity visualization and HRI
- **ROS-TCP-Connector** enables bidirectional Unity-ROS 2 communication
- **URDF Importer** converts robot descriptions to Unity GameObjects
- **ArticulationBody** provides physics-based joint control
- **HRI scenarios** require proximity detection and safety considerations

## What's Next

In [Chapter 3: Sensor Simulation & Validation](./chapter-3-sensors.md), you'll learn to:
- Configure LiDAR, depth cameras, and IMU sensors in Gazebo
- Add realistic noise models to sensor data
- Feed simulated sensor data to ROS 2 perception pipelines

## Additional Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF Importer Guide](https://github.com/Unity-Technologies/URDF-Importer)
- [Unity Manual: ArticulationBody](https://docs.unity3d.com/Manual/class-ArticulationBody.html)
