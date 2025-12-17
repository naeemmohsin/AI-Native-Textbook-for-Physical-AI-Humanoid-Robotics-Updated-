---
sidebar_position: 1
title: "Module 2: The Digital Twin"
description: "Physics-based simulation with Gazebo, high-fidelity digital twins with Unity, and sensor simulation for AI robotics"
---

# Module 2: The Digital Twin (Gazebo & Unity)

Welcome to Module 2! In this module, you'll learn to create digital twins of humanoid robots using two powerful simulation platforms: **Gazebo** for physics-based simulation and **Unity** for high-fidelity rendering and human-robot interaction (HRI).

## What You'll Learn

By the end of this module, you will be able to:

- Understand digital twin concepts and their role in robotics development
- Import URDF humanoid robots into Gazebo and configure physics properties
- Set up Unity with ROS 2 integration for high-fidelity visualization
- Simulate LiDAR, depth cameras, and IMU sensors with realistic noise models
- Feed simulated sensor data to ROS 2 perception pipelines

## Prerequisites

Before starting this module, ensure you have:

- Completed **Module 1: ROS 2 Fundamentals** (nodes, topics, services, URDF)
- Ubuntu 22.04 LTS (native or WSL2) with ROS 2 Humble installed
- A GPU with OpenGL 3.3+ support (4.5 / Vulkan recommended)
- At least 8 GB RAM (16 GB recommended)
- 20 GB free disk space (50 GB recommended)

## Module Structure

This module is organized into three chapters:

### Chapter 1: Physics Simulation with Gazebo

Learn the fundamentals of physics-based simulation:
- Digital twin concepts and value proposition
- Physics engines: rigid body dynamics, gravity, collisions
- Importing URDF humanoids into Gazebo Harmonic
- Creating world files with obstacles and physics properties

### Chapter 2: Digital Twins & HRI in Unity

Build high-fidelity visual environments:
- When to use Unity vs. Gazebo
- Setting up ROS-Unity integration
- Importing robot models with URDF Importer
- Creating human-robot interaction scenarios

### Chapter 3: Sensor Simulation & Validation

Configure realistic sensor simulation:
- LiDAR simulation with point cloud generation
- Depth camera simulation with RGB-D output
- IMU simulation with accelerometer and gyroscope data
- Sensor noise models: Gaussian noise, bias, drift

## Technology Stack

| Component | Version | Purpose |
|-----------|---------|---------|
| Gazebo Sim | Harmonic (8.x) | Physics simulation |
| Unity | 2022.3 LTS | High-fidelity rendering |
| ROS 2 | Humble | Robot middleware |
| ros_gz_bridge | Humble | Gazebo-ROS 2 communication |
| Unity Robotics Hub | Latest | Unity-ROS 2 integration |

## Simulation Platform Comparison

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Primary Use** | Physics simulation | Visual fidelity |
| **Physics Engine** | DART, Bullet, ODE | PhysX |
| **Rendering** | Basic | Photorealistic |
| **ROS Integration** | Native (ros_gz) | Via TCP connector |
| **HRI Support** | Limited | Extensive |
| **Learning Curve** | Moderate | Steeper |

:::tip When to Use Each Platform
- **Gazebo**: Physics testing, sensor validation, algorithm development
- **Unity**: HRI scenarios, visual demos, perception training data generation
:::

## Getting Started

Ready to begin? Start with [Chapter 1: Physics Simulation with Gazebo](./chapter-1-gazebo.md) to learn the fundamentals of digital twin creation.

For environment setup instructions, refer to the [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic/install) and [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub).
