#!/usr/bin/env python3
"""
Isaac Sim Launch Script for Humanoid Robot Scene
Module 3, Chapter 1: Isaac Sim Photorealistic Simulation

This script demonstrates programmatic loading and configuration of
an Isaac Sim scene with a humanoid robot and ROS 2 bridge.

Usage:
    From Isaac Sim's Python environment:
    $ ./python.sh launch_sim.py

    Or from the Script Editor within Isaac Sim.
"""

import argparse
import asyncio
import sys
from pathlib import Path

# ============================================================================
# Configuration
# ============================================================================

# Default paths (adjust for your setup)
DEFAULT_URDF_PATH = "/path/to/your/humanoid.urdf"
DEFAULT_SCENE_PATH = "omniverse://localhost/Projects/humanoid_scene.usd"

# Physics configuration
PHYSICS_DT = 1.0 / 120.0  # 120 Hz physics
RENDERING_DT = 1.0 / 30.0  # 30 Hz rendering

# Sensor configuration
CAMERA_RESOLUTION = (640, 480)
CAMERA_FREQUENCY = 30
IMU_FREQUENCY = 200
LIDAR_FREQUENCY = 20


# ============================================================================
# Isaac Sim Imports (must be after Omniverse initialization)
# ============================================================================

def initialize_isaac_sim():
    """Initialize Isaac Sim and import required modules."""
    from omni.isaac.kit import SimulationApp

    # Create simulation application
    simulation_app = SimulationApp({
        "headless": False,
        "width": 1920,
        "height": 1080,
        "anti_aliasing": 4,
        "renderer": "RayTracedLighting",
    })

    return simulation_app


# ============================================================================
# Scene Setup
# ============================================================================

async def setup_scene(urdf_path: str = None, use_sample_robot: bool = True):
    """Set up the Isaac Sim scene with humanoid robot."""

    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.sensor import Camera, IMUSensor
    import numpy as np

    # Create world
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=PHYSICS_DT,
        rendering_dt=RENDERING_DT,
    )

    # Add ground plane
    world.scene.add_default_ground_plane()

    # Configure physics
    physics_context = world.get_physics_context()
    physics_context.enable_gpu_dynamics(True)
    physics_context.enable_ccd(True)

    # Load robot
    if use_sample_robot or urdf_path is None:
        # Use Isaac Sim's sample humanoid
        from omni.isaac.nucleus import get_assets_root_path
        assets_root = get_assets_root_path()
        robot_usd = f"{assets_root}/Isaac/Robots/Humanoid/humanoid.usd"
        add_reference_to_stage(robot_usd, "/World/Robots/Humanoid")
    else:
        # Import from URDF
        await import_urdf(urdf_path, "/World/Robots/Humanoid")

    # Get the robot articulation
    humanoid = world.scene.add(
        Articulation(
            prim_path="/World/Robots/Humanoid",
            name="humanoid",
        )
    )

    # Add sensors
    await add_sensors(world)

    # Initialize world
    await world.initialize_async()

    # Print robot info
    print(f"Robot DOF: {humanoid.num_dof}")
    print(f"Joint names: {humanoid.dof_names}")

    return world, humanoid


async def import_urdf(urdf_path: str, target_path: str):
    """Import a URDF file as USD."""
    from omni.isaac.urdf import _urdf

    urdf_interface = _urdf.acquire_urdf_interface()

    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False  # Free-floating base for humanoid
    import_config.make_default_prim = True
    import_config.create_physics_scene = False  # We already have one

    # Parse and import
    result = urdf_interface.parse_urdf(urdf_path, import_config)

    if not result:
        raise RuntimeError(f"Failed to parse URDF: {urdf_path}")

    urdf_interface.import_robot(
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=target_path,
    )

    print(f"Imported URDF from {urdf_path} to {target_path}")


async def add_sensors(world):
    """Add sensors to the humanoid robot."""
    from omni.isaac.sensor import Camera, IMUSensor
    from omni.isaac.range_sensor import LidarRtx

    # RGB Camera on head
    camera = Camera(
        prim_path="/World/Robots/Humanoid/head/Camera",
        name="head_camera",
        frequency=CAMERA_FREQUENCY,
        resolution=CAMERA_RESOLUTION,
    )
    world.scene.add(camera)

    # Depth Camera
    depth_camera = Camera(
        prim_path="/World/Robots/Humanoid/head/DepthCamera",
        name="depth_camera",
        frequency=CAMERA_FREQUENCY,
        resolution=CAMERA_RESOLUTION,
    )
    world.scene.add(depth_camera)

    # IMU on torso
    imu = IMUSensor(
        prim_path="/World/Robots/Humanoid/torso/IMU",
        name="torso_imu",
        frequency=IMU_FREQUENCY,
    )
    world.scene.add(imu)

    print("Sensors added: head_camera, depth_camera, torso_imu")


# ============================================================================
# ROS 2 Bridge Setup
# ============================================================================

def setup_ros2_bridge():
    """Configure ROS 2 bridge action graph."""
    import omni.graph.core as og

    keys = og.Controller.Keys

    # Create the action graph
    og.Controller.edit(
        {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),

                # Clock publisher
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),

                # Camera publishers
                ("ReadCamera", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("PublishImage", "omni.isaac.ros2_bridge.ROS2PublishImage"),
                ("PublishCameraInfo", "omni.isaac.ros2_bridge.ROS2PublishCameraInfo"),

                # IMU publisher
                ("ReadIMU", "omni.isaac.sensor.IsaacReadIMU"),
                ("PublishIMU", "omni.isaac.ros2_bridge.ROS2PublishImu"),

                # Joint state publisher
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),

                # Joint command subscriber
                ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),

                # TF publisher
                ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                # Clock
                ("OnPlaybackTick.outputs:tick", "ReadSimTime.inputs:execIn"),
                ("ReadSimTime.outputs:execOut", "PublishClock.inputs:execIn"),

                # Camera
                ("OnPlaybackTick.outputs:tick", "ReadCamera.inputs:execIn"),
                ("ReadCamera.outputs:execOut", "PublishImage.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishCameraInfo.inputs:execIn"),

                # IMU
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),

                # Joint states
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),

                # TF
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),

                # Joint commands
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ],
            keys.SET_VALUES: [
                ("ReadCamera.inputs:cameraPrim", "/World/Robots/Humanoid/head/Camera"),
                ("PublishImage.inputs:topicName", "/camera/image_raw"),
                ("PublishCameraInfo.inputs:topicName", "/camera/camera_info"),
                ("ReadIMU.inputs:imuPrim", "/World/Robots/Humanoid/torso/IMU"),
                ("PublishIMU.inputs:topicName", "/imu/data"),
                ("PublishJointState.inputs:targetPrim", "/World/Robots/Humanoid"),
                ("PublishJointState.inputs:topicName", "/joint_states"),
                ("SubscribeJointState.inputs:topicName", "/joint_commands"),
                ("ArticulationController.inputs:robotPath", "/World/Robots/Humanoid"),
                ("PublishTF.inputs:targetPrims", ["/World/Robots/Humanoid"]),
            ],
        }
    )

    print("ROS 2 bridge action graph created")


# ============================================================================
# Main Entry Point
# ============================================================================

async def main():
    """Main async entry point."""

    print("=" * 60)
    print("Isaac Sim Humanoid Robot Scene Launcher")
    print("Module 3, Chapter 1: NVIDIA Isaac Sim")
    print("=" * 60)

    # Setup scene
    world, humanoid = await setup_scene(use_sample_robot=True)

    # Setup ROS 2 bridge
    setup_ros2_bridge()

    print("\nScene setup complete!")
    print("Starting simulation...")
    print("\nROS 2 Topics:")
    print("  Publishers:")
    print("    /clock")
    print("    /camera/image_raw")
    print("    /camera/camera_info")
    print("    /imu/data")
    print("    /joint_states")
    print("    /tf")
    print("  Subscribers:")
    print("    /joint_commands")
    print("\nPress PLAY to start simulation.")

    # Run simulation loop
    while True:
        world.step(render=True)
        await asyncio.sleep(0)


if __name__ == "__main__":
    # Initialize simulation app
    simulation_app = initialize_isaac_sim()

    # Run main async function
    asyncio.get_event_loop().run_until_complete(main())

    # Cleanup
    simulation_app.close()
