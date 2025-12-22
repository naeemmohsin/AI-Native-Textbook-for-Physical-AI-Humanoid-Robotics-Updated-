#!/usr/bin/env python3
"""
Synthetic Data Generation Configuration for NVIDIA Isaac Sim
Module 3, Chapter 1: Isaac Sim Photorealistic Simulation

This script configures Omniverse Replicator for generating synthetic
training data with domain randomization for humanoid robot perception.
"""

import omni.replicator.core as rep
import numpy as np
from pathlib import Path

# ============================================================================
# Configuration
# ============================================================================

OUTPUT_DIR = Path("/data/synthetic_dataset")
NUM_FRAMES = 1000
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

# Camera configuration
CAMERA_POSITIONS = [
    (2.0, 2.0, 1.5),
    (2.5, 0.0, 1.8),
    (0.0, 2.5, 1.5),
    (-2.0, 2.0, 2.0),
]

# Lighting ranges for randomization
LIGHT_INTENSITY_RANGE = (500, 2000)
LIGHT_COLOR_RANGE = ((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))

# Robot pose randomization ranges (degrees)
ROBOT_ROTATION_RANGE = ((-10, -180, -10), (10, 180, 10))

# ============================================================================
# Randomizers
# ============================================================================

def register_randomizers():
    """Register all domain randomization functions."""

    @rep.randomizer.register
    def randomize_lighting():
        """Randomize dome light intensity and color."""
        lights = rep.get.prims(path_pattern="/World/Environment/*Light*")
        with lights:
            rep.modify.attribute(
                "inputs:intensity",
                rep.distribution.uniform(*LIGHT_INTENSITY_RANGE)
            )
            rep.modify.attribute(
                "inputs:color",
                rep.distribution.uniform(*LIGHT_COLOR_RANGE)
            )

    @rep.randomizer.register
    def randomize_robot_pose():
        """Randomize humanoid robot pose within safe ranges."""
        robot = rep.get.prims(path_pattern="/World/Robots/Humanoid")
        with robot:
            rep.modify.pose(
                rotation=rep.distribution.uniform(*ROBOT_ROTATION_RANGE)
            )

    @rep.randomizer.register
    def randomize_textures():
        """Randomize material colors for domain randomization."""
        materials = rep.get.prims(path_pattern="/World/Looks/*")
        with materials:
            rep.modify.attribute(
                "inputs:diffuseColor",
                rep.distribution.uniform(
                    (0.1, 0.1, 0.1),
                    (0.9, 0.9, 0.9)
                )
            )

    @rep.randomizer.register
    def randomize_camera_position():
        """Cycle through predefined camera positions."""
        camera = rep.get.prims(path_pattern="/World/Cameras/DataCapture*")
        with camera:
            position = rep.distribution.choice(CAMERA_POSITIONS)
            rep.modify.pose(position=position, look_at=(0, 0, 0.5))


# ============================================================================
# Semantic Labels
# ============================================================================

def setup_semantic_labels():
    """Configure semantic segmentation labels for scene objects."""

    # Robot parts
    rep.modify.semantics([
        ("class", "robot"),
        ("class", "robot_arm"),
        ("class", "robot_leg"),
        ("class", "robot_torso"),
        ("class", "robot_head"),
    ])

    # Environment
    rep.modify.semantics([
        ("class", "floor"),
        ("class", "wall"),
        ("class", "obstacle"),
        ("class", "background"),
    ])


# ============================================================================
# Data Writers
# ============================================================================

def setup_writers(render_products: list):
    """Configure output writers for different annotation types."""

    # Basic writer for RGB, depth, and segmentation
    basic_writer = rep.WriterRegistry.get("BasicWriter")
    basic_writer.initialize(
        output_dir=str(OUTPUT_DIR / "basic"),
        rgb=True,
        semantic_segmentation=True,
        instance_segmentation=True,
        distance_to_camera=True,
        bounding_box_2d_tight=True,
        bounding_box_2d_loose=True,
        bounding_box_3d=True,
        camera_params=True,
    )
    basic_writer.attach(render_products)

    # KITTI format for object detection
    kitti_writer = rep.WriterRegistry.get("KittiWriter")
    kitti_writer.initialize(
        output_dir=str(OUTPUT_DIR / "kitti"),
        semantic_types=["class"],
        colorize_semantic_segmentation=True,
    )
    kitti_writer.attach(render_products)

    # COCO format for segmentation tasks
    coco_writer = rep.WriterRegistry.get("CocoWriter")
    coco_writer.initialize(
        output_dir=str(OUTPUT_DIR / "coco"),
        semantic_types=["class"],
    )
    coco_writer.attach(render_products)

    return [basic_writer, kitti_writer, coco_writer]


# ============================================================================
# Main Pipeline
# ============================================================================

def create_capture_camera():
    """Create a camera for data capture with proper settings."""
    camera = rep.create.camera(
        name="DataCaptureCamera",
        position=CAMERA_POSITIONS[0],
        look_at=(0, 0, 0.5),
        focal_length=24.0,
    )
    return camera


def create_render_product(camera):
    """Create render product for data capture."""
    render_product = rep.create.render_product(
        camera,
        resolution=(IMAGE_WIDTH, IMAGE_HEIGHT),
    )
    return render_product


def run_data_generation():
    """Execute the synthetic data generation pipeline."""

    print(f"Starting synthetic data generation...")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Number of frames: {NUM_FRAMES}")
    print(f"Resolution: {IMAGE_WIDTH}x{IMAGE_HEIGHT}")

    # Create output directory
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Register randomizers
    register_randomizers()

    # Setup semantic labels
    setup_semantic_labels()

    # Create camera and render product
    camera = create_capture_camera()
    render_product = create_render_product(camera)

    # Setup writers
    writers = setup_writers([render_product])

    # Define the generation loop with randomization
    with rep.trigger.on_frame(num_frames=NUM_FRAMES):
        rep.randomizer.randomize_lighting()
        rep.randomizer.randomize_robot_pose()
        rep.randomizer.randomize_textures()
        rep.randomizer.randomize_camera_position()

    print(f"Data generation configured. Starting capture...")

    # Run the replicator
    rep.orchestrator.run()

    print(f"Synthetic data generation complete!")
    print(f"Data saved to: {OUTPUT_DIR}")


# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    # This script should be run from Isaac Sim's Script Editor
    # or via the Isaac Sim Python environment

    # Check if running in Isaac Sim
    try:
        import omni.isaac.core
        run_data_generation()
    except ImportError:
        print("Error: This script must be run within NVIDIA Isaac Sim.")
        print("Launch Isaac Sim and run this script from the Script Editor.")
        print("Alternatively, use: ./python.sh replicator_config.py")
