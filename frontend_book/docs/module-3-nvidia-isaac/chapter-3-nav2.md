---
sidebar_position: 3
title: "Chapter 3: Navigation with Nav2"
description: "Integrate Nav2 for autonomous navigation with localization, mapping, and path planning for humanoid robots"
---

# Chapter 3: Navigation with Nav2

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the Nav2 architecture and behavior tree navigation
2. Configure costmaps for humanoid robot navigation
3. Build maps using SLAM Toolbox
4. Set up AMCL localization for indoor environments
5. Implement autonomous goal-seeking behavior

## Prerequisites

Before starting this chapter, ensure you have:

- Completed Chapter 1 (Isaac Sim) and Chapter 2 (Isaac ROS)
- ROS 2 Humble with Nav2 packages installed
- A simulation environment with LiDAR or depth camera
- Basic understanding of occupancy grids and path planning

:::info Nav2 Overview
Nav2 is the navigation stack for ROS 2, providing modular components for mapping, localization, and autonomous navigation. It replaces the ROS 1 Navigation Stack with improved architecture and capabilities.
:::

## Nav2 Architecture

Nav2 uses a behavior tree-based architecture for flexible navigation control.

### Core Components

Nav2 consists of several interconnected servers:

```text
┌─────────────────────────────────────────────────────────────┐
│                     Behavior Tree Navigator                  │
│  (Coordinates all navigation behaviors via behavior trees)  │
└────────────────────────┬────────────────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         ▼               ▼               ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│   Planner   │  │ Controller  │  │  Recovery   │
│   Server    │  │   Server    │  │   Server    │
│ (Global     │  │ (Local      │  │ (Backup     │
│  Planning)  │  │  Control)   │  │  Behaviors) │
└─────────────┘  └─────────────┘  └─────────────┘
         │               │               │
         └───────────────┼───────────────┘
                         ▼
              ┌─────────────────────┐
              │    Costmap Server   │
              │ (Occupancy Grids)   │
              └─────────────────────┘
```

### Behavior Tree Navigation

Nav2 uses behavior trees (BT) for mission execution:

- **Sequence**: Execute actions in order
- **Fallback**: Try alternatives if actions fail
- **Decorator**: Modify child behavior (retry, timeout)
- **Action**: Execute navigation primitives

Default navigation behavior tree:
```text
NavigateRecovery (RecoveryNode)
├── NavigateWithReplanning (PipelineSequence)
│   ├── RateController
│   │   └── ComputePathToPose
│   └── FollowPath
└── RecoveryActions (Sequence)
    ├── ClearCostmaps
    ├── Spin
    ├── Wait
    └── BackUp
```

### Plugin Architecture

Nav2 supports plugins for different algorithms:

| Component | Plugin Type | Common Options |
|-----------|-------------|----------------|
| Global Planner | nav2_core::GlobalPlanner | NavFn, Smac 2D, Smac Hybrid-A* |
| Local Controller | nav2_core::Controller | DWB, MPPI, RPP |
| Recoveries | nav2_core::Recovery | Spin, BackUp, Wait |
| Costmap Layers | nav2_costmap_2d::Layer | Static, Inflation, Obstacle |

## Costmap Configuration

Costmaps represent the robot's environment for navigation planning.

### Global vs Local Costmaps

- **Global Costmap**: Full environment map for path planning
- **Local Costmap**: Rolling window around robot for obstacle avoidance

```yaml
# Costmap configuration structure
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.3  # Humanoid footprint radius
      resolution: 0.05   # 5cm grid cells

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
```

### Costmap Layers

Configure layers for sensor fusion:

```yaml
# nav2_params.yaml (costmap layers section)
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # Robot radius + safety margin
```

### Humanoid Robot Considerations

Humanoid robots have unique costmap requirements:

```yaml
# Humanoid-specific parameters
robot_radius: 0.3  # Approximate circular footprint
footprint: "[[0.25, 0.15], [0.25, -0.15], [-0.25, -0.15], [-0.25, 0.15]]"

# Consider robot height for obstacle detection
obstacle_layer:
  max_obstacle_height: 2.0  # Full body height
  min_obstacle_height: 0.1  # Ignore ground plane

# Larger inflation for stability
inflation_layer:
  inflation_radius: 0.6
  cost_scaling_factor: 2.5
```

:::warning Footprint Selection
For bipedal humanoids, use `robot_radius` for planning (simpler) or a rectangular `footprint` for tight spaces. The footprint affects path width requirements.
:::

## Mapping with SLAM Toolbox

SLAM Toolbox provides 2D LiDAR-based mapping for indoor environments.

### SLAM Toolbox Features

- **Online Mapping**: Build maps while navigating
- **Lifelong Mapping**: Update existing maps over time
- **Localization Mode**: Localize against a saved map
- **Loop Closure**: Correct accumulated drift

### Configuration

Create a SLAM Toolbox configuration:

```yaml
# slam_toolbox_config.yaml
slam_toolbox:
  ros__parameters:
    # Plugin selection
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG

    # Mapping parameters
    resolution: 0.05  # 5cm resolution
    map_update_interval: 5.0
    max_laser_range: 12.0

    # Scan matching
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0

    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Performance
    use_scan_matching: true
    use_scan_barycenter: true
    transform_publish_period: 0.02

    # Frame IDs
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
```

### Mapping Workflow

1. **Start SLAM Toolbox in mapping mode**:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=slam_toolbox_config.yaml
```

2. **Drive the robot** through the environment using teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. **Monitor map quality** in RViz2:
   - Add Map display subscribing to `/map`
   - Check for loop closures and drift

4. **Save the map** when complete:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

This creates:
- `my_map.yaml`: Map metadata
- `my_map.pgm`: Occupancy grid image

### Map Editing

Edit maps in image software (GIMP, Photoshop):

- **Black (0)**: Occupied (obstacles)
- **White (254)**: Free space
- **Gray (205)**: Unknown

:::tip Map Quality
For best navigation results, ensure clear corridors between rooms, remove mapping artifacts, and verify doorways are properly captured.
:::

## Localization with AMCL

AMCL (Adaptive Monte Carlo Localization) provides pose estimation within a known map.

### AMCL Algorithm

AMCL uses a particle filter for localization:

```text
1. Initialize particle cloud around initial pose
2. Motion update: Move particles based on odometry
3. Sensor update: Weight particles by sensor match
4. Resampling: Concentrate particles on likely poses
5. Repeat from step 2
```

### AMCL Configuration

```yaml
# nav2_params.yaml (AMCL section)
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2   # Rotation noise from rotation
    alpha2: 0.2   # Rotation noise from translation
    alpha3: 0.2   # Translation noise from translation
    alpha4: 0.2   # Translation noise from rotation
    alpha5: 0.2   # Translation noise (omnidirectional)

    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3

    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1

    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.1
    laser_model_type: "likelihood_field"

    max_beams: 60
    max_particles: 2000
    min_particles: 500

    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99

    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1

    robot_model_type: "nav2_amcl::DifferentialMotionModel"

    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true

    transform_tolerance: 1.0
    update_min_a: 0.2  # Rotation threshold (radians)
    update_min_d: 0.25  # Translation threshold (meters)
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

### Initial Pose Estimation

Set the initial pose in RViz2:
1. Click **2D Pose Estimate** button
2. Click and drag on the map to set position and orientation
3. AMCL will converge to the correct pose

Programmatically:
```bash
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}, \
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

### Monitoring Localization Quality

Check particle convergence:

```bash
# View particle cloud
ros2 topic echo /particlecloud

# Check pose estimate
ros2 topic echo /amcl_pose
```

Good localization indicators:
- Particle cloud is tight (low variance)
- Pose updates are stable
- Map-to-odom TF is consistent

## Path Planning

Nav2 provides multiple planners for different navigation scenarios.

### Global Planners

**NavFn** (Grid-based A*/Dijkstra):
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

**Smac Planner** (Lattice-based, smoother paths):
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["SmacPlanner"]
    SmacPlanner:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.25
      minimum_turning_radius: 0.5  # Humanoid turning radius
      motion_model_for_search: "REEDS_SHEPP"
```

### Local Controllers

**DWB Controller** (Dynamic Window):
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5    # Slow for humanoid stability
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 1.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.0
      decel_lim_theta: -2.0
```

**Regulated Pure Pursuit** (Smoother, velocity regulation):
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.4
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.0
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: true
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
```

### Humanoid Planner Selection

For humanoid robots, consider:

| Scenario | Recommended Planner | Reasoning |
|----------|---------------------|-----------|
| Open areas | SmacPlannerHybrid | Smooth paths, natural motion |
| Tight spaces | NavFn + DWB | Reactive obstacle avoidance |
| Slow/stable | RPP Controller | Velocity regulation, smooth curves |

### Goal Tolerance

Configure goal tolerance for humanoid precision:

```yaml
controller_server:
  ros__parameters:
    goal_checker_plugins: ["general_goal_checker"]
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25    # 25cm position tolerance
      yaw_goal_tolerance: 0.25   # ~14 degrees orientation
      stateful: true
```

## Complete Navigation Workflow

Launch the full Nav2 stack for autonomous navigation.

### Launch Configuration

Create a combined launch file:

```python
# humanoid_nav.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    nav2_bringup = FindPackageShare('nav2_bringup')

    return LaunchDescription([
        # Nav2 bringup with custom parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_bringup, 'launch', 'bringup_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': '/path/to/nav2_params.yaml',
                'map': '/path/to/my_map.yaml',
            }.items()
        ),
    ])
```

### Sending Goals via RViz2

1. Launch RViz2 with Nav2 configuration:
```bash
ros2 launch nav2_bringup rviz_launch.py
```

2. Click **2D Pose Estimate** to set initial position

3. Click **Nav2 Goal** to send navigation goals

4. Monitor progress in the behavior tree panel

### Programmatic Goal Sending

Send goals from Python:

```python
#!/usr/bin/env python3
"""Send navigation goals programmatically."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def send_goal(self, x, y, yaw=0.0):
        """Send a navigation goal."""
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        import math
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete!')


def main():
    rclpy.init()
    navigator = NavigationClient()
    navigator.send_goal(x=2.0, y=1.5, yaw=1.57)  # Go to (2, 1.5) facing +Y
    rclpy.spin(navigator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Navigation Feedback

Monitor navigation status:

```bash
# Goal status
ros2 topic echo /navigate_to_pose/_action/status

# Current velocity commands
ros2 topic echo /cmd_vel

# Costmap updates
ros2 topic echo /global_costmap/costmap
```

## Hands-On Exercise

Build a map and navigate autonomously through it.

### Step 1: Launch Simulation

Start Isaac Sim with your humanoid robot and LiDAR sensor (from Chapter 1).

### Step 2: Build a Map

1. Launch SLAM Toolbox:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

2. Launch teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. Drive around the environment to build the map

4. Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/humanoid_map
```

### Step 3: Launch Navigation

1. Stop SLAM Toolbox

2. Launch Nav2 with localization:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=~/maps/humanoid_map.yaml \
  params_file:=/path/to/nav2_params.yaml
```

3. Set initial pose in RViz2

### Step 4: Send Navigation Goals

1. Use RViz2 **Nav2 Goal** button to send goals

2. Or run the programmatic navigation script:

```bash
ros2 run my_package navigation_goal.py --ros-args -p x:=2.0 -p y:=1.5
```

### Step 5: Test Recovery Behaviors

1. Place an obstacle in the robot's path during navigation

2. Observe the robot:
   - Attempting to replan around the obstacle
   - Executing recovery behaviors if stuck

### Success Criteria

You have successfully completed this exercise when:

- [ ] Map is generated with clear walls and corridors
- [ ] AMCL localizes the robot correctly in the map
- [ ] Robot navigates to goal positions autonomously
- [ ] Robot avoids dynamic obstacles during navigation
- [ ] Recovery behaviors execute when robot gets stuck

## Key Takeaways

1. **Nav2 behavior trees** provide flexible, configurable navigation workflows with recovery capabilities

2. **Costmap layers** combine multiple sensor sources for comprehensive obstacle representation

3. **SLAM Toolbox** enables efficient 2D mapping with loop closure for drift correction

4. **AMCL localization** provides robust pose estimation using particle filtering

5. **Planner selection** matters - choose based on environment complexity and robot dynamics

## Next Steps

You have completed Module 3: The AI-Robot Brain! You now know how to:

- Create photorealistic simulations with Isaac Sim
- Build GPU-accelerated perception pipelines with Isaac ROS
- Implement autonomous navigation with Nav2

Consider exploring:
- Outdoor navigation with GPS waypoints
- Multi-floor navigation with elevator handling
- Human-aware navigation and social costmaps
- Reinforcement learning for navigation policy optimization

[← Back to Module 3 Overview](/docs/module-3-nvidia-isaac)
