---
sidebar_position: 4
title: "Chapter 3: Sensor Simulation & Validation"
description: "Configure LiDAR, depth cameras, and IMU sensors with realistic noise models in Gazebo"
---

# Chapter 3: Sensor Simulation & Validation

## Learning Objectives

By the end of this chapter, you will be able to:

- Configure LiDAR sensors in Gazebo with realistic parameters
- Set up depth cameras with RGB-D output
- Simulate IMU sensors with accelerometer and gyroscope data
- Apply Gaussian noise, bias, and drift to sensor data
- Feed simulated sensor data to ROS 2 perception nodes

## Prerequisites

Before starting this chapter, ensure you have:

- Completed Chapter 1 (Gazebo physics simulation)
- Gazebo Harmonic with ros_gz packages installed
- ROS 2 Humble workspace configured
- A robot model spawned in Gazebo

---

## Sensor Simulation Fundamentals

Sensors are the robot's interface to the physical world. In simulation, sensors must produce data that closely resembles real-world sensor output to enable effective algorithm development.

### Why Simulate Sensors?

Simulated sensors enable:

| Benefit | Description |
|---------|-------------|
| **Algorithm development** | Test perception code without hardware |
| **Edge case testing** | Create scenarios too dangerous for real testing |
| **Training data generation** | Produce labeled datasets for ML |
| **Reproducibility** | Repeat exact conditions for debugging |

### The Sim-to-Real Gap

Real sensors differ from ideal simulated sensors in several ways:

- **Noise**: Random variations in measurements
- **Bias**: Systematic offsets from true values
- **Drift**: Gradual change in bias over time
- **Dropouts**: Occasional missing readings
- **Environmental effects**: Temperature, humidity, vibration

Configuring realistic noise models helps close this gap.

### Gazebo Sensor Plugins

Gazebo provides built-in sensor plugins via the `gz-sensors` library:

| Sensor Type | Plugin | ROS Message |
|-------------|--------|-------------|
| 2D LiDAR | `gz-sensors-lidar` | `sensor_msgs/LaserScan` |
| 3D LiDAR | `gz-sensors-gpu_lidar` | `sensor_msgs/PointCloud2` |
| RGB Camera | `gz-sensors-camera` | `sensor_msgs/Image` |
| Depth Camera | `gz-sensors-rgbd_camera` | `sensor_msgs/Image` |
| IMU | `gz-sensors-imu` | `sensor_msgs/Imu` |

---

## LiDAR Simulation

LiDAR (Light Detection and Ranging) sensors measure distances by emitting laser pulses and measuring return times. They're essential for mapping, localization, and obstacle detection.

### LiDAR Configuration

Add a LiDAR sensor to your robot's SDF:

```xml title="examples/module-2/chapter-3/sensor_configs/lidar_config.sdf"
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.1 0 0 0</pose>
  <update_rate>10</update_rate>
  <always_on>true</always_on>
  <visualize>true</visualize>

  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>

  <topic>/scan</topic>
</sensor>
```

### Key Parameters

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `samples` | Rays per scan | 360-1080 |
| `resolution` | Angular resolution (deg) | 0.25-1.0 |
| `min/max_angle` | Field of view (rad) | -π to π for 360° |
| `range.min` | Minimum detection | 0.05-0.3 m |
| `range.max` | Maximum detection | 10-100 m |
| `noise.stddev` | Range noise | 0.01-0.03 m |

### Bridging to ROS 2

Connect the Gazebo LiDAR topic to ROS 2:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

### Visualizing in RViz2

```bash
# Launch RViz2 with default config
ros2 run rviz2 rviz2

# Add a LaserScan display
# Set Topic to /scan
# Set Fixed Frame to the LiDAR link frame
```

---

## Depth Camera Simulation

Depth cameras provide both color images and depth information, enabling 3D perception. They're commonly used for object detection, manipulation, and navigation.

### Depth Camera Configuration

```xml title="examples/module-2/chapter-3/sensor_configs/depth_camera_config.sdf"
<sensor name="rgbd_camera" type="rgbd_camera">
  <pose>0 0 0 0 0 0</pose>
  <update_rate>30</update_rate>
  <always_on>true</always_on>

  <camera name="depth_camera">
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <depth_camera>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </depth_camera>
    <horizontal_fov>1.047</horizontal_fov>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>

  <topic>/camera</topic>
</sensor>
```

### Camera Parameters

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `width/height` | Image resolution | 640x480, 1280x720 |
| `format` | Color format | R8G8B8, R8G8B8A8 |
| `horizontal_fov` | Field of view (rad) | 1.047 (60°) |
| `depth_camera.clip` | Depth range | 0.1-10 m |
| `noise.stddev` | Image noise | 0.005-0.01 |

### Output Topics

The RGBD camera produces multiple topics:

- `/camera/image`: RGB color image
- `/camera/depth`: Depth image (float32)
- `/camera/points`: Point cloud
- `/camera/camera_info`: Calibration data

### Bridging to ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera/depth@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

---

## IMU Simulation

Inertial Measurement Units (IMUs) provide orientation, angular velocity, and linear acceleration data. They're essential for state estimation and balance control in humanoid robots.

### IMU Configuration

```xml title="examples/module-2/chapter-3/sensor_configs/imu_config.sdf"
<sensor name="imu" type="imu">
  <pose>0 0 0 0 0 0</pose>
  <update_rate>100</update_rate>
  <always_on>true</always_on>
  <visualize>true</visualize>

  <imu>
    <orientation_reference_frame>
      <localization>ENU</localization>
    </orientation_reference_frame>

    <!-- Gyroscope noise -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
          <dynamic_bias_stddev>0.00000035</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- y and z similar -->
    </angular_velocity>

    <!-- Accelerometer noise -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
          <dynamic_bias_stddev>0.00015</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- y and z similar -->
    </linear_acceleration>

    <enable_orientation>true</enable_orientation>
  </imu>

  <topic>/imu</topic>
</sensor>
```

### IMU Components

| Component | Measures | Units |
|-----------|----------|-------|
| **Accelerometer** | Linear acceleration | m/s² |
| **Gyroscope** | Angular velocity | rad/s |
| **Magnetometer** | Magnetic field (optional) | Tesla |
| **Orientation** | Quaternion (derived) | unitless |

### Bridging to ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

---

## Sensor Noise Models

Realistic noise modeling is crucial for developing robust perception algorithms. Gazebo supports several noise types.

### Gaussian Noise

The most common noise model adds random variations:

```xml
<noise type="gaussian">
  <mean>0.0</mean>       <!-- Average offset -->
  <stddev>0.01</stddev>  <!-- Standard deviation -->
</noise>
```

**When to use**: General sensor noise, range measurements, images

### Bias

Static offset that persists across measurements:

```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>
  <bias_mean>0.1</bias_mean>     <!-- Average bias -->
  <bias_stddev>0.001</bias_stddev> <!-- Bias variation -->
</noise>
```

**When to use**: IMU accelerometers, gyroscopes, sensors with calibration errors

### Drift

Time-varying bias that changes slowly:

```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>
  <dynamic_bias_stddev>0.0001</dynamic_bias_stddev>
  <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
</noise>
```

**Parameters**:
- `dynamic_bias_stddev`: Rate of drift
- `dynamic_bias_correlation_time`: Time constant for drift changes (seconds)

**When to use**: Gyroscope integration, long-duration IMU operation

### Noise Parameter Selection

Match noise to real sensor datasheets:

| Sensor | Typical Noise (stddev) |
|--------|------------------------|
| LiDAR range | 0.01-0.03 m |
| RGB camera pixel | 0.005-0.01 |
| Gyroscope | 0.0002-0.001 rad/s |
| Accelerometer | 0.01-0.05 m/s² |

---

## ROS 2 Sensor Integration

Connecting simulated sensors to ROS 2 perception pipelines enables testing algorithms without hardware.

### ros_gz_bridge Configuration

For multiple sensors, use a launch file:

```python title="examples/module-2/chapter-3/sensor_processing/sensor_bridge.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
    )

    return LaunchDescription([bridge])
```

### Processing Sensor Data

Create a ROS 2 node to process incoming sensor data:

```python title="examples/module-2/chapter-3/sensor_processing/sensor_subscriber.py"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import numpy as np

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid = ranges[np.isfinite(ranges)]
        if len(valid) > 0:
            self.get_logger().info(
                f'LiDAR: min={np.min(valid):.2f}m, '
                f'mean={np.mean(valid):.2f}m'
            )

    def imu_callback(self, msg):
        acc = msg.linear_acceleration
        self.get_logger().info(
            f'IMU acc: ({acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f})'
        )

def main():
    rclpy.init()
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Verifying Sensor Data

Check that sensors are publishing correctly:

```bash
# List available topics
ros2 topic list

# Check publishing rate
ros2 topic hz /scan
ros2 topic hz /imu

# Echo sensor data
ros2 topic echo /scan --no-arr  # Skip array fields
ros2 topic echo /imu
```

---

## Hands-On Exercise

### Exercise: Add Sensors to Your Humanoid

**Objective**: Configure LiDAR, depth camera, and IMU sensors on your humanoid robot and verify data flows to ROS 2.

### Step 1: Modify Robot SDF

Add sensor definitions to your robot's head or torso link:

1. Add IMU sensor to the torso (base_link)
2. Add depth camera to the head
3. Add LiDAR if appropriate for your robot

### Step 2: Configure Noise Models

For each sensor, add noise parameters:

1. LiDAR: `stddev=0.02` (2cm range noise)
2. Depth camera: `stddev=0.007`
3. IMU gyroscope: `stddev=0.0002`, `bias_mean=0.0000075`
4. IMU accelerometer: `stddev=0.017`, `bias_mean=0.1`

### Step 3: Create Bridge Launch File

Create a launch file that bridges all sensor topics to ROS 2.

### Step 4: Process Sensor Data

Use the provided `sensor_subscriber.py` or create your own node to:
- Subscribe to all sensor topics
- Print basic statistics (min, max, mean)
- Count message rates

### Step 5: Visualize in RViz2

Set up RViz2 displays for:
- [ ] LaserScan
- [ ] PointCloud2 (from depth camera)
- [ ] TF (robot model with sensor frames)

### Expected Outcome

Your simulation should:
- [ ] Publish LiDAR scans at 10 Hz with noise
- [ ] Publish depth images at 30 Hz
- [ ] Publish IMU data at 100 Hz with realistic noise
- [ ] All data visible in RViz2

---

## Key Takeaways

- **Sensor simulation** enables perception algorithm development without hardware
- **LiDAR sensors** provide range data via laser scanning
- **Depth cameras** output RGB images, depth maps, and point clouds
- **IMUs** measure orientation, angular velocity, and acceleration
- **Noise models** (Gaussian, bias, drift) make simulated data realistic
- **ros_gz_bridge** connects Gazebo sensors to ROS 2 topics

## Module Summary

Congratulations! You've completed Module 2: The Digital Twin. You now have the skills to:

1. **Create physics simulations** in Gazebo with realistic dynamics
2. **Build high-fidelity digital twins** in Unity for HRI testing
3. **Simulate sensors** with configurable noise for perception development

## What's Next

In **Module 3**, you'll learn to integrate AI capabilities with your robotic systems, covering:
- Machine learning for robot perception
- Natural language interfaces for robot control
- Reinforcement learning for motion planning

## Additional Resources

- [gz-sensors API Documentation](https://gazebosim.org/api/sensors/8/index.html)
- [Sensor Noise Models](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1Noise.html)
- [ros_gz_bridge Tutorial](https://gazebosim.org/docs/harmonic/ros2_integration)
- [IMU Noise Characterization](https://www.vectornav.com/resources/inertial-navigation-articles/what-is-an-imu)
