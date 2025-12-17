# Quickstart: Module 1 Development Setup

**Feature**: 001-ros2-module
**Date**: 2025-12-16
**Purpose**: Step-by-step setup for developing and testing Module 1 content

## Prerequisites

- Node.js 18+ installed
- Git installed
- Text editor (VS Code recommended)
- (For testing examples) Ubuntu 22.04 or WSL2 with ROS 2 Humble

## Step 1: Clone Repository

```bash
git clone https://github.com/<org>/textbook.git
cd textbook
```

## Step 2: Initialize Docusaurus (First Time Only)

If Docusaurus is not yet initialized:

```bash
npx create-docusaurus@latest . classic --typescript
```

If already initialized, install dependencies:

```bash
npm install
```

## Step 3: Start Development Server

```bash
npm run start
```

Site will be available at `http://localhost:3000`

## Step 4: Create Module 1 Directory Structure

```bash
# Create docs directory for Module 1
mkdir -p docs/module-1-ros2

# Create examples directory
mkdir -p examples/module-1/chapter-1
mkdir -p examples/module-1/chapter-2
mkdir -p examples/module-1/chapter-3

# Create image directory
mkdir -p static/img/module-1
```

## Step 5: Create Category Configuration

Create `docs/module-1-ros2/_category_.json`:

```json
{
  "label": "Module 1: The Robotic Nervous System",
  "position": 1,
  "collapsible": true,
  "collapsed": false,
  "link": {
    "type": "doc",
    "id": "module-1-ros2/index"
  }
}
```

## Step 6: Create Module Index

Create `docs/module-1-ros2/index.md`:

```markdown
---
sidebar_position: 0
title: "Module 1: The Robotic Nervous System"
description: "Introduction to ROS 2 for Physical AI and humanoid robotics"
---

# Module 1: The Robotic Nervous System (ROS 2)

Welcome to Module 1! In this module, you'll learn...

## What You'll Learn

- ROS 2 fundamentals and communication patterns
- Python programming with rclpy
- Robot description with URDF

## Chapters

1. [ROS 2 Fundamentals](./chapter-1-fundamentals.md)
2. [Python Agents with rclpy](./chapter-2-rclpy.md)
3. [Humanoid Description with URDF](./chapter-3-urdf.md)

## Prerequisites

- Python programming experience (intermediate)
- Basic command-line proficiency
- Linux/WSL2 environment
```

## Step 7: Build and Test

```bash
# Build the site
npm run build

# Serve the built site locally
npm run serve
```

## Verification Checklist

- [ ] Development server starts without errors
- [ ] Module 1 appears in sidebar navigation
- [ ] Module index page loads correctly
- [ ] Internal links between chapters work
- [ ] Code blocks render with syntax highlighting
- [ ] Images load from static/img/module-1/

## Testing Code Examples (ROS 2 Environment)

To test that code examples work:

### 1. Set Up ROS 2 Humble (Ubuntu 22.04 / WSL2)

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### 2. Test Python Examples

```bash
cd examples/module-1/chapter-2

# Run publisher (Terminal 1)
python3 publisher_node.py

# Run subscriber (Terminal 2)
python3 subscriber_node.py
```

### 3. Test URDF Examples

```bash
cd examples/module-1/chapter-3

# Validate URDF
check_urdf humanoid_arm.urdf

# View in RViz2 (requires launch file)
ros2 launch display.launch.py
```

## Troubleshooting

### Docusaurus Build Fails

```bash
# Clear cache and rebuild
rm -rf .docusaurus build node_modules/.cache
npm run build
```

### Sidebar Not Showing Module

- Verify `_category_.json` is valid JSON
- Check `sidebars.ts` includes docs directory
- Restart development server

### Code Examples Don't Run

- Ensure ROS 2 Humble is sourced: `source /opt/ros/humble/setup.bash`
- Check Python version: `python3 --version` (should be 3.10+)
- Verify rclpy is installed: `python3 -c "import rclpy"`

## Next Steps

After setup is verified:

1. Run `/sp.tasks` to generate implementation tasks
2. Begin writing Chapter 1 content
3. Create code examples in parallel with content
4. Test examples before finalizing chapters
