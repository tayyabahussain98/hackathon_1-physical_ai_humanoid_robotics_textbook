---
title: Introduction to ROS 2
sidebar_position: 1
description: Learn the fundamentals of Robot Operating System 2 (ROS 2) for humanoid robotics applications
---

# Introduction to ROS 2

## Overview

Robot Operating System 2 (ROS 2) is not an actual operating system, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms, from humanoid robots to industrial manipulators.

In the context of humanoid robotics, ROS 2 serves as the "nervous system" of the robot, enabling different hardware and software components to communicate and coordinate with each other seamlessly.

## Key Concepts of ROS 2

### Nodes
Nodes are the fundamental building blocks of ROS 2. A node is a process that performs computation. In a humanoid robot, you might have nodes for:
- Joint controllers
- Sensor processing
- Motion planning
- Perception systems
- High-level decision making

Nodes are designed to be modular and reusable, allowing you to build complex robot behaviors from simple, focused components.

### Packages
Packages are the software containers in ROS 2. They contain libraries, executables, configuration files, and other resources needed for a specific functionality. A package might contain all the code needed for controlling a specific sensor or for implementing a particular algorithm.

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are the data structures that are passed between nodes. This publish-subscribe communication model allows for decoupled, asynchronous communication between different parts of your robot system.

For example, a camera node might publish image data to a `/camera/image_raw` topic, and multiple other nodes (like object detection, depth estimation, etc.) could subscribe to this topic to receive the same image data.

### Services
Services provide a request-response communication pattern, which is synchronous and allows for more direct interaction between nodes. A service call will block until the response is received.

### Actions
Actions are used for long-running tasks that may take a significant amount of time to complete, such as moving to a specific location or performing a complex manipulation task. Actions include feedback during execution and the ability to cancel ongoing tasks.

## Why ROS 2 for Humanoid Robotics?

ROS 2 has several advantages that make it particularly well-suited for humanoid robotics:

1. **Distributed Architecture**: Humanoid robots have many sensors and actuators distributed throughout the body. ROS 2's node-based architecture naturally supports this distributed design.

2. **Rich Ecosystem**: There are numerous existing packages for perception, navigation, manipulation, and other robotics functions that can be leveraged for humanoid robots.

3. **Simulation Integration**: ROS 2 works seamlessly with simulation environments like Gazebo, which is crucial for testing humanoid robot behaviors safely.

4. **Real-time Capabilities**: With proper configuration, ROS 2 can meet the real-time requirements needed for stable humanoid robot control.

5. **Multi-robot Support**: For scenarios involving multiple humanoid robots, ROS 2 provides the infrastructure for coordination.

## Basic Architecture

<img src="/img/module1/ros2-architecture.svg" alt="ROS 2 Architecture Diagram" style={{ width: "100%", height: "auto" }} />

The diagram above illustrates the basic architecture of a ROS 2 system. The ROS 2 middleware (RMW - ROS Middleware) handles the communication between nodes, abstracting the underlying transport mechanism (DDS - Data Distribution Service).

## Getting Started with ROS 2

To get started with ROS 2 development for humanoid robotics, you'll need to:

1. Install ROS 2 (recommended: latest stable version like Humble Hawksbill or Iron Irwini)
2. Set up your development workspace
3. Learn the basic command-line tools (ros2 run, ros2 launch, etc.)
4. Understand the build system (colcon)
5. Practice with basic examples before moving to complex humanoid systems

## Summary

ROS 2 provides the communication infrastructure that allows different components of a humanoid robot to work together. Understanding its fundamental concepts - nodes, packages, topics, services, and actions - is crucial for developing effective humanoid robot systems. The next sections will dive deeper into specific aspects of ROS 2, starting with how nodes communicate through topics and services.

## Installation and Setup

To start working with ROS 2, you'll need to install it on your system. The most common distributions include Rolling Ridley (latest development), Humble Hawksbill (LTS), and Iron Irwin. For humanoid robotics, it's recommended to use an LTS (Long Term Support) version like Humble Hawksbill.

### Installing ROS 2

On Ubuntu, you can install ROS 2 with these commands:

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
```

After installation, source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
```

### Creating a Workspace

For humanoid robotics development, you'll want to create a dedicated workspace:

```bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws
colcon build
source install/setup.bash
```

## Essential ROS 2 Commands

Here are the most important ROS 2 commands for humanoid robotics development:

- `ros2 run <package_name> <executable>`: Run a node
- `ros2 launch <package_name> <launch_file>`: Launch multiple nodes at once
- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Monitor messages on a topic
- `ros2 service list`: List all available services
- `ros2 node list`: List all active nodes
- `ros2 pkg create <pkg_name>`: Create a new package

## ROS 2 in Humanoid Robotics Context

In humanoid robotics, ROS 2 provides the communication infrastructure that allows different subsystems to work together. For example:

- **Perception Systems**: Camera nodes publish images to topics that perception algorithms subscribe to
- **Control Systems**: Joint controllers receive commands through topics or services from higher-level planners
- **Navigation**: Path planning algorithms publish trajectories that motion controllers follow
- **Behavior Trees**: Higher-level decision-making systems coordinate between different capabilities

## Community and Resources

The ROS community is vast and active, with extensive documentation, tutorials, and forums. For humanoid robotics specifically, the ROS wiki has dedicated sections for mobile manipulation, navigation, and control. The Humanoid Robot Working Group also provides specific resources and best practices.

## Summary

ROS 2 serves as the nervous system for humanoid robots, providing the communication infrastructure that allows different components to work together. Understanding its fundamental concepts - nodes, packages, topics, services, and actions - is crucial for developing effective humanoid robot systems. The distributed architecture naturally supports the complex, multi-component nature of humanoid robots.

## See Also

- [Module 2: The Digital Twin - Physics Simulation with Gazebo](../module2/2-1-physics-simulation-gazebo.md) - Learn how to simulate your ROS 2 nodes in a physics environment
- [Module 3: The AI-Robot Brain - Isaac Sim](../module3/3-1-nvidia-isaac-sim-photorealistic-simulation.md) - Explore advanced simulation for humanoid robotics
- [Module 1.3: Python Agents Bridging to ROS Controllers](./1-3-python-agents-ros-controllers.md) - Connect high-level decision making to ROS controllers

## Next Steps

In the next section, we'll explore ROS 2 Nodes, Topics, and Services in detail, with practical examples of how they're used in humanoid robotics applications.
