---
title: Understanding URDF for Humanoids
sidebar_position: 4
description: Learn how to describe humanoid robot structure using Unified Robot Description Format (URDF)
---

# Understanding URDF for Humanoids

## Overview

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF is crucial as it defines the physical structure, kinematic properties, and visual appearance of the robot. Understanding URDF is essential for simulation, visualization, and control of humanoid robots in ROS 2.

## URDF Basics

URDF describes a robot as a collection of links connected by joints. Each link represents a rigid body, and each joint defines how two links move relative to each other. For humanoid robots, this allows for modeling complex kinematic chains like arms, legs, and spines.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Links represent rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF Elements for Humanoid Robots

### Links

Links represent rigid bodies in the robot. For humanoid robots, common links include:

- `base_link`: The main reference frame (often the pelvis or torso)
- `head_link`: The head
- `left_arm_base_link`, `right_arm_base_link`: Arm mounting points
- `left_upper_arm_link`, `left_forearm_link`, etc.: Individual arm segments
- `left_hip_link`, `left_thigh_link`, etc.: Leg segments

### Joints

Joints define the relationship between links. For humanoid robots, you'll typically use:

- `revolute`: Rotational joints with limited range (elbows, knees)
- `continuous`: Rotational joints without limits (shoulders, hips)
- `prismatic`: Linear joints (rarely used in humanoids)
- `fixed`: Rigid connections (head to neck)

### Materials and Colors

You can define materials to give your robot visual properties:

```xml
<material name="blue">
  <color rgba="0.0 0.0 1.0 1.0"/>
</material>

<material name="white">
  <color rgba="1.0 1.0 1.0 1.0"/>
</material>
```

## Complete Humanoid URDF Example

Here's a more complete example of a simplified humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_model">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_to_elbow" type="revolute">
    <parent link="left_upper_arm_link"/>
    <child link="left_forearm_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_forearm_link">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="torso_to_left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh_link"/>
    <origin xyz="-0.05 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_thigh_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_thigh_to_knee" type="revolute">
    <parent link="left_thigh_link"/>
    <child link="left_shin_link"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_shin_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.025" ixy="0.0" ixz="0.0" iyy="0.025" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>
</robot>
```

## URDF for Simulation

For use in simulation environments like Gazebo, you'll often need additional elements:

```xml
<!-- Add gazebo-specific properties -->
<gazebo reference="base_link">
  <material>Gazebo/White</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<!-- Transmission elements for control -->
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_to_left_shoulder">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## URDF Best Practices for Humanoids

### 1. Kinematic Chain Design
- Design kinematic chains that match your actual robot hardware
- Consider the degrees of freedom needed for desired motions
- Ensure joints have appropriate limits based on physical constraints

### 2. Mass and Inertia Properties
- Accurately model mass distribution for realistic simulation
- Use CAD software to calculate inertia tensors when possible
- Balance accuracy with computational efficiency

### 3. Visual and Collision Models
- Use simplified collision models for better performance
- Ensure visual models match the physical appearance of your robot
- Consider using meshes for complex geometries

### 4. Naming Conventions
- Use consistent, descriptive names for links and joints
- Follow ROS conventions (e.g., `left_arm_joint1` instead of `j1`)

## URDF Visualization and Validation

To visualize your URDF model:

```bash
# Launch RViz with robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat your_robot.urdf)
```

To validate your URDF:

```bash
# Check for syntax errors
check_urdf your_robot.urdf

# View in RViz
ros2 launch urdf_tutorial display.launch.py model_path:=your_robot.urdf
```

## URDF and ROS 2 Integration

URDF models integrate with ROS 2 through:

1. **Robot State Publisher**: Publishes transforms based on joint states
2. **TF2**: Provides coordinate transformations between links
3. **MoveIt**: Uses URDF for motion planning
4. **Controllers**: Reference URDF joints for control

## Common URDF Issues in Humanoid Robots

### 1. Floating Base
Humanoid robots often have a floating base (no fixed world connection). You may need to add a virtual fixed joint:

```xml
<joint name="world_to_base" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### 2. Complex Joint Limits
Humanoid joints often have complex, coupled limits. Consider using joint limits that reflect the physical constraints of your robot.

### 3. Center of Mass
Accurate center of mass calculation is crucial for balance and locomotion planning in humanoid robots.

## Summary

URDF is fundamental to humanoid robotics in ROS 2, providing the description needed for simulation, visualization, and control. When creating URDF models for humanoid robots, focus on accurate kinematic descriptions, realistic physical properties, and appropriate simplifications for computational efficiency.

Understanding URDF is essential for working with humanoid robots in ROS 2, as it provides the foundation for all downstream tools and algorithms.

## Next Steps

This completes Module 1: The Robotic Nervous System (ROS 2). You now have a comprehensive understanding of ROS 2 fundamentals, communication patterns, agent integration, and robot description. The next module will cover the Digital Twin aspects of humanoid robotics using Gazebo and Unity.