---
sidebar_label: Robot Structure & URDF
title: Robot Structure & URDF
description: Understanding Unified Robot Description Format for robot modeling
---

# Robot Structure & URDF

## Introduction to URDF

Unified Robot Description Format (URDF) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, and other components.

## URDF Structure

### Links
Links represent rigid bodies of the robot. Each link has:
- Physical properties (mass, inertia)
- Visual properties (shape, color, mesh)
- Collision properties (collision geometry)

### Joints
Joints connect links and define how they can move relative to each other. Joint types include:
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Prismatic**: Linear movement along an axis
- **Continuous**: Continuous rotation (like a revolute joint without limits)
- **Floating**: 6 DOF movement

## Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Child link connected via joint -->
  <link name="child_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="base_to_child" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Advanced URDF Features

### Transmissions
Define how actuators are connected to joints, important for simulation and control.

### Gazebo Extensions
Include Gazebo-specific properties within URDF for simulation:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
</gazebo>
```

### Xacro
Xacro is an XML macro language that simplifies complex URDF files by allowing variables, math expressions, and macros.

## Best Practices

- Use consistent naming conventions
- Include proper inertial properties for accurate simulation
- Separate visual and collision geometry appropriately
- Organize complex robots in modular xacro files
- Validate URDF files using tools like check_urdf
