---
id: 03-building-a-robot-model
slug: /module-2/building-a-robot-model
---

# Chapter 3: Building a Robot Model (URDF)

## Intro

In the previous chapter, we explored simulation environments. Now, it's time to bring our robot to life in that virtual world. This chapter will guide you through creating a robot model using the Unified Robot Description Format (URDF).

## Concept

The **Unified Robot Description Format (URDF)** is an XML file format used in ROS to describe all aspects of a robot, including its kinematic and dynamic properties, visual appearance, and collision properties. A URDF model is essential for visualizing a robot in tools like RViz and for simulating it in Gazebo.

### Key URDF Elements

-   **`<link>`**: Represents a rigid body of the robot (e.g., chassis, wheel, sensor). Each link has:
    -   **`<visual>`**: Defines the graphical properties (geometry, color).
    -   **`<collision>`**: Defines the physical properties for collision detection.
    -   **`<inertial>`**: Defines the mass and inertia properties for physics simulation.
-   **`<joint>`**: Connects two links, defining their kinematic relationship. Each joint has:
    -   **`type`**: E.g., `continuous` (for wheels), `revolute`, `fixed`.
    -   **`<parent>`** and **`<child>`**: The two links it connects.
    -   **`<origin>`**: The transform (position and orientation) of the child link relative to the parent link.
    -   **`<axis>`**: The axis of rotation for rotational joints.

## Hands-on: Creating a Simple Differential Drive Robot URDF

We will now create the URDF for the simple differential drive robot we designed in the previous phase.

First, create a new ROS 2 package for your robot description. For the purpose of this tutorial, we assume you have a ROS 2 workspace set up.

Navigate into your ROS 2 workspace `src` directory and create a `robot_description` package (if not already done manually as part of the tasks):

```bash
# Assuming you are in your ROS 2 workspace root, or navigate there
# For example: cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake robot_description
```

Navigate into the `robot_description` package and create a `urdf` directory:

```bash
cd robot_description
mkdir urdf
```

Now, create a file named `simple_robot.urdf` inside the `urdf` directory:

```bash
touch urdf/simple_robot.urdf
```

Open `urdf/simple_robot.urdf` and add the following content. This URDF defines the `base_link`, two wheels, and a caster wheel.

```xml
<?xml version="1.0"?>
<robot name="simple_differential_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.005833" ixy="0.0" ixz="0.0" iyy="0.005833" iyz="0.0" izz="0.004167"/>
    </inertial>
  </link>

  <!-- Left Wheel Link -->
  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.000125" ixy="0.0" ixz="0.0" iyy="0.000125" iyz="0.0" izz="0.00025"/>
    </inertial>
  </link>

  <!-- Right Wheel Link -->
  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.000125" ixy="0.0" ixz="0.0" iyy="0.000125" iyz="0.0" izz="0.00025"/>
    </inertial>
  </link>

  <!-- Caster Wheel Link -->
  <link name="caster_wheel_link">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="silver">
        <color rgba="0.8 0.8 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.01"/>
      <inertia ixx="0.0000008" ixy="0.0" ixz="0.0" iyy="0.0000008" iyz="0.0" izz="0.0000008"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_link_to_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0.1 -0.11 0" rpy="-1.57079632679 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="base_link_to_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0.1 0.11 0" rpy="1.57079632679 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="base_link_to_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel_link"/>
    <origin xyz="0.13 0 -0.04" rpy="0 0 0"/>
  </joint>

</robot>
```

## Summary

You have successfully created a URDF model for a simple differential drive robot. This model can now be visualized in RViz and simulated in Gazebo. In the next chapter, we will learn how to add sensors to this robot model and integrate them with ROS 2.

## Exercises

1.  **Explain URDF Elements**: Describe the purpose of `<link>` and `<joint>` elements in URDF, and what sub-elements they typically contain.
2.  **Modify Robot Dimensions**: Change the dimensions of the `base_link` in `simple_robot.urdf` and observe the changes in RViz (after launching the robot).
3.  **Add a New Link**: Add a simple new link (e.g., a camera mount) to your robot's URDF, attaching it to the `base_link` with a fixed joint.
