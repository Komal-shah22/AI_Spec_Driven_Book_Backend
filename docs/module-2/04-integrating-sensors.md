# Chapter 4: Integrating Sensors in Simulation

## Intro

Sensors are the eyes and ears of a robot, providing crucial information about its environment and internal state. In simulation, we can model various sensors to generate realistic data for testing and development. This chapter will guide you through adding common sensors like a LiDAR and an IMU to our robot's URDF model and visualizing their data.

## Concept

Robot sensors gather data from the environment (e.g., distance, images, orientation) or from the robot itself (e.g., joint angles, battery level). In simulation, these sensors are modeled to produce data that mimics their physical counterparts, allowing us to test sensor-based algorithms.

### LiDAR (Light Detection and Ranging)

A LiDAR sensor measures distances to objects by emitting pulsed laser light and measuring the time it takes for the reflected light to return. It creates a point cloud representing the environment.

### IMU (Inertial Measurement Unit)

An IMU measures a robot's orientation, angular velocity, and linear acceleration. It typically consists of accelerometers, gyroscopes, and sometimes magnetometers.

## Hands-on: Adding Sensors to the URDF

We will now enhance our `simple_robot.urdf` by adding a LiDAR and an IMU sensor.

First, open your `simple_robot.urdf` file:

```bash
# Assuming you are in your ROS 2 workspace src directory
# nano robot_description/urdf/simple_robot.urdf
```

### Adding a LiDAR Sensor

Add the following `<link>` and `<joint>` elements within the `<robot>` tags of your `simple_robot.urdf` file, after the caster wheel joint.

```xml
  <!-- LiDAR Link -->
  <link name="hokuyo_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/hokuyo.dae"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/hokuyo.dae"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.000008" ixy="0.0" ixz="0.0" iyy="0.000008" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- LiDAR Joint -->
  <joint name="hokuyo_joint" type="fixed">
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="hokuyo_link"/>
  </joint>

  <!-- Gazebo specific plugins for LiDAR -->
  <gazebo reference="hokuyo_link">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <argument>~/out</argument>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>hokuyo_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

*(Note: The `<mesh>` tag for the LiDAR requires a 3D model file, e.g., `hokuyo.dae`. For a complete tutorial, you would need to provide this mesh file in `robot_description/meshes/`.)*

### Adding an IMU Sensor

Add the following `<link>` and `<joint>` elements after the LiDAR section. For simplicity, we'll attach the IMU directly to the `base_link`.

```xml
  <!-- IMU Link -->
  <link name="imu_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.01"/>
      <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- IMU Joint -->
  <joint name="imu_joint" type="fixed">
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="imu_link"/>
  </joint>

  <!-- Gazebo specific plugins for IMU -->
  <gazebo reference="imu_link">
    <sensor type="imu" name="imu_sensor">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <topic>imu</topic>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <argument>~/out</argument>
          <remapping>~/out:=imu</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>
```

## Summary

You have successfully added a LiDAR and an IMU sensor to your robot's URDF model, complete with Gazebo plugins to simulate their behavior. In the next sections, we will explore how to visualize this sensor data using ROS 2 tools like RViz.

### Visualizing Sensor Data with RViz2

Once your robot with sensors is launched in Gazebo, you can visualize the sensor data and the robot's transformations using RViz2.

1.  **Launch RVviz2**:
    ```bash
    rviz2
    ```
2.  **Add `RobotModel` display**: To see your robot.
3.  **Add `LaserScan` display**: Subscribe to `/scan` topic to visualize LiDAR data.
4.  **Add `IMU` display**: Subscribe to `/imu` topic to visualize IMU data.

### Understanding TF2 (Transformations)

**TF2** is a ROS 2 package that keeps track of multiple coordinate frames and maintains the relationships between them in a tree structure. This is crucial for robotics, as sensors, robot parts, and the environment each have their own coordinate systems. TF2 allows you to ask questions like: "What is the position of the LiDAR sensor relative to the robot's base at this specific time?"

**Key Concepts**:

-   **Coordinate Frames**: Each link in your URDF defines a coordinate frame. Sensors also have their own frames.
-   **Transforms**: The mathematical description of the relationship between two coordinate frames.
-   **Broadcasters**: Nodes that publish transforms, indicating where frames are relative to each other.
-   **Listeners**: Nodes that receive transforms and can look up the relationship between any two frames at any time.

In our `simple_robot.urdf`, the `<joint>` tags implicitly define the static transforms between the `base_link` and the `left_wheel_link`, `right_wheel_link`, `caster_wheel_link`, `hokuyo_link`, and `imu_link`. These static transforms are automatically provided to the TF2 tree when the robot model is loaded.

## Code Samples: Subscribing to Sensor Data (rclpy)

*(This section will refer to the `sensor_logger.py` code example from T3.6)*

To process the data from the simulated sensors, you need to write ROS 2 nodes that subscribe to the sensor topics. The `sensor_logger.py` script we created earlier demonstrates how to subscribe to `/scan` (from the LiDAR) and `/imu` (from the IMU) topics and print their data.

Run the `sensor_logger.py` node:

```bash
# In your ROS 2 workspace
# source install/setup.bash
# ros2 run sensor_subscribers sensor_logger.py
```

This will show you the raw data streaming from your simulated sensors.

## Exercises

1.  **Add a Camera Sensor**: Extend your `simple_robot.urdf` to include a camera sensor. Use Gazebo plugins to simulate a camera and visualize its feed in RViz2.
2.  **TF2 Listener**: Write a simple `rclpy` node that uses the TF2 listener to find the transform between the `base_link` and your newly added camera sensor.
3.  **Sensor Data Interpretation**: Based on the `sensor_logger.py` output, describe what each piece of data from the LiDAR and IMU tells you about the robot's environment and state.
