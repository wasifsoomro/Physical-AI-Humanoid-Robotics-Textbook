# Module 2: Digital Twin Simulation (Gazebo + Unity)

## Chapter 2: Physics and Sensors

### Introduction

In the previous chapter, we learned how URDF describes the physical structure of our humanoid robot. Now, we'll explore how Gazebo adds realistic physics to these models and how we can equip our digital robots with various sensors to perceive their simulated world. Understanding physics and sensors is crucial for creating intelligent robots that can interact with their environment.

### Realistic Physics in Gazebo

Gazebo isn't just about looks; it has a powerful physics engine that makes simulations behave like the real world. This engine calculates things like:

*   **Gravity**: How objects fall.
*   **Collisions**: What happens when two objects touch or bump into each other.
*   **Friction**: How surfaces resist motion when sliding or rolling.
*   **Inertia**: How much an object resists changes in its motion.

These properties are defined in the URDF file using `<collision>` and `<inertial>` tags for each link. For example, the `<mass>` tag inside `<inertial>` tells Gazebo how heavy a link is, which affects how it moves and interacts with forces.

### Common Sensors for Humanoid Robots

Robots need to sense their environment to make smart decisions. In Gazebo, we can add virtual sensors to our URDF models. Here are some common sensors used in humanoid robotics:

1.  **IMU (Inertial Measurement Unit)**: Imagine a robot's inner ear. An IMU measures a robot's orientation, angular velocity (how fast it's turning), and linear acceleration (how fast it's speeding up or slowing down). It helps the robot know its balance and movement.
    *   **Measurements**: Orientation (Roll, Pitch, Yaw), Angular Velocity (X, Y, Z), Linear Acceleration (X, Y, Z).

2.  **LiDAR (Light Detection and Ranging)**: Think of a robot's eyes that use laser beams. LiDAR sensors emit laser pulses and measure the time it takes for them to return after hitting an object. This creates a detailed 2D or 3D map of the robot's surroundings.
    *   **Measurements**: Distance to objects, creating point clouds.

3.  **Depth Camera**: Similar to how humans see depth with two eyes, a depth camera (like a Microsoft Kinect or Intel RealSense) captures both color images and depth information. It can tell you how far away objects are from the camera, which is great for obstacle avoidance and object manipulation.
    *   **Measurements**: Color image (RGB), Depth image (distance to pixels).

### Adding Sensors to URDF for Gazebo

To add a sensor to your robot in Gazebo, you typically define it within a `<link>` in your URDF using a `<sensor>` tag. This tag then specifies the sensor type (e.g., `imu`, `ray` for LiDAR, `camera` for depth camera) and its properties (e.g., field of view, update rate, noise). Gazebo plugins are often used to bridge these simulated sensor readings to ROS 2 topics.

Here's a conceptual example for an IMU sensor:

```xml
<robot name="humanoid_with_imu">
  <link name="imu_link">
    <!-- ... visual and inertial properties ... -->
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <!-- ... IMU specific parameters ... -->
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>  <namespace>/humanoid</namespace>  </ros>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </link>

  <joint name="torso_to_imu" type="fixed">
    <parent link="torso_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

</robot>
```

### Summary

This chapter introduced us to the world of realistic physics in Gazebo, where properties like gravity, collisions, and friction make our simulated robots behave authentically. We also explored essential sensors for humanoid robots: IMUs for orientation, LiDAR for mapping, and depth cameras for perceiving distance. Understanding how to integrate these sensors into our URDF models is key to building intelligent and interactive digital twins.

### Quiz Questions

1.  List three physical phenomena that Gazebo's physics engine simulates.
2.  What information does an IMU provide to a robot?
3.  How does a LiDAR sensor work to create a map of its surroundings?
4.  What is the main difference between a regular camera and a depth camera in robotics?
5.  In a URDF, what tag is typically used to define a sensor, and what are Gazebo plugins used for in this context?
