---
sidebar_position: 6
---

# Module 2 Mini-Project: Advanced Humanoid Simulation with Sensors

## Project Overview

In this mini-project, you will create a complete simulation environment for a humanoid robot equipped with multiple sensors. You'll integrate the concepts from all three chapters of Module 2 to build a realistic digital twin that includes physics simulation and sensor models.

## Learning Objectives

By completing this project, you will:
- Integrate a humanoid robot model with Gazebo physics simulation
- Configure multiple sensor types (IMU, LiDAR, depth camera)
- Implement ROS-Gazebo communication for real-time interaction
- Validate sensor data quality and realism

## Project Requirements

### 1. Enhanced URDF Model
Create a URDF model of a humanoid robot with:
- Complete kinematic chain for basic humanoid structure
- IMU sensor mounted on the torso
- 2D LiDAR sensor positioned appropriately
- Depth camera for 3D perception
- Proper physical properties and collision geometries

### 2. Simulation Environment
Create a Gazebo world with:
- Basic obstacles for sensor validation
- Proper lighting conditions
- Appropriate physics properties

### 3. ROS Integration
Implement ROS 2 nodes that:
- Subscribe to sensor data from the simulated robot
- Process sensor information
- Publish commands to control the robot in simulation

### 4. Launch System
Create launch files that start the complete simulation environment.

## Implementation Steps

### Step 1: Create the Enhanced URDF Model
Create `advanced_humanoid.urdf` in the `docs/module2/urdf/` directory:

```xml
<?xml version="1.0"?>
<robot name="advanced_humanoid">
  <!-- Base/Root link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.2"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Head with sensors -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
  </joint>

  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- IMU Sensor Link -->
  <joint name="torso_to_imu" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0.05 0 0.1"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- LiDAR Sensor Link -->
  <joint name="torso_to_lidar" type="fixed">
    <parent link="torso"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.2"/>
  </joint>

  <link name="lidar_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- Camera Sensor Link -->
  <joint name="torso_to_camera" type="fixed">
    <parent link="torso"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.1"/>
  </joint>

  <link name="camera_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- Gazebo-specific definitions -->
  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <!-- IMU Sensor Configuration -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100.0</update_rate>
      <imu>
        <noise>
          <type>gaussian</type>
          <rate>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>2e-6</bias_stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- LiDAR Sensor Configuration -->
  <gazebo reference="lidar_link">
    <sensor name="lidar_sensor" type="ray">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
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
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Depth Camera Configuration -->
  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <ros>
          <namespace>/humanoid</namespace>
        </ros>
        <frame_name>camera_link</frame_name>
        <baseline>0.2</baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Step 2: Create a Simple World File
Create `simple_world.world` in the `docs/module2/simulations/` directory:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Simple obstacles -->
    <model name="wall_1">
      <pose>2 0 1 0 0 0</pose>
      <link name="wall_1_link">
        <collision name="wall_1_collision">
          <geometry>
            <box>
              <size>0.1 4 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_1_visual">
          <geometry>
            <box>
              <size>0.1 4 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="box_1">
      <pose>-1 1 0.5 0 0 0</pose>
      <link name="box_1_link">
        <collision name="box_1_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_1_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="cylinder_1">
      <pose>-1 -1 1 0 0 0</pose>
      <link name="cylinder_1_link">
        <collision name="cylinder_1_collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="cylinder_1_visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Step 3: Create a Sensor Data Processing Node
Create `sensor_processor.py` in the `docs/module2/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Sensor Processor Node for Humanoid Robot Simulation
This node subscribes to various sensor data from the simulated humanoid robot
and processes it for analysis or further use.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np
import cv2


class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers for different sensor types
        self.imu_subscription = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/humanoid/scan',
            self.scan_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/humanoid/camera/depth/image_raw',
            self.camera_callback,
            10
        )

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Statistics for sensor data
        self.imu_count = 0
        self.scan_count = 0
        self.camera_count = 0

        self.get_logger().info('Sensor Processor Node Started')

    def imu_callback(self, msg):
        self.imu_count += 1

        # Extract orientation and angular velocity
        orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        angular_velocity = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

        if self.imu_count % 100 == 0:  # Log every 100th message
            self.get_logger().info(
                f'IMU - Count: {self.imu_count}, '
                f'Orientation: ({orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f}), '
                f'Angular Vel: ({angular_velocity[0]:.3f}, {angular_velocity[1]:.3f}, {angular_velocity[2]:.3f})'
            )

    def scan_callback(self, msg):
        self.scan_count += 1

        # Process LiDAR data - find minimum distance
        if len(msg.ranges) > 0:
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                min_distance = min(valid_ranges)
                max_distance = max(valid_ranges) if valid_ranges else float('inf')

                if self.scan_count % 50 == 0:  # Log every 50th message
                    self.get_logger().info(
                        f'LiDAR - Count: {self.scan_count}, '
                        f'Min Distance: {min_distance:.2f}m, '
                        f'Max Distance: {max_distance:.2f}m, '
                        f'Scan Points: {len(valid_ranges)}'
                    )

    def camera_callback(self, msg):
        self.camera_count += 1

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Basic image statistics
            if len(cv_image.shape) == 2:  # Grayscale depth image
                valid_pixels = cv_image[cv_image > 0]  # Only consider non-zero depth values
                if len(valid_pixels) > 0:
                    avg_depth = np.mean(valid_pixels)
                    min_depth = np.min(valid_pixels)
                    max_depth = np.max(valid_pixels)

                    if self.camera_count % 30 == 0:  # Log every 30th message
                        self.get_logger().info(
                            f'Camera - Count: {self.camera_count}, '
                            f'Avg Depth: {avg_depth:.2f}m, '
                            f'Depth Range: {min_depth:.2f}-{max_depth:.2f}m, '
                            f'Resolution: {cv_image.shape[1]}x{cv_image.shape[0]}'
                        )
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    sensor_processor = SensorProcessor()

    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create a Complete Launch File
Create `advanced_simulation.launch.py` in the `docs/module2/simulations/` directory:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='simple_world.world')

    # Path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('humanoid_robotics_book'),  # Replace with your package name
        'docs', 'module2', 'urdf', 'advanced_humanoid.urdf'
    )

    # Path to the world file
    world_file_path = os.path.join(
        get_package_share_directory('humanoid_robotics_book'),  # Replace with your package name
        'docs', 'module2', 'simulations', world_name
    )

    # Launch Gazebo with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            'gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file_path).read()
        }],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'advanced_humanoid', '-file', urdf_file_path, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # Sensor processor node
    sensor_processor = Node(
        package='humanoid_robotics_book',  # Replace with your package name
        executable='sensor_processor',  # This would be your compiled node
        name='sensor_processor',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='simple_world.world',
            description='Choose one of the world files from `/path/to/your/package/worlds`'
        ),
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # sensor_processor  # Uncomment if you have the sensor processor node
    ])
```

## Testing Your Implementation

1. Launch the complete simulation: `ros2 launch your_package advanced_simulation.launch.py`
2. In another terminal, check available topics: `ros2 topic list`
3. Verify sensor data is being published: `ros2 topic echo /humanoid/scan` or `ros2 topic echo /humanoid/imu/data`
4. Monitor the sensor processor node output for processed sensor information

## Extension Challenges

1. Add a navigation stack to the simulated robot to enable autonomous movement
2. Implement a simple SLAM algorithm using the LiDAR data
3. Create a teleoperation interface to manually control the simulated robot
4. Add more complex obstacles or dynamic elements to the simulation environment

## What You've Learned

This mini-project integrated all the concepts from Module 2:
- Created a complex humanoid URDF with multiple sensors
- Configured physics simulation in Gazebo
- Integrated multiple sensor types with realistic parameters
- Implemented ROS-Gazebo communication for real-time interaction
- Processed and analyzed sensor data from the simulation

These skills form the foundation for advanced robotics simulation and testing.