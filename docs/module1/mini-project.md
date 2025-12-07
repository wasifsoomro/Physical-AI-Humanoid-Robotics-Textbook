---
sidebar_position: 6
---

# Module 1 Mini-Project: Humanoid Arm Controller

## Project Overview

In this mini-project, you will create a complete ROS 2 system that controls a simple humanoid arm with multiple joints. You'll implement both the publisher (commander) and subscriber (monitor) nodes to demonstrate your understanding of ROS 2 communication patterns and joint control.

## Learning Objectives

By completing this project, you will:
- Implement multi-joint control using ROS 2
- Create a publisher node that sends joint commands
- Create a subscriber node that monitors joint states
- Integrate concepts from all three chapters of Module 1

## Project Requirements

### 1. URDF Model
Create a simple URDF model of a humanoid arm with:
- Shoulder joint (revolute)
- Elbow joint (revolute)
- Wrist joint (revolute)

### 2. Joint Commander Node
Create a Python node that:
- Publishes joint commands to move the arm in a coordinated pattern
- Moves the arm through at least 3 different poses
- Uses appropriate message types for joint commands

### 3. Joint Monitor Node
Create a Python node that:
- Subscribes to joint state messages
- Prints the current joint positions to the console
- Logs the state changes over time

### 4. Launch File
Create a launch file that starts all required nodes together.

## Implementation Steps

### Step 1: Create the URDF Model
Create a file `humanoid_arm.urdf` in the `docs/module1/projects/` directory:

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
  </link>

  <!-- Shoulder joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8"/>
      </material>
    </visual>
  </link>

  <!-- Wrist joint -->
  <joint name="wrist_joint" type="revolute">
    <parent link="forearm"/>
    <child link="hand"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.08"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 0.8"/>
      </material>
    </visual>
  </link>
</robot>
```

### Step 2: Create the Joint Commander Node
Create `arm_commander.py` in the `docs/module1/projects/` directory:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')

        # Create publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer to send commands at regular intervals
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Define joint names
        self.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']

        # Define different poses for the arm
        self.poses = [
            [0.0, 0.0, 0.0],           # Initial position
            [0.5, -0.5, 0.2],          # Pose 1: Arm bent
            [-0.3, 0.8, -0.4],         # Pose 2: Different configuration
            [0.0, 0.0, 0.0]            # Return to initial
        ]

        self.current_pose_index = 0

        self.get_logger().info('Arm Commander Node Started')

    def timer_callback(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.poses[self.current_pose_index]

        # Publish the message
        self.joint_pub.publish(msg)

        self.get_logger().info(f'Published joint positions: {self.poses[self.current_pose_index]}')

        # Move to next pose
        self.current_pose_index = (self.current_pose_index + 1) % len(self.poses)

def main(args=None):
    rclpy.init(args=args)
    arm_commander = ArmCommander()

    try:
        rclpy.spin(arm_commander)
    except KeyboardInterrupt:
        pass
    finally:
        arm_commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create the Joint Monitor Node
Create `joint_monitor.py` in the `docs/module1/projects/` directory:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')

        # Create subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Joint Monitor Node Started')

    def joint_state_callback(self, msg):
        self.get_logger().info('--- Joint States Received ---')
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                position = msg.position[i]
                self.get_logger().info(f'{name}: {position:.3f} rad')
        self.get_logger().info('---------------------------')

def main(args=None):
    rclpy.init(args=args)
    joint_monitor = JointMonitor()

    try:
        rclpy.spin(joint_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        joint_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Create a Launch File
Create `arm_control_launch.py` in the `docs/module1/projects/` directory:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',  # Replace with actual package name in real implementation
            executable='arm_commander',
            name='arm_commander',
            output='screen'
        ),
        Node(
            package='your_package_name',  # Replace with actual package name in real implementation
            executable='joint_monitor',
            name='joint_monitor',
            output='screen'
        )
    ])
```

## Testing Your Implementation

1. Run your commander node: `ros2 run your_package arm_commander`
2. In another terminal, run your monitor node: `ros2 run your_package joint_monitor`
3. Observe how the joint monitor receives and displays the joint positions published by the commander
4. Verify that the arm moves through the different poses as programmed

## Extension Challenges

1. Modify the commander to create a smooth trajectory between poses instead of jumping between them
2. Add velocity and effort values to the joint state messages
3. Implement a service that allows external nodes to request specific poses
4. Create a simple GUI that allows manual control of the arm joints

## What You've Learned

This mini-project integrated all the concepts from Module 1:
- Created a URDF model for a simple humanoid arm
- Implemented ROS 2 publisher and subscriber nodes
- Controlled multiple joints simultaneously
- Used appropriate message types for joint communication
- Demonstrated the publish/subscribe communication pattern

These skills form the foundation for more complex robotics applications in the upcoming modules.