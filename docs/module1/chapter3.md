---
sidebar_position: 3
---

# Joint Control Examples

Welcome to Chapter 3 of Module 1! In this chapter, we will delve into the practical aspects of controlling humanoid robot joints using ROS 2. Building upon our understanding of URDF and TF, we'll explore how to send commands to individual joints and create basic motion sequences with Python.

## Fundamentals of Joint Control in ROS 2

Controlling a robot's joints involves publishing messages to specific ROS 2 topics. For most robotic platforms, especially those integrated with `ros2_control` (a set of packages that provides a generic and reusable framework for robot control), you'll interact with joint state and joint command interfaces.

### Key Concepts:

-   **Joint State**: Provides the current position, velocity, and effort of each joint. This is typically published by the robot's hardware interface or simulation.
-   **Joint Command**: These are the desired position, velocity, or effort values that you want to send to the robot's joints. You will publish these commands.
-   **Controller Manager**: In `ros2_control`, this manages the various controllers (e.g., position, velocity, effort controllers) that interpret your commands and send them to the hardware.

## Python Example: Controlling a Single Joint (Conceptual)

Let's consider a simplified conceptual example of how you might send a position command to a single joint of a humanoid robot using `rclpy` (the Python client library for ROS 2).

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 # Or a more specific joint command message type

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.publisher_ = self.create_publisher(Float64, '/joint_name/commands', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.joint_position = 0.0
        self.direction = 1 # 1 for increasing, -1 for decreasing
        self.get_logger().info('Joint Commander Node started')

    def timer_callback(self):
        msg = Float64()
        msg.data = self.joint_position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Simple oscillation for demonstration
        self.joint_position += (0.1 * self.direction)
        if self.joint_position >= 1.0 or self.joint_position <= -1.0:
            self.direction *= -1
            self.get_logger().info('Changing direction')

def main(args=None):
    rclpy.init(args=args)
    joint_commander = JointCommander()
    rclpy.spin(joint_commander)
    joint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation of the code:**

1.  **Node Initialization**: We create a ROS 2 node called `joint_commander`.
2.  **Publisher**: A publisher is created to send `Float64` messages (representing a single joint position) to the topic `/joint_name/commands`. *Note: The exact message type and topic name can vary based on your robot's configuration and `ros2_control` setup.*
3.  **Timer Callback**: A timer is set up to call `timer_callback` every 0.5 seconds.
4.  **Publishing Commands**: Inside `timer_callback`, a `Float64` message is created with a target joint position (`self.joint_position`) and published.
5.  **Simple Oscillation**: The `self.joint_position` variable is made to oscillate between -1.0 and 1.0 radians, simulating a basic back-and-forth joint movement.

## Controlling Multiple Joints (Conceptual)

For controlling multiple joints, you would typically use a message type that can carry an array of joint names and their corresponding command values, such as `JointState` (though `JointState` is often for *reading* states, similar structures are used for commands). A common approach with `ros2_control` is to use specific command interfaces provided by a `JointGroupPositionController` or similar.

```python
# Conceptual example - specific message types and topics will depend on ros2_control setup
import rclpy
from rclpy.node import Node
# from control_msgs.msg import JointTrajectoryControllerState # Example for state, commands are similar
# For commands, you might use JointTrajectory or similar custom messages

# Placeholder for a multi-joint command message (often defined in a robot's specific control packages)
class MultiJointCommand:
    def __init__(self, joint_names, positions):
        self.joint_names = joint_names
        self.positions = positions

class MultiJointCommander(Node):
    def __init__(self):
        super().__init__('multi_joint_commander')
        # Replace with actual message type and topic for your robot's controller
        self.publisher_ = self.create_publisher(MultiJointCommand, '/multi_joint_controller/commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Multi-Joint Commander Node started')

        self.joint_names = ['left_shoulder_joint', 'right_shoulder_joint', 'elbow_joint']
        self.current_positions = [0.0, 0.0, 0.0]
        self.target_positions_cycle = [
            [0.5, -0.5, 0.0],  # Pose 1
            [-0.5, 0.5, 0.0],  # Pose 2
            [0.0, 0.0, 1.0]   # Pose 3
        ]
        self.cycle_index = 0

    def timer_callback(self):
        target_pose = self.target_positions_cycle[self.cycle_index]
        msg = MultiJointCommand(self.joint_names, target_pose)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing command for joints {self.joint_names} to positions {target_pose}')

        self.cycle_index = (self.cycle_index + 1) % len(self.target_positions_cycle)

def main(args=None):
    rclpy.init(args=args)
    multi_joint_commander = MultiJointCommander()
    rclpy.spin(multi_joint_commander)
    multi_joint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key takeaways for multi-joint control:**

-   You define a list of joint names that your controller will manage.
-   You prepare a corresponding list of target positions (or velocities/efforts) for these joints.
-   A single message containing both the joint names and their commands is published to the controller's command topic.
-   Complex movements are achieved by sequencing these commands over time.

## What's Next?

In the upcoming projects, you will get to implement these concepts with a simulated robot. You will learn to:

-   Identify the correct ROS 2 topics and message types for your robot.
-   Write Python scripts to command single and multiple joints.
-   Observe the robot's response in a simulation environment.

This hands-on experience will solidify your understanding of how to bring your humanoid robot to life through precise joint control!
