---
sidebar_position: 5
---

# Module 1 Quiz: ROS 2 Fundamentals

Test your understanding of ROS 2 core concepts, URDF, TF, and joint control with these questions.

## Multiple Choice Questions

1. What is a ROS 2 Node?
   - A) A physical component of a robot
   - B) An independent process that performs computations and communicates with other nodes
   - C) A type of message used for communication
   - D) A configuration file for robot description

2. In ROS 2, what is the communication pattern used for Topics?
   - A) Request/Response
   - B) Synchronous communication
   - C) Publish/Subscribe
   - D) Peer-to-peer

3. What does URDF stand for?
   - A) Universal Robot Description Framework
   - B) Unified Robot Description Format
   - C) Universal Robot Design Format
   - D) Unified Robot Development Framework

4. What is the primary purpose of TF (Transform) in ROS?
   - A) To describe robot physical properties
   - B) To track coordinate frames and their relationships over time
   - C) To control robot joints
   - D) To manage ROS 2 nodes

5. Which ROS 2 communication type is best suited for long-running tasks with feedback?
   - A) Topics
   - B) Services
   - C) Actions
   - D) Parameters

6. What is the typical message type used for publishing single joint positions in ROS 2?
   - A) JointState
   - B) Float64
   - C) JointCommand
   - D) PositionMsg

7. What is the primary advantage of using ros2_control framework?
   - A) It simplifies URDF creation
   - B) It provides hardware abstraction and reusability
   - C) It improves TF performance
   - D) It reduces memory usage

8. In a URDF file, what element defines the relationship between two links?
   - A) `<joint>`
   - B) `<connection>`
   - C) `<link>`
   - D) `<transform>`

9. Which communication pattern in ROS 2 is synchronous?
   - A) Topics
   - B) Services
   - C) Actions
   - D) Parameters

10. What is the typical update frequency for joint state publishers?
    - A) 1 Hz
    - B) 10 Hz
    - C) 50-100 Hz
    - D) 1000 Hz

## Short Answer Questions

11. Explain the difference between a ROS 2 Topic and a Service, providing an example of when you would use each.

12. Describe the relationship between URDF and TF in the context of a humanoid robot.

13. What are the three main types of joint commands that can be sent to a robot using ros2_control?

14. Why is it important to understand both joint states and joint commands when controlling a humanoid robot?

15. Describe a scenario where you would use ROS 2 Actions instead of Topics or Services.

## Practical Exercise

16. Write a Python function that creates a ROS 2 node that publishes joint position commands to move a single joint from -0.5 radians to 0.5 radians in 0.1 radian increments, with a 1-second pause between each increment.

## Answers

1. B
2. C
3. B
4. B
5. C
6. B (Float64 is commonly used for single joint positions)
7. B
8. A
9. B
10. C (Joint states are typically published at 50-100 Hz for real-time control)
11. Topics provide asynchronous publish/subscribe communication (e.g., sensor data publishing). Services provide synchronous request/response communication (e.g., requesting a specific computation result).
12. URDF describes the static structure of the robot (links and joints), while TF manages the dynamic transformations between coordinate frames over time based on joint states.
13. Position, velocity, and effort commands.
14. Joint states provide feedback about the current position, velocity, and effort of joints, while joint commands tell the robot where to move. Both are needed for closed-loop control.
15. Actions are used for long-running tasks that require feedback and the ability to cancel, such as navigation to a goal or complex manipulation tasks.