---
sidebar_position: 6
---

# Module 3 Quiz: AI-Robot Brain

Test your understanding of perception, Visual SLAM, Nav2, and synthetic data generation with these questions.

## Multiple Choice Questions

1. What does SLAM stand for?
   - A) Systematic Localization and Mapping
   - B) Simultaneous Localization and Mapping
   - C) Synchronized Localization and Movement
   - D) Sensor-based Localization and Mapping

2. In Visual SLAM, what is the main challenge known as the "chicken-and-egg" problem?
   - A) Processing speed vs. accuracy trade-off
   - B) The need to map the environment while simultaneously localizing in it
   - C) Camera calibration requirements
   - D) Feature detection in varying lighting

3. Which of the following is NOT a typical component of Nav2?
   - A) Planner Server
   - B) Controller Server
   - C) Recovery Server
   - D) Vision Server

4. What is the primary advantage of synthetic data generation in robotics?
   - A) It's always more accurate than real data
   - B) It provides unlimited training data without real-world collection costs and risks
   - C) It eliminates the need for real testing
   - D) It's faster to process than real data

5. In humanoid navigation, what makes path planning more complex than for wheeled robots?
   - A) Humanoid robots are slower
   - B) Humanoid robots require balance, footstep planning, and bipedal dynamics
   - C) Humanoid robots have more sensors
   - D) Humanoid robots are more expensive

6. What is domain randomization used for in synthetic data generation?
   - A) Reducing computational requirements
   - B) Increasing the diversity of synthetic data to improve real-world transfer
   - C) Making simulation more realistic
   - D) Reducing the amount of data needed

7. Which Isaac ROS package provides GPU-accelerated visual-inertial SLAM?
   - A) Isaac ROS Detection
   - B) Isaac ROS Visual SLAM
   - C) Isaac ROS Image Pipeline
   - D) Isaac ROS Point Cloud

8. What is the main purpose of costmaps in Nav2?
   - A) To store camera images
   - B) To represent the environment with obstacle information for navigation planning
   - C) To store robot joint positions
   - D) To manage robot communication

9. In Visual SLAM, what is triangulation used for?
   - A) Calculating robot speed
   - B) Converting 2D image features to 3D world coordinates
   - C) Detecting obstacles
   - D) Planning robot paths

10. What is a key challenge in humanoid navigation compared to wheeled navigation?
    - A) Processing power requirements
    - B) Maintaining balance while moving and planning foot placement
    - C) Sensor limitations
    - D) Communication protocols

## Short Answer Questions

11. Explain the difference between global and local path planning in Nav2.

12. Describe how Visual SLAM uses feature detection and tracking to build maps.

13. What are the advantages of using Isaac Sim for synthetic data generation?

14. How do recovery behaviors in Nav2 help with navigation failures?

15. What is the "reality gap" in synthetic data training, and how can it be addressed?

## Practical Exercise

16. Design a simple algorithm to detect when a Visual SLAM system has accumulated significant drift and needs loop closure.

## Answers

1. B
2. B
3. D
4. B
5. B
6. B
7. B
8. B
9. B
10. B

11. Global path planning finds a route from start to goal using a known map. Local path planning executes this plan while avoiding immediate obstacles and adjusting for dynamic conditions.

12. Visual SLAM detects distinctive features in images, tracks them across frames, estimates camera motion using these features, and triangulates their 3D positions to build a map.

13. Advantages include photorealistic rendering, physically accurate simulation, large-scale environment creation, and the ability to generate diverse, annotated training data safely.

14. Recovery behaviors provide strategies like clearing costmaps, rotating in place, or backing up when the navigation system fails to make progress.

15. The reality gap is the difference between synthetic and real data that can cause trained models to fail on real data. It can be addressed through domain adaptation, domain randomization, and mixed training approaches.