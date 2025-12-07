---
sidebar_position: 7
---

# Module 3 Mini-Project: Integrated Perception and Navigation System

## Project Overview

In this mini-project, you will create an integrated system that combines perception, mapping, and navigation for a humanoid robot. You'll implement a complete AI-Robot Brain that can perceive its environment, create maps, and navigate autonomously using Isaac ROS concepts.

## Learning Objectives

By completing this project, you will:
- Integrate perception, mapping, and navigation systems
- Implement a complete SLAM pipeline
- Configure Nav2 for humanoid-specific navigation
- Generate and utilize synthetic data for training
- Create a cohesive AI-Robot Brain system

## Project Requirements

### 1. Perception Module
Create a perception system that:
- Processes camera data for feature detection
- Integrates IMU data for improved tracking
- Provides environmental understanding
- Outputs semantic information about the scene

### 2. Mapping and Localization Module
Implement SLAM functionality that:
- Creates environmental maps using visual data
- Estimates robot position within the map
- Handles drift correction and loop closure
- Maintains map consistency over time

### 3. Navigation Module
Build a navigation system that:
- Plans paths through the environment
- Avoids obstacles in real-time
- Accounts for humanoid-specific constraints
- Executes smooth, balanced movement

### 4. Synthetic Data Pipeline
Develop a synthetic data generation system that:
- Creates diverse training scenarios
- Provides perfect ground truth annotations
- Enables domain randomization
- Accelerates AI model development

## Implementation Steps

### Step 1: Create the Perception Node
Create `integrated_perception.py` in the `docs/module3/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Integrated Perception Node for Humanoid Robot
Combines visual, inertial, and semantic perception.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


class IntegratedPerception(Node):
    def __init__(self):
        super().__init__('integrated_perception')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # TF2 broadcaster and buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.feature_pub = self.create_publisher(MarkerArray, 'perception/features', 10)
        self.object_pub = self.create_publisher(String, 'perception/objects', 10)
        self.semantic_pub = self.create_publisher(MarkerArray, 'perception/semantic', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Internal state
        self.camera_matrix = None
        self.imu_data = None
        self.feature_detector = cv2.ORB_create(nfeatures=500)
        self.prev_features = None
        self.feature_tracks = {}  # Track features over time

        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_callback)  # 10 Hz

        self.get_logger().info('Integrated Perception Node Started')

    def camera_info_callback(self, msg):
        """Store camera calibration information"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def imu_callback(self, msg):
        """Store IMU data for sensor fusion"""
        self.imu_data = {
            'orientation': (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            'angular_velocity': (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            'linear_acceleration': (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        }

    def process_image(self, image):
        """Process image to extract perception information"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect features
        keypoints = self.feature_detector.detect(gray, None)
        keypoints, descriptors = self.feature_detector.compute(gray, keypoints)

        # Store features for tracking
        self.prev_features = [(kp.pt[0], kp.pt[1]) for kp in keypoints]

        # Simple object detection (for demonstration)
        # In a real system, this would use deep learning models
        objects = self.detect_simple_objects(image)

        # Publish detected objects
        if objects:
            for obj_type, center in objects:
                obj_msg = String()
                obj_msg.data = f'{obj_type}: ({center[0]:.1f}, {center[1]:.1f})'
                self.object_pub.publish(obj_msg)

    def detect_simple_objects(self, image):
        """Simple object detection for demonstration (in real system, use deep learning)"""
        objects = []

        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red regions (as an example)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        red_mask = mask1 + mask2
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small detections
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    objects.append(('red_object', (cx, cy)))

        return objects

    def process_callback(self):
        """Periodic processing callback"""
        # Publish visualization markers for detected features
        if self.prev_features:
            marker_array = MarkerArray()

            for i, (x, y) in enumerate(self.prev_features[:20]):  # Limit for performance
                marker = Marker()
                marker.header.frame_id = 'humanoid_camera_optical_frame'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'features'
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x / 100.0  # Scale down for visualization
                marker.pose.position.y = y / 100.0
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker_array.markers.append(marker)

            self.feature_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    perception_node = IntegratedPerception()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2: Create the SLAM Node
Create `integrated_slam.py` in the `docs/module3/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Integrated SLAM Node for Humanoid Robot
Combines visual and inertial data for mapping and localization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import MarkerArray, Marker
from tf2_msgs.msg import TFMessage
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
from collections import deque


class IntegratedSLAM(Node):
    def __init__(self):
        super().__init__('integrated_slam')

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'slam/map', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'slam/pose', 10)
        self.path_pub = self.create_publisher(MarkerArray, 'slam/trajectory', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        # SLAM state
        self.robot_pose = PoseStamped()
        self.robot_pose.pose.position.x = 0.0
        self.robot_pose.pose.position.y = 0.0
        self.robot_pose.pose.position.z = 0.0
        self.robot_pose.pose.orientation.w = 1.0

        # Map representation
        self.map_resolution = 0.1  # meters per cell
        self.map_width = 200  # cells (20m x 20m)
        self.map_height = 200
        self.map_origin_x = -10.0  # meters
        self.map_origin_y = -10.0
        self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Trajectory storage
        self.trajectory = deque(maxlen=1000)  # Keep last 1000 poses

        # IMU integration
        self.imu_data = None
        self.prev_imu_time = None

        # Processing timer
        self.timer = self.create_timer(0.1, self.process_callback)  # 10 Hz

        self.get_logger().info('Integrated SLAM Node Started')

    def image_callback(self, msg):
        """Process visual data for SLAM"""
        # In a real implementation, this would perform visual SLAM
        # For this demo, we'll simulate pose estimation
        self.estimate_visual_motion()

    def imu_callback(self, msg):
        """Process IMU data for drift correction"""
        self.imu_data = {
            'linear_acceleration': (msg.linear_acceleration.x,
                                    msg.linear_acceleration.y,
                                    msg.linear_acceleration.z),
            'angular_velocity': (msg.angular_velocity.x,
                                 msg.angular_velocity.y,
                                 msg.angular_velocity.z),
            'orientation': (msg.orientation.x, msg.orientation.y,
                            msg.orientation.z, msg.orientation.w),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

        # Integrate IMU data for pose correction
        if self.prev_imu_time is not None:
            dt = self.imu_data['timestamp'] - self.prev_imu_time
            self.integrate_imu_data(dt)

        self.prev_imu_time = self.imu_data['timestamp']

    def estimate_visual_motion(self):
        """Simulate visual motion estimation (in real system, use feature tracking)"""
        # For demonstration, simulate forward motion
        # In a real system, this would use visual odometry
        motion_noise = np.random.normal(0, 0.01, 2)  # Add small noise
        self.robot_pose.pose.position.x += 0.02 + motion_noise[0]
        self.robot_pose.pose.position.y += 0.005 + motion_noise[1]

        # Update orientation based on movement direction
        angle = math.atan2(
            self.robot_pose.pose.position.y - (self.robot_pose.pose.position.y - 0.005),
            self.robot_pose.pose.position.x - (self.robot_pose.pose.position.x - 0.02)
        )
        self.robot_pose.pose.orientation.w = math.cos(angle/2)
        self.robot_pose.pose.orientation.z = math.sin(angle/2)

    def integrate_imu_data(self, dt):
        """Integrate IMU data for pose correction"""
        if self.imu_data:
            # Simple integration (in real system, use more sophisticated methods)
            # This is a basic example - real IMU integration is more complex
            pass

    def update_map(self):
        """Update occupancy grid based on current pose and sensor data"""
        # Convert robot position to map coordinates
        map_x = int((self.robot_pose.pose.position.x - self.map_origin_x) / self.map_resolution)
        map_y = int((self.robot_pose.pose.position.y - self.map_origin_y) / self.map_resolution)

        # Mark current position as free space
        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            # In a real system, this would integrate sensor data
            # For demo, we'll just mark the immediate area as free
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    x = map_x + dx
                    y = map_y + dy
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        distance = math.sqrt(dx*dx + dy*dy)
                        if distance <= 2:  # Mark within 20cm (2 cells) as free
                            self.occupancy_map[y, x] = 0  # Free space

    def process_callback(self):
        """Main SLAM processing loop"""
        # Update map with latest information
        self.update_map()

        # Update trajectory
        current_time = self.get_clock().now()
        self.robot_pose.header.stamp = current_time.to_msg()
        self.robot_pose.header.frame_id = 'map'
        self.trajectory.append((current_time.to_msg(),
                               (self.robot_pose.pose.position.x,
                                self.robot_pose.pose.position.y)))

        # Publish current pose
        self.pose_pub.publish(self.robot_pose)

        # Publish map
        self.publish_map()

        # Publish trajectory visualization
        self.publish_trajectory()

        # Broadcast transform
        self.broadcast_transform()

    def publish_map(self):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        map_msg.info = MapMetaData()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatten the map array
        map_msg.data = self.occupancy_map.flatten().tolist()

        self.map_pub.publish(map_msg)

    def publish_trajectory(self):
        """Publish trajectory visualization"""
        marker_array = MarkerArray()

        # Create line strip for trajectory
        if len(self.trajectory) > 1:
            trajectory_marker = Marker()
            trajectory_marker.header.frame_id = 'map'
            trajectory_marker.header.stamp = self.get_clock().now().to_msg()
            trajectory_marker.ns = 'trajectory'
            trajectory_marker.id = 0
            trajectory_marker.type = Marker.LINE_STRIP
            trajectory_marker.action = Marker.ADD
            trajectory_marker.pose.orientation.w = 1.0
            trajectory_marker.scale.x = 0.05
            trajectory_marker.color.r = 0.0
            trajectory_marker.color.g = 1.0
            trajectory_marker.color.b = 0.0
            trajectory_marker.color.a = 0.8

            for _, (x, y) in self.trajectory:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                trajectory_marker.points.append(point)

            marker_array.markers.append(trajectory_marker)

        self.path_pub.publish(marker_array)

    def broadcast_transform(self):
        """Broadcast robot's transform in the map frame"""
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'humanoid_base_link'

        t.transform.translation.x = self.robot_pose.pose.position.x
        t.transform.translation.y = self.robot_pose.pose.position.y
        t.transform.translation.z = self.robot_pose.pose.position.z
        t.transform.rotation.x = self.robot_pose.pose.orientation.x
        t.transform.rotation.y = self.robot_pose.pose.orientation.y
        t.transform.rotation.z = self.robot_pose.pose.orientation.z
        t.transform.rotation.w = self.robot_pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    slam_node = IntegratedSLAM()

    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create the Navigation Node
Create `integrated_navigation.py` in the `docs/module3/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Integrated Navigation Node for Humanoid Robot
Implements Nav2-like functionality for humanoid navigation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
import numpy as np
import math
from typing import List, Tuple
from dataclasses import dataclass


@dataclass
class Waypoint:
    x: float
    y: float
    theta: float = 0.0


class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'humanoid/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'navigation/local_plan', 10)
        self.status_pub = self.create_publisher(String, 'navigation/status', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'humanoid/goal',
            self.goal_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            'slam/pose',
            self.pose_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'slam/map',
            self.map_callback,
            10
        )

        # Navigation state
        self.current_pose = None
        self.goal_pose = None
        self.navigation_active = False
        self.local_path = []
        self.current_waypoint_idx = 0
        self.occupancy_map = None
        self.map_info = None

        # Humanoid-specific parameters
        self.step_size = 0.1  # meters per step
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 0.5  # rad/s
        self.arrival_threshold = 0.3  # meters to goal
        self.waypoint_threshold = 0.2  # meters to waypoint

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Integrated Navigation Node Started')

    def goal_callback(self, msg):
        """Handle new navigation goal"""
        self.goal_pose = msg
        self.navigation_active = True
        self.current_waypoint_idx = 0
        self.local_path = []

        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Plan path to goal
        if self.occupancy_map is not None:
            self.plan_local_path()
        else:
            self.get_logger().warn('No map available for path planning')

    def pose_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg

    def map_callback(self, msg):
        """Update occupancy map"""
        self.occupancy_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def plan_local_path(self):
        """Plan a local path to the goal"""
        if self.current_pose is None or self.goal_pose is None:
            return

        # Simple path planning - direct line with basic obstacle avoidance
        start = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        goal = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)

        # Generate path points
        path_points = self.generate_path_points(start, goal)

        # Convert to Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(path_points):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0

            # Calculate orientation toward next point
            if i < len(path_points) - 1:
                next_x, next_y = path_points[i + 1]
                angle = math.atan2(next_y - y, next_x - x)
                pose_stamped.pose.orientation.w = math.cos(angle/2)
                pose_stamped.pose.orientation.z = math.sin(angle/2)
            else:
                # Use final orientation
                pose_stamped.pose.orientation = self.goal_pose.pose.orientation

            path_msg.poses.append(pose_stamped)

        self.local_path = path_points
        self.current_waypoint_idx = 0

        # Publish path
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Local path planned with {len(path_points)} points')

    def generate_path_points(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Generate path points from start to goal with basic obstacle consideration"""
        path = []

        # Calculate straight-line path
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Already at goal
            return [start]

        # Number of steps based on step size
        num_steps = max(2, int(distance / self.step_size))

        for i in range(num_steps + 1):
            t = i / num_steps
            x = start[0] + t * dx
            y = start[1] + t * dy

            # Check if this point is in free space (simplified)
            if self.is_free_space(x, y):
                path.append((x, y))
            else:
                # If blocked, try to find an alternative route
                # For this demo, we'll just skip blocked points
                continue

        # Ensure we end at the goal
        if path and math.sqrt((path[-1][0] - goal[0])**2 + (path[-1][1] - goal[1])**2) > 0.1:
            path.append(goal)

        return path if path else [start, goal]

    def is_free_space(self, x: float, y: float) -> bool:
        """Check if a point in the map is free space"""
        if self.occupancy_map is None or self.map_info is None:
            return True  # Assume free if no map available

        # Convert world coordinates to map coordinates
        map_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        # Check bounds
        if (0 <= map_x < self.map_info.width and
            0 <= map_y < self.map_info.height):
            # Check occupancy value (0 = free, >60 = occupied, 255 = unknown)
            return self.occupancy_map[map_y, map_x] < 60
        else:
            return False  # Out of bounds considered occupied

    def control_loop(self):
        """Main navigation control loop"""
        if not self.navigation_active or self.current_pose is None:
            # Stop the robot if not navigating
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return

        if not self.local_path:
            # Plan path if none exists
            if self.occupancy_map is not None:
                self.plan_local_path()
            return

        # Check if reached goal
        if self.goal_pose:
            goal_dist = math.sqrt(
                (self.current_pose.pose.position.x - self.goal_pose.pose.position.x)**2 +
                (self.current_pose.pose.position.y - self.goal_pose.pose.position.y)**2
            )

            if goal_dist < self.arrival_threshold:
                self.navigation_active = False
                self.get_logger().info('Goal reached successfully!')

                status_msg = String()
                status_msg.data = 'GOAL_REACHED'
                self.status_pub.publish(status_msg)

                # Stop the robot
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
                return

        # Get next waypoint
        if self.current_waypoint_idx < len(self.local_path):
            target_x, target_y = self.local_path[self.current_waypoint_idx]

            # Calculate distance to waypoint
            dist_to_waypoint = math.sqrt(
                (self.current_pose.pose.position.x - target_x)**2 +
                (self.current_pose.pose.position.y - target_y)**2
            )

            # Move to next waypoint if close enough
            if dist_to_waypoint < self.waypoint_threshold:
                self.current_waypoint_idx += 1

                if self.current_waypoint_idx >= len(self.local_path):
                    # Reached end of path, replan or finish
                    self.get_logger().info('Reached end of path, replanning...')
                    self.plan_local_path()
                    return

            # Calculate control commands
            cmd_vel = self.calculate_control_command(target_x, target_y)
            self.cmd_vel_pub.publish(cmd_vel)

    def calculate_control_command(self, target_x: float, target_y: float) -> Twist:
        """Calculate velocity commands to move toward target"""
        cmd = Twist()

        if self.current_pose is None:
            return cmd

        # Calculate direction to target
        dx = target_x - self.current_pose.pose.position.x
        dy = target_y - self.current_pose.pose.position.y

        # Calculate distance and angle
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)

        # Current orientation from quaternion
        current_w = self.current_pose.pose.orientation.w
        current_z = self.current_pose.pose.orientation.z
        current_angle = math.atan2(2 * current_w * current_z, current_w*current_w - current_z*current_z)

        # Calculate angle difference
        angle_diff = target_angle - current_angle
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # PID-like control for rotation
        angular_kp = 1.0
        cmd.angular.z = max(-self.max_angular_speed,
                           min(self.max_angular_speed,
                               angular_kp * angle_diff))

        # Move forward if approximately aligned with target
        if abs(angle_diff) < 0.5:  # 0.5 rad = ~29 degrees
            cmd.linear.x = min(self.max_linear_speed, distance * 0.5)  # Scale with distance
        else:
            cmd.linear.x = 0.0  # Don't move forward while turning significantly

        return cmd


def main(args=None):
    rclpy.init(args=args)
    nav_node = IntegratedNavigation()

    print("Integrated Navigation Node for Humanoid Robot")
    print("=" * 50)
    print("This node implements Nav2-like functionality adapted for humanoid robots.")
    print("It includes path planning, obstacle avoidance, and humanoid-specific movement.")
    print("")

    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_cmd = Twist()
        nav_node.cmd_vel_pub.publish(stop_cmd)
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Create a Synthetic Data Generator
Create `synthetic_data_generator.py` in the `docs/module3/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Synthetic Data Generator for Humanoid Robot Perception
Generates synthetic sensor data for training AI models.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Header
import random
import math


class SyntheticDataGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_data_generator')

        # Publishers
        self.image_pub = self.create_publisher(Image, 'synthetic/camera/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'synthetic/imu/data', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'synthetic/camera/camera_info', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Camera parameters
        self.image_width = 640
        self.image_height = 480
        self.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ])

        # Scene parameters for domain randomization
        self.scene_params = {
            'lighting': random.uniform(0.3, 1.0),
            'object_count': random.randint(3, 8),
            'texture_complexity': random.uniform(0.1, 1.0),
            'clutter_level': random.uniform(0.0, 0.8)
        }

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_vx = 0.1  # linear velocity
        self.robot_vtheta = 0.0  # angular velocity

        # Timer for data generation
        self.timer = self.create_timer(0.1, self.generate_data)  # 10 Hz

        self.get_logger().info('Synthetic Data Generator Started')

    def generate_scene(self):
        """Generate a synthetic scene with randomized parameters"""
        # Create base image
        image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)

        # Add floor pattern
        for y in range(0, self.image_height, 40):
            for x in range(0, self.image_width, 40):
                color = [random.randint(100, 150), random.randint(100, 150), random.randint(100, 150)]
                cv2.rectangle(image, (x, y), (x+20, y+20), color, -1)

        # Add walls
        wall_color = [random.randint(150, 200), random.randint(150, 200), random.randint(150, 200)]
        cv2.rectangle(image, (0, 0), (self.image_width, 30), wall_color, -1)  # Top wall
        cv2.rectangle(image, (0, self.image_height-30), (self.image_width, self.image_height), wall_color, -1)  # Bottom wall
        cv2.rectangle(image, (0, 0), (30, self.image_height), wall_color, -1)  # Left wall
        cv2.rectangle(image, (self.image_width-30, 0), (self.image_width, self.image_height), wall_color, -1)  # Right wall

        # Add random objects
        for _ in range(int(self.scene_params['object_count'])):
            obj_x = random.randint(50, self.image_width-50)
            obj_y = random.randint(50, self.image_height-50)
            obj_size = random.randint(20, 60)
            obj_color = [random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)]

            shape = random.choice(['circle', 'rectangle', 'triangle'])
            if shape == 'circle':
                cv2.circle(image, (obj_x, obj_y), obj_size//2, obj_color, -1)
            elif shape == 'rectangle':
                cv2.rectangle(image, (obj_x-obj_size//2, obj_y-obj_size//2),
                             (obj_x+obj_size//2, obj_y+obj_size//2), obj_color, -1)
            else:  # triangle
                pts = np.array([
                    [obj_x, obj_y-obj_size//2],
                    [obj_x-obj_size//2, obj_y+obj_size//2],
                    [obj_x+obj_size//2, obj_y+obj_size//2]
                ], np.int32)
                cv2.fillPoly(image, [pts], obj_color)

        # Add texture based on complexity parameter
        if self.scene_params['texture_complexity'] > 0.5:
            # Add random noise for texture
            noise = np.random.normal(0, self.scene_params['texture_complexity'] * 30,
                                   (self.image_height, self.image_width, 3))
            image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        # Adjust lighting
        image = (image * self.scene_params['lighting']).astype(np.uint8)

        # Add motion blur based on robot movement
        if abs(self.robot_vx) > 0.05 or abs(self.robot_vtheta) > 0.05:
            # Simulate motion blur
            kernel_size = int(max(2, abs(self.robot_vx) * 10))
            if kernel_size > 1:
                kernel = np.zeros((kernel_size, kernel_size))
                kernel[int((kernel_size-1)/2), :] = np.ones(kernel_size)
                kernel = kernel / kernel_size
                image = cv2.filter2D(image, -1, kernel)

        return image

    def generate_imu_data(self):
        """Generate synthetic IMU data"""
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'humanoid_imu_link'

        # Simulate IMU readings with realistic noise
        # In a moving robot scenario
        imu_msg.linear_acceleration.x = random.gauss(0, 0.1)  # m/s^2
        imu_msg.linear_acceleration.y = random.gauss(9.81, 0.2)  # gravity in y (side acceleration)
        imu_msg.linear_acceleration.z = random.gauss(0, 0.1)  # m/s^2

        # Angular velocity (if turning)
        imu_msg.angular_velocity.x = random.gauss(0, 0.05)  # rad/s
        imu_msg.angular_velocity.y = random.gauss(0, 0.05)  # rad/s
        imu_msg.angular_velocity.z = self.robot_vtheta + random.gauss(0, 0.02)  # rad/s

        # Orientation (integrate angular velocity)
        self.robot_theta += self.robot_vtheta * 0.1  # dt = 0.1s
        # Convert to quaternion
        cy = math.cos(self.robot_theta * 0.5)
        sy = math.sin(self.robot_theta * 0.5)
        imu_msg.orientation.w = cy
        imu_msg.orientation.z = sy
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0

        # Add noise to orientation
        imu_msg.orientation_covariance[0] = 0.01  # x
        imu_msg.orientation_covariance[4] = 0.01  # y
        imu_msg.orientation_covariance[8] = 0.01  # z

        return imu_msg

    def generate_camera_info(self):
        """Generate camera info message"""
        info_msg = CameraInfo()
        info_msg.header.frame_id = 'humanoid_camera_optical_frame'
        info_msg.width = self.image_width
        info_msg.height = self.image_height
        info_msg.k = self.camera_matrix.flatten().tolist()
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Identity rotation
        info_msg.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix

        return info_msg

    def generate_data(self):
        """Generate and publish synthetic sensor data"""
        # Update robot position
        self.robot_x += self.robot_vx * 0.1 * math.cos(self.robot_theta)  # dt = 0.1s
        self.robot_y += self.robot_vx * 0.1 * math.sin(self.robot_theta)

        # Occasionally change direction to create more interesting data
        if random.random() < 0.02:  # 2% chance per iteration
            self.robot_vtheta = random.uniform(-0.3, 0.3)

        # Generate synthetic scene
        synthetic_image = self.generate_scene()

        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(synthetic_image, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'humanoid_camera_optical_frame'

        # Generate IMU data
        imu_msg = self.generate_imu_data()

        # Generate camera info
        camera_info_msg = self.generate_camera_info()

        # Publish data
        self.image_pub.publish(image_msg)
        self.imu_pub.publish(imu_msg)
        self.camera_info_pub.publish(camera_info_msg)

        # Occasionally change scene parameters for domain randomization
        if random.random() < 0.05:  # 5% chance per iteration
            self.scene_params = {
                'lighting': random.uniform(0.3, 1.0),
                'object_count': random.randint(3, 8),
                'texture_complexity': random.uniform(0.1, 1.0),
                'clutter_level': random.uniform(0.0, 0.8)
            }

        self.get_logger().info(f'Synthetic data generated - Pos: ({self.robot_x:.2f}, {self.robot_y:.2f}), Theta: {self.robot_theta:.2f}', throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)
    data_gen = SyntheticDataGenerator()

    print("Synthetic Data Generator for Humanoid Robot Perception")
    print("=" * 55)
    print("This node generates synthetic sensor data for training AI models.")
    print("It includes domain randomization to improve real-world transfer.")
    print("")

    try:
        rclpy.spin(data_gen)
    except KeyboardInterrupt:
        pass
    finally:
        data_gen.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Testing Your Implementation

1. Start the synthetic data generator: `ros2 run your_package synthetic_data_generator`
2. Start the perception node: `ros2 run your_package integrated_perception`
3. Start the SLAM node: `ros2 run your_package integrated_slam`
4. Start the navigation node: `ros2 run your_package integrated_navigation`
5. Send a navigation goal to test the integrated system

## Extension Challenges

1. Add deep learning components for more sophisticated perception
2. Implement more advanced SLAM algorithms with loop closure
3. Add semantic mapping capabilities
4. Integrate with real robot hardware

## What You've Learned

This mini-project integrated all the concepts from Module 3:
- Combined perception, mapping, and navigation into a cohesive system
- Implemented SLAM with visual and inertial data
- Created Nav2-like functionality for humanoid robots
- Developed synthetic data generation for AI training
- Built an integrated AI-Robot Brain system

These skills form the foundation for creating intelligent humanoid robots capable of autonomous operation in complex environments.