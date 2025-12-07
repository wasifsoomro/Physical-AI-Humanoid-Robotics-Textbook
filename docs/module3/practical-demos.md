---
sidebar_position: 8
---

# Module 3 Practical Demos: AI-Robot Brain

## Demo 1: Isaac ROS Visual SLAM Integration

This demo shows how to integrate Isaac ROS Visual SLAM with a simulated humanoid robot.

### Isaac ROS Visual SLAM Node (`docs/module3/demos/isaac_vslam_demo.py`):
```python
#!/usr/bin/env python3
"""
Isaac ROS Visual SLAM Integration Demo
Demonstrates how to use Isaac ROS Visual SLAM packages with humanoid robots.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped


class IsaacVSLAMDemo(Node):
    def __init__(self):
        super().__init__('isaac_vslam_demo')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'visual_slam/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, 'visual_slam/odometry', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Internal state
        self.camera_info = None
        self.prev_image = None
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.has_initialized = False

        # Timer for processing
        self.timer = self.create_timer(0.05, self.process_callback)  # 20 Hz

        self.get_logger().info('Isaac ROS Visual SLAM Demo Started')

    def camera_info_callback(self, msg):
        """Store camera calibration information"""
        self.camera_info = msg

    def image_callback(self, msg):
        """Process incoming camera images for SLAM"""
        try:
            # In a real Isaac ROS implementation, this would interface with
            # Isaac ROS Visual SLAM packages like:
            # - Isaac ROS Visual SLAM
            # - Isaac ROS AprilTag
            # - Isaac ROS Stereo Dense Reconstruction

            # For this demo, we'll simulate the process
            self.process_visual_slam(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image for SLAM: {str(e)}')

    def process_visual_slam(self, image_msg):
        """Simulate Isaac ROS Visual SLAM processing"""
        # This would normally interface with Isaac ROS Visual SLAM packages
        # which use GPU acceleration for real-time performance

        # For demonstration, we'll simulate pose estimation
        # In real implementation, this would use Isaac ROS optimized algorithms
        if not self.has_initialized:
            self.has_initialized = True
            self.prev_image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        else:
            # Simulate movement based on time
            current_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
            dt = current_time - self.prev_image_time

            # Simulate forward movement
            self.robot_pose[0] += 0.1 * dt  # Move forward at 0.1 m/s
            self.robot_pose[1] += 0.02 * dt * np.sin(current_time)  # Slight side movement

            # Update orientation
            self.robot_pose[2] += 0.05 * dt * np.sin(current_time * 2)  # Gentle rotation

            self.prev_image_time = current_time

    def process_callback(self):
        """Publish SLAM results"""
        if not self.has_initialized:
            return

        # Create and publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(self.robot_pose[0])
        pose_msg.pose.position.y = float(self.robot_pose[1])
        pose_msg.pose.position.z = 0.0

        # Convert orientation to quaternion
        theta = self.robot_pose[2]
        pose_msg.pose.orientation.w = np.cos(theta / 2)
        pose_msg.pose.orientation.z = np.sin(theta / 2)

        self.pose_pub.publish(pose_msg)

        # Create and publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = pose_msg.header.stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'humanoid_base_link'
        odom_msg.pose.pose = pose_msg.pose

        self.odom_pub.publish(odom_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'humanoid_base_link'
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f'SLAM Pose: ({self.robot_pose[0]:.2f}, {self.robot_pose[1]:.2f}, {self.robot_pose[2]:.2f})',
            throttle_duration_sec=1
        )


def main(args=None):
    rclpy.init(args=args)
    slam_demo = IsaacVSLAMDemo()

    print("Isaac ROS Visual SLAM Demo")
    print("=" * 30)
    print("This demo simulates Isaac ROS Visual SLAM integration.")
    print("In a real implementation, this would use Isaac ROS GPU-accelerated packages.")
    print("")

    try:
        rclpy.spin(slam_demo)
    except KeyboardInterrupt:
        pass
    finally:
        slam_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Demo 2: Isaac ROS Perception Pipeline

This demo demonstrates the Isaac ROS perception pipeline for humanoid robots.

### Isaac ROS Perception Pipeline (`docs/module3/demos/isaac_perception_demo.py`):
```python
#!/usr/bin/env python3
"""
Isaac ROS Perception Pipeline Demo
Demonstrates Isaac ROS perception components for humanoid robots.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import tf2_ros


class IsaacPerceptionDemo(Node):
    def __init__(self):
        super().__init__('isaac_perception_demo')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, 'perception/detections', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'perception/pointcloud', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        # Internal state
        self.detection_id = 0

        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_callback)  # 10 Hz

        self.get_logger().info('Isaac ROS Perception Demo Started')

    def image_callback(self, msg):
        """Process incoming camera images through Isaac ROS pipeline"""
        try:
            # In a real Isaac ROS implementation, this would connect to:
            # - Isaac ROS Image Pipeline
            # - Isaac ROS Detection Pipeline
            # - Isaac ROS Stereo Pipeline
            # - Isaac ROS Point Cloud Generation

            # For this demo, we'll simulate the perception pipeline
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_perception_pipeline(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error in perception pipeline: {str(e)}')

    def process_perception_pipeline(self, image):
        """Simulate Isaac ROS perception pipeline processing"""
        # This would normally use Isaac ROS optimized perception packages
        # such as Isaac ROS Image Pipeline, Detection Pipeline, etc.

        # For demonstration, detect simple colored objects
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red objects
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process detections
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small detections
                x, y, w, h = cv2.boundingRect(contour)
                detection = self.create_detection(x, y, w, h, 'red_object', area)
                detections.append(detection)

        # Publish detections
        if detections:
            detection_array = Detection2DArray()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = 'humanoid_camera_optical_frame'
            detection_array.detections = detections

            self.detection_pub.publish(detection_array)

    def create_detection(self, x, y, w, h, label, confidence=0.8):
        """Create a vision_msgs/Detection2D message"""
        detection = Detection2D()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = 'humanoid_camera_optical_frame'

        # Bounding box
        detection.bbox.center.x = x + w/2
        detection.bbox.center.y = y + h/2
        detection.bbox.size_x = w
        detection.bbox.size_y = h

        # Results
        result = ObjectHypothesisWithPose()
        result.id = f'{label}_{self.detection_id}'
        result.score = confidence
        self.detection_id += 1

        detection.results.append(result)

        return detection

    def process_callback(self):
        """Periodic processing"""
        # In a real implementation, this would handle continuous perception
        # and maintain perception state over time
        pass


def main(args=None):
    rclpy.init(args=args)
    perception_demo = IsaacPerceptionDemo()

    print("Isaac ROS Perception Pipeline Demo")
    print("=" * 35)
    print("This demo simulates Isaac ROS perception pipeline components.")
    print("In a real implementation, this would use Isaac ROS GPU-accelerated perception.")
    print("")

    try:
        rclpy.spin(perception_demo)
    except KeyboardInterrupt:
        pass
    finally:
        perception_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Import cv2 here to avoid import errors if not available
    try:
        import cv2
        main()
    except ImportError:
        print("OpenCV not available, skipping perception demo")
        print("Install OpenCV with: pip install opencv-python")