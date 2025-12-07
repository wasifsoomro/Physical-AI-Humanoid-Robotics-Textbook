#!/usr/bin/env python3
"""
Visual SLAM Example for Humanoid Robot
This example demonstrates the basic concepts of Visual SLAM using simulated data.
Note: This is a conceptual example for educational purposes. A real implementation
would require Isaac ROS packages and appropriate hardware.
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from dataclasses import dataclass


@dataclass
class CameraPose:
    """Represents a camera pose with position and orientation"""
    position: np.ndarray  # 3D position [x, y, z]
    rotation: np.ndarray  # Rotation matrix 3x3
    timestamp: float


@dataclass
class Feature:
    """Represents a visual feature with 2D image coordinates and 3D position"""
    id: int
    pixel_coords: Tuple[float, float]  # 2D coordinates in image
    world_coords: Optional[np.ndarray] = None  # 3D coordinates in world space


class SimpleVisualSLAM:
    """
    A simplified Visual SLAM implementation for educational purposes.
    This demonstrates the core concepts without the complexity of real-world optimizations.
    """

    def __init__(self):
        self.keyframes = []  # List of CameraPose
        self.map_points = {}  # Dictionary of 3D points
        self.feature_tracks = {}  # Feature tracks across frames
        self.current_frame_id = 0
        self.intrinsic_matrix = np.array([[500, 0, 320],  # Camera intrinsic matrix
                                          [0, 500, 240],
                                          [0, 0, 1]])

    def detect_features(self, image: np.ndarray) -> List[Feature]:
        """
        Detect visual features in the image using ORB.
        In a real implementation, this might use Isaac ROS visual processing.
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Use ORB for feature detection (simplified)
        orb = cv2.ORB_create(nfeatures=500)
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        features = []
        for i, kp in enumerate(keypoints):
            feature = Feature(
                id=self.current_frame_id * 1000 + i,  # Unique ID
                pixel_coords=(kp.pt[0], kp.pt[1])
            )
            features.append(feature)

        return features

    def estimate_motion(self, features_prev: List[Feature],
                       features_curr: List[Feature]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate relative camera motion between two frames.
        This is a simplified version using essential matrix estimation.
        """
        # Find matching features between frames
        matched_prev = []
        matched_curr = []

        # Simplified matching based on proximity
        for prev_feat in features_prev:
            for curr_feat in features_curr:
                # Simple distance check for demonstration
                dist = np.sqrt((prev_feat.pixel_coords[0] - curr_feat.pixel_coords[0])**2 +
                              (prev_feat.pixel_coords[1] - curr_feat.pixel_coords[1])**2)
                if dist < 50:  # Threshold for matching
                    matched_prev.append(prev_feat.pixel_coords)
                    matched_curr.append(curr_feat.pixel_coords)
                    break

        if len(matched_prev) < 8:  # Need at least 8 points for essential matrix
            return np.eye(3), np.zeros(3)  # No motion

        # Convert to numpy arrays
        prev_points = np.array(matched_prev, dtype=np.float32)
        curr_points = np.array(matched_curr, dtype=np.float32)

        # Estimate essential matrix
        essential_matrix, mask = cv2.findEssentialMat(
            curr_points, prev_points, self.intrinsic_matrix,
            method=cv2.RANSAC, threshold=1.0
        )

        if essential_matrix is not None and essential_matrix.shape[0] >= 3:
            # Decompose essential matrix to get rotation and translation
            _, rotation, translation, _ = cv2.recoverPose(
                essential_matrix, curr_points, prev_points, self.intrinsic_matrix
            )
            return rotation, translation.flatten()
        else:
            return np.eye(3), np.zeros(3)

    def triangulate_points(self, features_prev: List[Feature],
                          features_curr: List[Feature],
                          pose_prev: CameraPose,
                          pose_curr: CameraPose) -> List[np.ndarray]:
        """
        Triangulate 3D points from matched 2D features and camera poses.
        """
        matched_prev = []
        matched_curr = []
        valid_indices = []

        # Match features again for triangulation
        for i, prev_feat in enumerate(features_prev):
            for j, curr_feat in enumerate(features_curr):
                dist = np.sqrt((prev_feat.pixel_coords[0] - curr_feat.pixel_coords[0])**2 +
                              (prev_feat.pixel_coords[1] - curr_feat.pixel_coords[1])**2)
                if dist < 50:
                    matched_prev.append(prev_feat.pixel_coords)
                    matched_curr.append(curr_feat.pixel_coords)
                    valid_indices.append((i, j))
                    break

        if len(matched_prev) < 2:
            return []

        # Convert to homogeneous coordinates
        prev_points = np.array(matched_prev, dtype=np.float32)
        curr_points = np.array(matched_curr, dtype=np.float32)

        # Create projection matrices
        proj_matrix_prev = self.intrinsic_matrix @ np.hstack([pose_prev.rotation, pose_prev.position.reshape(3, 1)])
        proj_matrix_curr = self.intrinsic_matrix @ np.hstack([pose_curr.rotation, pose_curr.position.reshape(3, 1)])

        # Triangulate points
        points_4d = cv2.triangulatePoints(
            proj_matrix_prev, proj_matrix_curr,
            prev_points.T, curr_points.T
        )

        # Convert from homogeneous to 3D
        points_3d = (points_4d[:3] / points_4d[3]).T

        return points_3d

    def process_frame(self, image: np.ndarray) -> CameraPose:
        """
        Process a single frame and update the SLAM state.
        """
        # Detect features in current frame
        current_features = self.detect_features(image)

        # If this is the first frame, create initial pose
        if len(self.keyframes) == 0:
            initial_pose = CameraPose(
                position=np.zeros(3),
                rotation=np.eye(3),
                timestamp=self.current_frame_id
            )
            self.keyframes.append(initial_pose)
            self.current_frame_id += 1
            return initial_pose

        # Get the previous frame's features
        prev_features = getattr(self, 'previous_features', [])

        if prev_features:
            # Estimate motion
            rotation, translation = self.estimate_motion(prev_features, current_features)

            # Get previous pose
            prev_pose = self.keyframes[-1]

            # Update current pose based on estimated motion
            new_position = prev_pose.position + translation
            new_rotation = rotation @ prev_pose.rotation

            current_pose = CameraPose(
                position=new_position,
                rotation=new_rotation,
                timestamp=self.current_frame_id
            )

            # Add to keyframes if movement is significant
            position_diff = np.linalg.norm(current_pose.position - prev_pose.position)
            if position_diff > 0.1:  # Add keyframe if moved more than 0.1m
                self.keyframes.append(current_pose)

                # Triangulate new map points
                new_points = self.triangulate_points(prev_features, current_features,
                                                   prev_pose, current_pose)

                # Add new points to map (simplified)
                for i, point in enumerate(new_points):
                    if np.all(np.isfinite(point)):  # Check for valid triangulation
                        point_id = f"point_{len(self.map_points)}"
                        self.map_points[point_id] = point

        # Store current features for next iteration
        self.previous_features = current_features
        self.current_frame_id += 1

        # Return the current estimated pose
        if len(self.keyframes) > 0:
            return self.keyframes[-1]
        else:
            return CameraPose(np.zeros(3), np.eye(3), self.current_frame_id - 1)


def create_simulated_environment():
    """
    Create a simple simulated environment for demonstration.
    """
    # Create a simple scene with distinctive features
    height, width = 480, 640
    scene = np.zeros((height, width, 3), dtype=np.uint8)

    # Add some distinctive patterns that can be detected as features
    cv2.rectangle(scene, (50, 50), (150, 150), (255, 0, 0), -1)  # Blue square
    cv2.circle(scene, (300, 200), 40, (0, 255, 0), -1)           # Green circle
    cv2.rectangle(scene, (400, 300), (550, 400), (0, 0, 255), -1)  # Red square

    # Add some texture/patterns
    for i in range(0, width, 50):
        cv2.line(scene, (i, 0), (i, height), (255, 255, 255), 1)
    for i in range(0, height, 50):
        cv2.line(scene, (0, i), (width, i), (255, 255, 255), 1)

    return scene


def simulate_robot_movement():
    """
    Simulate robot moving through the environment.
    """
    base_scene = create_simulated_environment()
    frames = []

    # Simulate the robot moving in a square pattern
    positions = [
        (0, 0), (1, 0), (1, 1), (0, 1),  # Square path
        (0, 0), (-1, 0), (-1, -1), (0, -1)  # Return path
    ]

    for i, (dx, dy) in enumerate(positions):
        # Create a shifted version of the scene to simulate movement
        M = np.float32([[1, 0, dx * 50], [0, 1, dy * 50]])  # Move by 50 pixels per step
        shifted_scene = cv2.warpAffine(base_scene, M, (base_scene.shape[1], base_scene.shape[0]))

        # Add some noise to make it more realistic
        noise = np.random.normal(0, 5, shifted_scene.shape).astype(np.int8)
        noisy_scene = np.clip(shifted_scene.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        frames.append(noisy_scene)

    return frames


def visualize_slam_results(slam: SimpleVisualSLAM):
    """
    Visualize the SLAM results.
    """
    if not slam.keyframes:
        print("No keyframes to visualize")
        return

    # Extract positions for plotting
    positions = np.array([kf.position for kf in slam.keyframes])

    fig = plt.figure(figsize=(12, 5))

    # Plot 1: Robot trajectory
    plt.subplot(1, 2, 1)
    if len(positions) > 0:
        plt.plot(positions[:, 0], positions[:, 1], 'b-o', label='Robot Path', markersize=6)
        plt.scatter(positions[0, 0], positions[0, 1], color='green', s=100, label='Start', zorder=5)
        plt.scatter(positions[-1, 0], positions[-1, 1], color='red', s=100, label='End', zorder=5)

    # Plot map points
    if slam.map_points:
        map_points = np.array(list(slam.map_points.values()))
        if len(map_points) > 0:
            plt.scatter(map_points[:, 0], map_points[:, 1], c='orange', s=20, alpha=0.7, label='Map Points')

    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('SLAM: Robot Trajectory and Map')
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Plot 2: Number of keyframes over time
    plt.subplot(1, 2, 2)
    frame_numbers = range(len(slam.keyframes))
    plt.plot(frame_numbers, [kf.timestamp for kf in slam.keyframes], 'g-o', markersize=4)
    plt.xlabel('Keyframe Index')
    plt.ylabel('Timestamp')
    plt.title('Keyframe Generation Over Time')
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    """
    Main function to demonstrate Visual SLAM.
    """
    print("Visual SLAM Example for Humanoid Robot")
    print("=" * 40)

    # Initialize SLAM system
    slam = SimpleVisualSLAM()

    # Generate simulated frames
    print("Generating simulated environment and robot movement...")
    frames = simulate_robot_movement()

    print(f"Generated {len(frames)} frames to process")

    # Process each frame
    print("Processing frames with Visual SLAM...")
    for i, frame in enumerate(frames):
        print(f"Processing frame {i+1}/{len(frames)}")

        # Process the frame
        current_pose = slam.process_frame(frame)

        # Print current position
        print(f"  Estimated position: [{current_pose.position[0]:.2f}, {current_pose.position[1]:.2f}, {current_pose.position[2]:.2f}]")
        print(f"  Number of keyframes: {len(slam.keyframes)}")
        print(f"  Number of map points: {len(slam.map_points)}")

    print("\nSLAM processing complete!")
    print(f"Final map contains {len(slam.keyframes)} keyframes and {len(slam.map_points)} map points")

    # Visualize results
    print("Visualizing results...")
    visualize_slam_results(slam)

    print("\nThis example demonstrates the basic concepts of Visual SLAM:")
    print("- Feature detection in images")
    print("- Estimating camera motion between frames")
    print("- Creating a map of the environment")
    print("- Tracking the robot's position in the map")
    print("\nIn a real implementation with Isaac ROS, this would be accelerated using GPU computing and include more sophisticated algorithms.")


if __name__ == "__main__":
    main()