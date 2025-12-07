#!/usr/bin/env python3
"""
Nav2 Humanoid Control Demo
This example demonstrates the concepts of Nav2 for humanoid robots.
Note: This is a conceptual example for educational purposes. A real implementation
would require Nav2 packages, Isaac ROS, and appropriate robot hardware.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time
from typing import List, Tuple
import math


class Nav2HumanoidDemo(Node):
    """
    A demonstration of Nav2 concepts adapted for humanoid robots.
    This simulates the navigation stack components for educational purposes.
    """

    def __init__(self):
        super().__init__('nav2_humanoid_demo')

        # Publishers
        self.path_publisher = self.create_publisher(Path, 'humanoid/plan', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'humanoid/goal', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'humanoid/markers', 10)
        self.current_pose_publisher = self.create_publisher(PoseStamped, 'humanoid/current_pose', 10)

        # Timer for simulation
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Robot state
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.w = 1.0  # No rotation initially

        # Navigation state
        self.goal_pose = None
        self.current_path = Path()
        self.navigation_active = False
        self.path_index = 0
        self.simulation_step = 0

        # Costmap representation (simplified grid)
        self.costmap_resolution = 0.1  # meters per cell
        self.costmap_width = 200  # cells (20m x 20m area)
        self.costmap_height = 200  # cells
        self.costmap_origin_x = -10.0  # meters
        self.costmap_origin_y = -10.0  # meters
        self.costmap = self.initialize_costmap()

        self.get_logger().info('Nav2 Humanoid Control Demo Started')

    def initialize_costmap(self):
        """
        Initialize a simple costmap with some obstacles.
        """
        costmap = np.zeros((self.costmap_height, self.costmap_width), dtype=np.uint8)

        # Add some obstacles (represented as high cost values)
        # Obstacle 1: Wall
        for x in range(80, 120):
            for y in range(80, 85):
                costmap[y, x] = 254  # High cost (near obstacle)

        # Obstacle 2: Pillar
        center_x, center_y = 150, 150
        radius = 8
        for x in range(max(0, center_x - radius), min(self.costmap_width, center_x + radius)):
            for y in range(max(0, center_y - radius), min(self.costmap_height, center_y + radius)):
                dist = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                if dist <= radius:
                    costmap[y, x] = 254

        # Obstacle 3: Narrow passage
        for x in range(40, 60):
            for y in range(140, 145):
                costmap[y, x] = 254
            for y in range(155, 160):
                costmap[y, x] = 254

        return costmap

    def timer_callback(self):
        """
        Main simulation loop.
        """
        self.simulation_step += 1

        # Publish current pose
        self.current_pose_publisher.publish(self.current_pose)

        # If navigation is active, move toward goal
        if self.navigation_active and self.goal_pose and self.current_path.poses:
            self.move_toward_goal()

        # Publish visualization markers periodically
        if self.simulation_step % 10 == 0:  # Every second
            self.publish_visualization()

    def move_toward_goal(self):
        """
        Simulate robot movement toward the goal.
        """
        if not self.current_path.poses:
            return

        # Get next waypoint
        if self.path_index < len(self.current_path.poses):
            target_pose = self.current_path.poses[self.path_index].pose

            # Calculate direction to target
            dx = target_pose.position.x - self.current_pose.pose.position.x
            dy = target_pose.position.y - self.current_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            # Define humanoid-specific movement parameters
            step_size = 0.05  # meters per step (conservative for humanoid balance)
            threshold = 0.1   # distance threshold to consider waypoint reached

            if distance > threshold:
                # Move toward target
                move_x = dx / distance * step_size
                move_y = dy / distance * step_size

                self.current_pose.pose.position.x += move_x
                self.current_pose.pose.position.y += move_y

                # Update orientation to face movement direction
                angle = math.atan2(dy, dx)
                self.current_pose.pose.orientation.w = math.cos(angle/2)
                self.current_pose.pose.orientation.z = math.sin(angle/2)
            else:
                # Reached current waypoint, move to next
                self.path_index += 1

                # Check if reached the end of path
                if self.path_index >= len(self.current_path.poses):
                    goal_distance = math.sqrt(
                        (self.goal_pose.pose.position.x - self.current_pose.pose.position.x)**2 +
                        (self.goal_pose.pose.position.y - self.current_pose.pose.position.y)**2
                    )

                    if goal_distance < 0.3:  # Close enough to goal
                        self.get_logger().info('Goal reached successfully!')
                        self.navigation_active = False
                        self.current_path = Path()  # Clear path
                        self.path_index = 0
                    else:
                        # Need to replan (simplified)
                        self.get_logger().info('Replanning path...')
                        self.plan_path()

    def plan_path(self):
        """
        Plan a path from current position to goal using a simplified algorithm.
        In real Nav2, this would use A*, Dijkstra, or other sophisticated planners.
        """
        if not self.goal_pose:
            return

        # Simplified path planning using wavefront or A* like approach
        start_x = int((self.current_pose.pose.position.x - self.costmap_origin_x) / self.costmap_resolution)
        start_y = int((self.current_pose.pose.position.y - self.costmap_origin_y) / self.costmap_resolution)
        goal_x = int((self.goal_pose.pose.position.x - self.costmap_origin_x) / self.costmap_resolution)
        goal_y = int((self.goal_pose.pose.position.y - self.costmap_origin_y) / self.costmap_resolution)

        # Check bounds
        if (0 <= start_x < self.costmap_width and 0 <= start_y < self.costmap_height and
            0 <= goal_x < self.costmap_width and 0 <= goal_y < self.costmap_height):

            # Use a simplified path planning - direct line with obstacle avoidance
            path_poses = self.generate_path_with_obstacles(start_x, start_y, goal_x, goal_y)

            # Convert path to Path message
            self.current_path = Path()
            self.current_path.header.frame_id = 'map'
            self.current_path.header.stamp = self.get_clock().now().to_msg()

            for pose in path_poses:
                stamped_pose = PoseStamped()
                stamped_pose.header.frame_id = 'map'
                stamped_pose.pose.position.x = pose[0]
                stamped_pose.pose.position.y = pose[1]
                stamped_pose.pose.position.z = 0.0
                # Simple orientation toward next point
                if len(path_poses) > 1:
                    idx = path_poses.index(pose)
                    if idx < len(path_poses) - 1:
                        next_pose = path_poses[idx + 1]
                        angle = math.atan2(next_pose[1] - pose[1], next_pose[0] - pose[0])
                        stamped_pose.pose.orientation.w = math.cos(angle/2)
                        stamped_pose.pose.orientation.z = math.sin(angle/2)
                    else:
                        stamped_pose.pose.orientation.w = self.current_pose.pose.orientation.w
                        stamped_pose.pose.orientation.z = self.current_pose.pose.orientation.z

                self.current_path.poses.append(stamped_pose)

            # Publish path
            self.path_publisher.publish(self.current_path)
            self.path_index = 0
            self.get_logger().info(f'Path planned with {len(self.current_path.poses)} waypoints')

    def generate_path_with_obstacles(self, start_x, start_y, goal_x, goal_y):
        """
        Generate a path that tries to avoid obstacles.
        This is a simplified implementation for demonstration.
        """
        path = []

        # Start position in meters
        current_x = self.costmap_origin_x + start_x * self.costmap_resolution
        current_y = self.costmap_origin_y + start_y * self.costmap_resolution

        # Goal position in meters
        goal_real_x = self.costmap_origin_x + goal_x * self.costmap_resolution
        goal_real_y = self.costmap_origin_y + goal_y * self.costmap_resolution

        # Simple path following algorithm with obstacle detection
        step_size = 0.2  # meters

        while True:
            path.append((current_x, current_y))

            # Calculate direction to goal
            dx = goal_real_x - current_x
            dy = goal_real_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < step_size:
                path.append((goal_real_x, goal_real_y))
                break

            # Normalize direction
            dx /= distance
            dy /= distance

            # Check next step for obstacles
            next_x = current_x + dx * step_size
            next_y = current_y + dy * step_size

            # Convert to costmap coordinates
            next_cx = int((next_x - self.costmap_origin_x) / self.costmap_resolution)
            next_cy = int((next_y - self.costmap_origin_y) / self.costmap_resolution)

            # Check if next step is in bounds and not an obstacle
            is_safe = (0 <= next_cx < self.costmap_width and
                      0 <= next_cy < self.costmap_height and
                      self.costmap[next_cy, next_cx] < 200)  # Safe threshold

            if is_safe:
                current_x = next_x
                current_y = next_y
            else:
                # Simple obstacle avoidance: try to go around
                # In a real implementation, this would use proper local planning
                self.get_logger().info('Obstacle detected, replanning locally...')
                # For demo, just move around the obstacle in a simple way
                current_x += -dy * step_size * 0.5  # Move perpendicular
                current_y += dx * step_size * 0.5

                # Check if this new position is safe
                check_cx = int((current_x - self.costmap_origin_x) / self.costmap_resolution)
                check_cy = int((current_y - self.costmap_origin_y) / self.costmap_resolution)

                if (not (0 <= check_cx < self.costmap_width and
                        0 <= check_cy < self.costmap_height) or
                    self.costmap[check_cy, check_cx] >= 200):
                    # If perpendicular move is also blocked, go back
                    current_x = self.costmap_origin_x + start_x * self.costmap_resolution
                    current_y = self.costmap_origin_y + start_y * self.costmap_resolution
                    break

        return path

    def send_goal(self, x: float, y: float):
        """
        Send a navigation goal to the system.
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.goal_pose = goal_pose
        self.navigation_active = True

        # Publish goal for visualization
        self.goal_publisher.publish(goal_pose)

        # Plan initial path
        self.plan_path()

        self.get_logger().info(f'Navigation goal sent: ({x}, {y})')

    def publish_visualization(self):
        """
        Publish visualization markers for RViz.
        """
        marker_array = MarkerArray()

        # Create costmap visualization (simplified as occupancy grid)
        costmap_marker = Marker()
        costmap_marker.header.frame_id = 'map'
        costmap_marker.header.stamp = self.get_clock().now().to_msg()
        costmap_marker.ns = 'costmap'
        costmap_marker.id = 0
        costmap_marker.type = Marker.CUBE_LIST
        costmap_marker.action = Marker.ADD
        costmap_marker.pose.orientation.w = 1.0
        costmap_marker.scale.x = self.costmap_resolution
        costmap_marker.scale.y = self.costmap_resolution
        costmap_marker.scale.z = 0.1
        costmap_marker.frame_locked = True

        # Add points for high-cost areas (obstacles)
        for y in range(0, self.costmap_height, 5):  # Sample every 5th cell for performance
            for x in range(0, self.costmap_width, 5):
                if self.costmap[y, x] > 200:  # Obstacle threshold
                    point = Point()
                    point.x = self.costmap_origin_x + x * self.costmap_resolution
                    point.y = self.costmap_origin_y + y * self.costmap_resolution
                    point.z = 0.0
                    costmap_marker.points.append(point)

                    # Color based on cost (red for obstacles)
                    costmap_marker.colors.append(self.get_cost_color(self.costmap[y, x]))

        marker_array.markers.append(costmap_marker)

        # Visualize current path if active
        if self.current_path.poses:
            path_marker = Marker()
            path_marker.header = self.current_path.header
            path_marker.ns = 'path'
            path_marker.id = 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.pose.orientation.w = 1.0
            path_marker.scale.x = 0.05
            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 0.8

            for pose_stamped in self.current_path.poses:
                path_marker.points.append(pose_stamped.pose.position)

            marker_array.markers.append(path_marker)

        # Visualize robot
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = 'robot'
        robot_marker.id = 2
        robot_marker.type = Marker.CYLINDER
        robot_marker.action = Marker.ADD
        robot_marker.pose = self.current_pose.pose
        robot_marker.scale.x = 0.3  # Robot width
        robot_marker.scale.y = 0.3  # Robot width
        robot_marker.scale.z = 1.0  # Robot height
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0
        robot_marker.color.a = 0.8

        marker_array.markers.append(robot_marker)

        # Publish markers
        self.marker_publisher.publish(marker_array)

    def get_cost_color(self, cost):
        """
        Get color based on cost value for visualization.
        """
        from std_msgs.msg import ColorRGBA
        color = ColorRGBA()

        if cost > 200:  # Obstacle
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
            color.a = 0.8
        elif cost > 100:  # High cost area
            color.r = 1.0
            color.g = 0.5
            color.b = 0.0
            color.a = 0.6
        else:  # Free space
            color.r = 0.0
            color.g = 1.0
            color.b = 0.0
            color.a = 0.3

        return color


def main(args=None):
    """
    Main function to run the Nav2 Humanoid Control demo.
    """
    rclpy.init(args=args)

    nav2_demo = Nav2HumanoidDemo()

    # Example: Send a navigation goal after a short delay
    def send_example_goal():
        # Send goal to (5, 5) in the map
        nav2_demo.send_goal(5.0, 5.0)

    # Schedule the example goal after 2 seconds
    timer = nav2_demo.create_timer(2.0, send_example_goal)

    # Cancel the timer after first execution
    def cancel_timer():
        timer.cancel()

    nav2_demo.create_timer(2.1, cancel_timer)

    print("Nav2 Humanoid Control Demo")
    print("=" * 30)
    print("This demo simulates Nav2 concepts for humanoid robots.")
    print("The robot will navigate from (0,0) to (5,5) avoiding obstacles.")
    print("Visualization markers are published for RViz viewing.")
    print("Run RViz2 and add the appropriate displays to visualize the demo.")
    print("")

    try:
        rclpy.spin(nav2_demo)
    except KeyboardInterrupt:
        pass
    finally:
        nav2_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()