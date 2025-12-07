#!/usr/bin/env python3
"""
Capstone Humanoid Robot System for Vision-Language-Action Robotics
This system integrates all modules learned in the book to create a complete
VLA (Vision-Language-Action) system for humanoid robot control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from builtin_interfaces.msg import Duration

import numpy as np
import cv2
from cv_bridge import CvBridge
import json
import threading
import queue
import time
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
import requests
import base64


@dataclass
class RobotState:
    """Represents the current state of the robot"""
    position: tuple = (0.0, 0.0, 0.0)  # x, y, theta
    is_moving: bool = False
    is_manipulating: bool = False
    battery_level: float = 100.0
    current_task: str = ""
    detected_objects: List[Dict] = None


class CapstoneRobotVLA(Node):
    """
    Capstone VLA System for Humanoid Robot
    Integrates Vision, Language, and Action capabilities
    """

    def __init__(self):
        super().__init__('capstone_robot_vla')

        # Initialize components
        self.bridge = CvBridge()
        self.robot_state = RobotState()
        self.command_queue = queue.Queue()
        self.response_publisher = self.create_publisher(String, 'robot/response', 10)
        self.status_publisher = self.create_publisher(String, 'robot/status', 10)

        # Vision system components
        self.current_image = None
        self.detected_objects = []
        self.object_locations = {}

        # Language system components
        self.llm_endpoint = "http://localhost:11434/api/generate"  # Example Ollama endpoint
        self.conversation_history = []

        # Action system components
        self.navigation_active = False
        self.manipulation_active = False

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/perception/detections',
            self.detection_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/humanoid/odom',
            self.odom_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/robot/command',
            self.command_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_commands)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Threading for long-running tasks
        self.processing_thread = threading.Thread(target=self.command_processor, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Capstone VLA Robot System Initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detection_callback(self, msg):
        """Process object detections"""
        self.detected_objects = []
        for detection in msg.detections:
            obj_info = {
                'id': detection.results[0].id if detection.results else 'unknown',
                'label': detection.results[0].id if detection.results else 'unknown',
                'confidence': detection.results[0].score if detection.results else 0.0,
                'x': detection.bbox.center.x,
                'y': detection.bbox.center.y,
                'width': detection.bbox.size_x,
                'height': detection.bbox.size_y
            }
            self.detected_objects.append(obj_info)

        # Update object locations
        for obj in self.detected_objects:
            self.object_locations[obj['label']] = (obj['x'], obj['y'])

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_state.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        )

    def command_callback(self, msg):
        """Receive natural language commands"""
        command = msg.data
        self.command_queue.put(command)
        self.get_logger().info(f'Received command: {command}')

    def process_commands(self):
        """Main processing loop for commands"""
        # This method handles real-time status updates and monitoring
        pass

    def publish_status(self):
        """Publish robot status"""
        status_msg = String()
        status_msg.data = json.dumps({
            'position': self.robot_state.position,
            'is_moving': self.robot_state.is_moving,
            'is_manipulating': self.robot_state.is_manipulating,
            'battery_level': self.robot_state.battery_level,
            'current_task': self.robot_state.current_task,
            'detected_objects': [obj['label'] for obj in self.detected_objects]
        })
        self.status_publisher.publish(status_msg)

    def command_processor(self):
        """Process commands in a separate thread"""
        while rclpy.ok():
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get(timeout=0.1)
                    self.process_natural_language_command(command)
                else:
                    time.sleep(0.1)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in command processor: {str(e)}')

    def process_natural_language_command(self, command: str):
        """Process a natural language command through VLA pipeline"""
        self.get_logger().info(f'Processing command: {command}')

        # Update current task
        self.robot_state.current_task = command

        try:
            # Step 1: Language Understanding - Parse the command using LLM
            action_plan = self.parse_command_with_llm(command)

            if not action_plan:
                self.respond("I couldn't understand that command. Could you please rephrase?")
                return

            # Step 2: Vision Integration - Get current scene context
            scene_context = self.get_scene_context()

            # Step 3: Action Planning - Combine language understanding with scene context
            executable_actions = self.plan_actions(action_plan, scene_context, command)

            # Step 4: Execution - Execute the planned actions
            success = self.execute_actions(executable_actions)

            if success:
                self.respond(f"I've completed the task: {command}")
            else:
                self.respond(f"I couldn't complete the task: {command}. Would you like me to try something else?")

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            self.respond(f"Sorry, I encountered an error processing your command: {str(e)}")

    def parse_command_with_llm(self, command: str) -> Optional[Dict[str, Any]]:
        """Use a language model to parse the natural language command"""
        # For this demo, we'll simulate LLM processing
        # In a real implementation, this would call an actual LLM API

        command_lower = command.lower()

        # Simple rule-based parsing for demonstration
        # In real implementation, this would use a proper LLM
        if any(word in command_lower for word in ['bring', 'get', 'fetch', 'pick up']):
            # Extract object to fetch
            for obj in ['cup', 'book', 'phone', 'keys', 'bottle']:
                if obj in command_lower:
                    return {
                        'action_type': 'fetch_object',
                        'target_object': obj,
                        'location': 'unknown'  # Will be determined from vision
                    }

        elif any(word in command_lower for word in ['go to', 'move to', 'navigate to', 'go', 'move']):
            # Extract destination
            for location in ['kitchen', 'living room', 'bedroom', 'office', 'dining room', 'bathroom']:
                if location in command_lower:
                    return {
                        'action_type': 'navigate',
                        'destination': location
                    }

        elif any(word in command_lower for word in ['find', 'locate', 'look for']):
            # Extract object to find
            for obj in ['keys', 'phone', 'wallet', 'book', 'glasses']:
                if obj in command_lower:
                    return {
                        'action_type': 'find_object',
                        'target_object': obj
                    }

        elif any(word in command_lower for word in ['wait', 'stop', 'pause']):
            return {
                'action_type': 'wait',
                'duration': 10  # Default wait time
            }

        elif any(word in command_lower for word in ['follow', 'come', 'follow me']):
            return {
                'action_type': 'follow_human'
            }

        # If no specific action identified, ask for clarification
        return None

    def get_scene_context(self) -> Dict[str, Any]:
        """Get current scene context from vision system"""
        context = {
            'detected_objects': self.detected_objects,
            'object_locations': self.object_locations.copy(),
            'current_position': self.robot_state.position,
            'current_image_available': self.current_image is not None
        }
        return context

    def plan_actions(self, action_plan: Dict[str, Any], scene_context: Dict[str, Any], original_command: str) -> List[Dict[str, Any]]:
        """Plan executable actions based on LLM output and scene context"""
        actions = []

        action_type = action_plan['action_type']

        if action_type == 'fetch_object':
            target_object = action_plan['target_object']

            # Find the object in the scene
            object_location = None
            for obj in scene_context['detected_objects']:
                if target_object in obj['label'] or obj['label'] in target_object:
                    object_location = (obj['x'], obj['y'])
                    break

            if object_location:
                # Plan sequence: navigate to object, grasp it, return
                actions.extend([
                    {
                        'type': 'navigate_to_object',
                        'object': target_object,
                        'location': object_location
                    },
                    {
                        'type': 'grasp_object',
                        'object': target_object
                    },
                    {
                        'type': 'return_to_user'
                    }
                ])
            else:
                # Object not found, ask user for help or search
                actions.append({
                    'type': 'search_for_object',
                    'object': target_object
                })

        elif action_type == 'navigate':
            destination = action_plan['destination']
            # In a real system, this would look up destination coordinates
            # For demo, we'll just acknowledge the command
            actions.append({
                'type': 'navigate_to_location',
                'location': destination
            })

        elif action_type == 'find_object':
            target_object = action_plan['target_object']
            actions.append({
                'type': 'search_for_object',
                'object': target_object
            })

        elif action_type == 'wait':
            actions.append({
                'type': 'wait',
                'duration': action_plan.get('duration', 10)
            })

        elif action_type == 'follow_human':
            actions.append({
                'type': 'follow_human'
            })

        return actions

    def execute_actions(self, actions: List[Dict[str, Any]]) -> bool:
        """Execute the planned actions"""
        success = True

        for action in actions:
            action_type = action['type']

            if action_type == 'navigate_to_object':
                obj = action['object']
                self.get_logger().info(f'Navigating to {obj}')
                # Simulate navigation
                time.sleep(2)
                self.respond(f"Approaching the {obj}")

            elif action_type == 'grasp_object':
                obj = action['object']
                self.get_logger().info(f'Grasping {obj}')
                # Simulate grasping
                time.sleep(2)
                self.respond(f"Grasped the {obj}")

            elif action_type == 'return_to_user':
                self.get_logger().info('Returning to user')
                # Simulate returning
                time.sleep(2)
                self.respond("Returned to you with the object")

            elif action_type == 'search_for_object':
                obj = action['object']
                self.get_logger().info(f'Searching for {obj}')
                # Simulate searching
                time.sleep(3)
                self.respond(f"Finished searching for {obj}")

            elif action_type == 'navigate_to_location':
                location = action['location']
                self.get_logger().info(f'Navigating to {location}')
                # Simulate navigation
                time.sleep(3)
                self.respond(f"Arrived at the {location}")

            elif action_type == 'wait':
                duration = action['duration']
                self.get_logger().info(f'Waiting for {duration} seconds')
                time.sleep(duration)
                self.respond("Wait period completed")

            elif action_type == 'follow_human':
                self.get_logger().info('Following human')
                # Simulate following
                time.sleep(5)
                self.respond("I'm following you")

            # Add small delay between actions
            time.sleep(0.5)

        return success

    def respond(self, response: str):
        """Publish a response to the user"""
        response_msg = String()
        response_msg.data = response
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'Response: {response}')


def main(args=None):
    """Main function to run the capstone VLA robot system"""
    rclpy.init(args=args)

    vla_robot = CapstoneRobotVLA()

    print("Capstone VLA Robot System")
    print("=" * 30)
    print("This system integrates all modules from the Physical AI & Humanoid Robotics book:")
    print("- Vision: Object detection and scene understanding")
    print("- Language: Natural language command processing")
    print("- Action: Navigation and manipulation execution")
    print("")
    print("The robot can understand commands like:")
    print("- 'Please bring me the red cup'")
    print("- 'Go to the kitchen and wait'")
    print("- 'Help me find my keys'")
    print("- 'Move to the living room'")
    print("")
    print("Publish commands to /robot/command topic to control the robot")
    print("")

    try:
        rclpy.spin(vla_robot)
    except KeyboardInterrupt:
        pass
    finally:
        vla_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()