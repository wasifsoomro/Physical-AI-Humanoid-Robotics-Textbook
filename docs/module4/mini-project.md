---
sidebar_position: 7
---

# Module 4 Mini-Project: Complete VLA Humanoid Robot System

## Project Overview

In this capstone mini-project, you will create a complete Vision-Language-Action (VLA) system that integrates all the concepts learned throughout this book. This system will enable a humanoid robot to understand natural language commands, interpret visual scenes, and execute appropriate physical actions.

## Learning Objectives

By completing this project, you will:
- Integrate vision, language, and action systems into a cohesive VLA architecture
- Implement natural language processing for robot command interpretation
- Create a scene understanding system that grounds language in visual context
- Develop action planning and execution capabilities
- Build a complete humanoid robot control system

## Project Requirements

### 1. Vision System
Create a vision system that:
- Processes camera images for object detection and scene understanding
- Tracks objects and their locations in the environment
- Provides spatial reasoning capabilities
- Integrates with the language understanding system

### 2. Language Processing System
Implement a language system that:
- Processes natural language commands from users
- Interprets commands using contextual understanding
- Resolves ambiguities using visual context
- Generates action plans from language input

### 3. Action Execution System
Build an action system that:
- Plans sequences of actions based on language commands
- Executes navigation and manipulation tasks
- Handles errors and exceptions gracefully
- Provides feedback to users

### 4. Integration Framework
Develop an integration framework that:
- Coordinates between vision, language, and action components
- Manages real-time processing requirements
- Handles multi-modal input and output
- Maintains interaction context across commands

## Implementation Steps

### Step 1: Create the Main VLA Node
Create `complete_vla_system.py` in the `docs/module4/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Complete VLA System for Humanoid Robot
Integrates vision, language, and action for natural human-robot interaction.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import json
import threading
import queue
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Any
import re


@dataclass
class RobotState:
    """Represents the current state of the robot"""
    position: tuple = (0.0, 0.0, 0.0)  # x, y, theta
    is_moving: bool = False
    is_executing: bool = False
    battery_level: float = 100.0
    current_task: str = ""
    detected_objects: List[Dict] = None
    interaction_context: Dict = None


class CompleteVLASystem(Node):
    """
    Complete Vision-Language-Action system for humanoid robot control.
    Integrates all modules learned in the book into a cohesive system.
    """

    def __init__(self):
        super().__init__('complete_vla_system')

        # Initialize components
        self.bridge = CvBridge()
        self.robot_state = RobotState()
        self.robot_state.detected_objects = []
        self.robot_state.interaction_context = {}

        # Command processing
        self.command_queue = queue.Queue()
        self.response_publisher = self.create_publisher(String, 'vla/response', 10)
        self.status_publisher = self.create_publisher(String, 'vla/status', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'humanoid/cmd_vel', 10)

        # Vision system
        self.current_image = None
        self.scene_objects = {}
        self.object_locations = {}

        # Language system
        self.conversation_history = []
        self.understanding_threshold = 0.7  # Minimum confidence for command execution

        # Action system
        self.action_queue = queue.Queue()
        self.is_executing = False

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
            '/vla/command',
            self.command_callback,
            10
        )

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_callback)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Threading for long-running tasks
        self.processing_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Complete VLA System Initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detection_callback(self, msg):
        """Process object detections from perception system"""
        self.scene_objects = {}
        self.object_locations = {}

        for detection in msg.detections:
            if detection.results:
                obj_label = detection.results[0].id
                obj_confidence = detection.results[0].score

                if obj_confidence > 0.5:  # Confidence threshold
                    obj_info = {
                        'label': obj_label,
                        'confidence': obj_confidence,
                        'position': (detection.bbox.center.x, detection.bbox.center.y),
                        'size': (detection.bbox.size_x, detection.bbox.size_y)
                    }

                    self.scene_objects[obj_label] = obj_info
                    self.object_locations[obj_label] = (detection.bbox.center.x, detection.bbox.center.y)

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

    def process_callback(self):
        """Main processing callback for real-time updates"""
        # This handles ongoing monitoring and status updates
        pass

    def publish_status(self):
        """Publish robot status"""
        status_msg = String()
        status_msg.data = json.dumps({
            'position': self.robot_state.position,
            'is_moving': self.robot_state.is_moving,
            'is_executing': self.robot_state.is_executing,
            'battery_level': self.robot_state.battery_level,
            'current_task': self.robot_state.current_task,
            'detected_objects': list(self.scene_objects.keys()),
            'interaction_context': self.robot_state.interaction_context
        })
        self.status_publisher.publish(status_msg)

    def process_commands(self):
        """Process commands in a separate thread"""
        while rclpy.ok():
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get(timeout=0.1)
                    self.execute_vla_pipeline(command)
                else:
                    time.sleep(0.1)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in command processor: {str(e)}')

    def execute_vla_pipeline(self, command: str):
        """Execute the complete VLA pipeline: Vision → Language → Action"""
        self.get_logger().info(f'Executing VLA pipeline for: {command}')
        self.robot_state.current_task = command
        self.robot_state.is_executing = True

        try:
            # Step 1: Language Understanding
            parsed_command = self.parse_language_command(command)

            if not parsed_command:
                self.respond("I couldn't understand that command. Could you please rephrase?")
                return

            # Step 2: Vision Integration (Ground the command in visual context)
            grounded_command = self.ground_command_in_vision(parsed_command)

            # Step 3: Action Planning
            action_sequence = self.plan_actions(grounded_command)

            # Step 4: Action Execution
            success = self.execute_action_sequence(action_sequence)

            if success:
                self.respond(f"I've completed the task: {command}")
            else:
                self.respond(f"I couldn't complete the task: {command}")

        except Exception as e:
            self.get_logger().error(f'Error in VLA pipeline: {str(e)}')
            self.respond(f"Sorry, I encountered an error: {str(e)}")
        finally:
            self.robot_state.is_executing = False

    def parse_language_command(self, command: str) -> Optional[Dict[str, Any]]:
        """Parse natural language command using rule-based or simple NLP"""
        command_lower = command.lower().strip()

        # Define action patterns
        patterns = {
            'fetch': [r'bring.*?(\w+)', r'get.*?(\w+)', r'fetch.*?(\w+)', r'pick.*?(\w+)'],
            'navigate': [r'go to.*?(\w+)', r'move to.*?(\w+)', r'navigate to.*?(\w+)', r'go.*?(\w+)'],
            'find': [r'find.*?(\w+)', r'locate.*?(\w+)', r'look for.*?(\w+)', r'where.*?(\w+)'],
            'wait': [r'wait', r'pause', r'stop'],
            'follow': [r'follow', r'come.*?me', r'follow.*?me']
        }

        for action_type, regex_list in patterns.items():
            for pattern in regex_list:
                match = re.search(pattern, command_lower)
                if match:
                    target = match.group(1) if len(match.groups()) > 0 else None
                    return {
                        'action_type': action_type,
                        'target': target,
                        'original_command': command
                    }

        # If no pattern matches, return None
        return None

    def ground_command_in_vision(self, parsed_command: Dict[str, Any]) -> Dict[str, Any]:
        """Ground the command in visual context"""
        grounded_command = parsed_command.copy()

        if parsed_command['action_type'] in ['fetch', 'find'] and parsed_command['target']:
            target_obj = parsed_command['target']

            # Look for the target object in scene
            found_obj = None
            for obj_label, obj_info in self.scene_objects.items():
                if target_obj.lower() in obj_label.lower() or obj_label.lower() in target_obj.lower():
                    found_obj = obj_info
                    break

            if found_obj:
                grounded_command['object_info'] = found_obj
                grounded_command['object_found'] = True
            else:
                grounded_command['object_found'] = False

        elif parsed_command['action_type'] == 'navigate' and parsed_command['target']:
            # In a real system, this would look up location coordinates
            # For demo, we'll just acknowledge the location
            grounded_command['location'] = parsed_command['target']

        return grounded_command

    def plan_actions(self, grounded_command: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Plan sequence of actions based on grounded command"""
        action_sequence = []

        action_type = grounded_command['action_type']

        if action_type == 'fetch' and grounded_command.get('object_found', False):
            obj_info = grounded_command['object_info']
            action_sequence = [
                {
                    'type': 'navigate_to_object',
                    'object': obj_info['label'],
                    'position': obj_info['position']
                },
                {
                    'type': 'grasp_object',
                    'object': obj_info['label']
                },
                {
                    'type': 'return_to_user'
                }
            ]

        elif action_type == 'fetch' and not grounded_command.get('object_found', False):
            action_sequence = [
                {
                    'type': 'search_for_object',
                    'object': grounded_command['target']
                }
            ]

        elif action_type == 'navigate':
            action_sequence = [
                {
                    'type': 'navigate_to_location',
                    'location': grounded_command['location']
                }
            ]

        elif action_type == 'find':
            if grounded_command.get('object_found', False):
                action_sequence = [
                    {
                        'type': 'point_to_object',
                        'object': grounded_command['object_info']['label'],
                        'position': grounded_command['object_info']['position']
                    }
                ]
            else:
                action_sequence = [
                    {
                        'type': 'search_for_object',
                        'object': grounded_command['target']
                    }
                ]

        elif action_type == 'wait':
            action_sequence = [
                {
                    'type': 'wait',
                    'duration': 5  # Default wait time
                }
            ]

        elif action_type == 'follow':
            action_sequence = [
                {
                    'type': 'follow_human'
                }
            ]

        return action_sequence

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]]) -> bool:
        """Execute the planned sequence of actions"""
        success = True

        for action in action_sequence:
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
                time.sleep(1.5)
                self.respond(f"Grasped the {obj}")

            elif action_type == 'return_to_user':
                self.get_logger().info('Returning to user')
                # Simulate returning
                time.sleep(2)
                self.respond("Returned to you with the object")

            elif action_type == 'navigate_to_location':
                location = action['location']
                self.get_logger().info(f'Navigating to {location}')
                # Simulate navigation
                time.sleep(2.5)
                self.respond(f"Arrived at the {location}")

            elif action_type == 'search_for_object':
                obj = action['object']
                self.get_logger().info(f'Searching for {obj}')
                # Simulate searching
                time.sleep(3)
                self.respond(f"Finished searching for {obj}")

            elif action_type == 'point_to_object':
                obj = action['object']
                self.get_logger().info(f'Pointing to {obj}')
                # Simulate pointing
                time.sleep(1)
                self.respond(f"I found the {obj} over there")

            elif action_type == 'wait':
                duration = action['duration']
                self.get_logger().info(f'Waiting for {duration} seconds')
                time.sleep(duration)
                self.respond("Wait period completed")

            elif action_type == 'follow_human':
                self.get_logger().info('Following human')
                # Simulate following
                time.sleep(4)
                self.respond("I'm following you")

            # Small delay between actions
            time.sleep(0.5)

        return success

    def respond(self, response: str):
        """Publish a response to the user"""
        response_msg = String()
        response_msg.data = response
        self.response_publisher.publish(response_msg)
        self.get_logger().info(f'Response: {response}')


def main(args=None):
    """Main function to run the complete VLA system"""
    rclpy.init(args=args)

    vla_system = CompleteVLASystem()

    print("Complete VLA System for Humanoid Robot")
    print("=" * 40)
    print("This system integrates Vision, Language, and Action for natural robot control.")
    print("The robot can understand commands like:")
    print("- 'Please bring me the red cup'")
    print("- 'Go to the kitchen and wait'")
    print("- 'Help me find my keys'")
    print("- 'Move to the living room'")
    print("")
    print("Publish commands to /vla/command topic to control the robot")
    print("Responses will be published to /vla/response topic")
    print("")

    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        pass
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 2: Create a Command Interface Node
Create `vla_command_interface.py` in the `docs/module4/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Command Interface for VLA System
Provides a simple interface for sending commands to the VLA system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios


class VLACommandInterface(Node):
    def __init__(self):
        super().__init__('vla_command_interface')

        # Publisher for commands
        self.command_publisher = self.create_publisher(String, 'vla/command', 10)

        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        # Timer for checking input
        self.timer = self.create_timer(0.1, self.check_for_input)

        self.get_logger().info("VLA Command Interface Started")
        self.get_logger().info("Type commands and press Enter to send them to the robot")
        self.get_logger().info("Press 'q' to quit")

    def check_for_input(self):
        """Check if there's input available from the terminal"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            if key == 'q':
                self.get_logger().info("Quitting...")
                rclpy.shutdown()
            elif key == '\r':  # Enter key
                # Read the full line
                line = sys.stdin.read(100)  # Read up to 100 characters
                line = line.rstrip('\n\r\x00')  # Clean up the line

                # If we have a command, publish it
                if line.strip():
                    cmd_msg = String()
                    cmd_msg.data = line.strip()
                    self.command_publisher.publish(cmd_msg)
                    self.get_logger().info(f'Sent command: "{cmd_msg.data}"')
            else:
                # Echo the character (for user feedback)
                sys.stdout.write(key)
                sys.stdout.flush()

    def destroy_node(self):
        """Restore terminal settings when shutting down"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    """Main function for the command interface"""
    rclpy.init(args=args)

    interface = VLACommandInterface()

    print("VLA Command Interface")
    print("=" * 20)
    print("Type natural language commands for the robot")
    print("Examples: 'Please bring me the cup', 'Go to the kitchen', 'Help me find my keys'")
    print("Press Enter after typing each command")
    print("Press 'q' to quit")
    print("")

    try:
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Create a Status Monitor Node
Create `vla_status_monitor.py` in the `docs/module4/nodes/` directory:

```python
#!/usr/bin/env python3
"""
Status Monitor for VLA System
Monitors and displays the status of the VLA system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class VLAStatusMonitor(Node):
    def __init__(self):
        super().__init__('vla_status_monitor')

        # Subscriber for status updates
        self.status_subscriber = self.create_subscription(
            String,
            'vla/status',
            self.status_callback,
            10
        )

        # Subscriber for responses
        self.response_subscriber = self.create_subscription(
            String,
            'vla/response',
            self.response_callback,
            10
        )

        self.get_logger().info("VLA Status Monitor Started")

    def status_callback(self, msg):
        """Handle status updates from the VLA system"""
        try:
            status_data = json.loads(msg.data)

            print("\n--- Robot Status ---")
            print(f"Position: ({status_data['position'][0]:.2f}, {status_data['position'][1]:.2f}, {status_data['position'][2]:.2f})")
            print(f"Moving: {status_data['is_moving']}")
            print(f"Executing: {status_data['is_executing']}")
            print(f"Battery: {status_data['battery_level']:.1f}%")
            print(f"Current Task: {status_data['current_task']}")
            print(f"Detected Objects: {', '.join(status_data['detected_objects']) if status_data['detected_objects'] else 'None'}")
            print("--------------------")

        except json.JSONDecodeError:
            self.get_logger().error(f"Could not parse status JSON: {msg.data}")

    def response_callback(self, msg):
        """Handle responses from the VLA system"""
        print(f"\n🤖 Robot Response: {msg.data}")


def main(args=None):
    """Main function for the status monitor"""
    rclpy.init(args=args)

    monitor = VLAStatusMonitor()

    print("VLA Status Monitor")
    print("=" * 18)
    print("Monitoring robot status and responses...")
    print("")

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Testing Your Implementation

### Running the Complete System

1. **Start the VLA system**: `ros2 run your_package complete_vla_system`
2. **Start the command interface**: `ros2 run your_package vla_command_interface`
3. **Start the status monitor**: `ros2 run your_package vla_status_monitor`
4. **Optionally, run a perception node** to provide object detections

### Example Commands to Try

- "Please bring me the cup"
- "Go to the kitchen and wait"
- "Help me find my keys"
- "Move to the living room"
- "What objects do you see?"

## Extension Challenges

1. **Integrate with simulation**: Connect to a Gazebo simulation environment
2. **Add speech recognition**: Convert speech to text for command input
3. **Improve language understanding**: Use a more sophisticated NLP model
4. **Add more complex actions**: Implement multi-step manipulation tasks
5. **Enhance scene understanding**: Add 3D scene analysis capabilities

## What You've Learned

This mini-project integrated all the concepts from Module 4 and the entire book:

- Combined vision, language, and action systems into a cohesive VLA architecture
- Implemented natural language processing for robot command interpretation
- Created scene understanding that grounds language in visual context
- Developed action planning and execution capabilities
- Built a complete humanoid robot control system
- Integrated all modules learned throughout the book

This capstone project represents the culmination of the Physical AI & Humanoid Robotics educational pathway, creating an intelligent humanoid robot capable of natural human interaction.