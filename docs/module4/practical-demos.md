---
sidebar_position: 8
---

# Module 4 Practical Demos: Vision-Language-Action Robotics

## Demo 1: Simple VLA Command Processor

This demo shows a basic implementation of the Vision-Language-Action pipeline for processing natural language commands.

### Simple VLA Processor (`docs/module4/demos/simple_vla_demo.py`):
```python
#!/usr/bin/env python3
"""
Simple VLA Command Processor Demo
Demonstrates the basic Vision-Language-Action pipeline.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import json
import re
from typing import Dict, List, Optional


class SimpleVLAProcessor(Node):
    def __init__(self):
        super().__init__('simple_vla_processor')

        # Initialize components
        self.bridge = CvBridge()
        self.current_objects = []
        self.robot_position = (0.0, 0.0, 0.0)

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            '/vla/simple_command',
            self.command_callback,
            10
        )

        self.response_pub = self.create_publisher(String, '/vla/simple_response', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)

        self.get_logger().info('Simple VLA Processor Started')

    def command_callback(self, msg):
        """Process a natural language command"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Step 1: Language Understanding
        parsed_command = self.parse_command(command)

        if not parsed_command:
            self.respond(f"Sorry, I didn't understand: {command}")
            return

        # Step 2: Vision Integration (simulated)
        vision_context = self.get_simulated_vision_context()

        # Step 3: Action Planning
        action_plan = self.plan_action(parsed_command, vision_context)

        # Step 4: Action Execution
        self.execute_action(action_plan)

    def parse_command(self, command: str) -> Optional[Dict]:
        """Parse natural language command"""
        command_lower = command.lower()

        # Simple pattern matching for demonstration
        if 'bring' in command_lower or 'get' in command_lower:
            # Extract object to bring
            obj_match = re.search(r'(cup|book|phone|keys|bottle)', command_lower)
            if obj_match:
                return {
                    'action': 'fetch',
                    'object': obj_match.group(1)
                }

        elif 'go to' in command_lower or 'move to' in command_lower:
            # Extract location
            loc_match = re.search(r'(kitchen|living room|bedroom|office)', command_lower)
            if loc_match:
                return {
                    'action': 'navigate',
                    'location': loc_match.group(1)
                }

        elif 'find' in command_lower or 'locate' in command_lower:
            # Extract object to find
            obj_match = re.search(r'(keys|phone|wallet|book)', command_lower)
            if obj_match:
                return {
                    'action': 'find',
                    'object': obj_match.group(1)
                }

        return None

    def get_simulated_vision_context(self) -> Dict:
        """Simulate getting vision context (in real system, this would process camera data)"""
        return {
            'objects': [
                {'name': 'red cup', 'position': (1.5, 2.0, 0.0)},
                {'name': 'blue book', 'position': (3.0, 1.0, 0.0)},
                {'name': 'phone', 'position': (0.5, 0.5, 0.0)}
            ],
            'locations': {
                'kitchen': (5.0, 5.0, 0.0),
                'living room': (2.0, 2.0, 0.0),
                'bedroom': (8.0, 3.0, 0.0),
                'office': (1.0, 7.0, 0.0)
            }
        }

    def plan_action(self, parsed_command: Dict, vision_context: Dict) -> Dict:
        """Plan action based on command and vision context"""
        action_plan = {
            'command': parsed_command,
            'steps': []
        }

        if parsed_command['action'] == 'fetch':
            # Find object in vision context
            target_obj = parsed_command['object']
            found_obj = None
            for obj in vision_context['objects']:
                if target_obj in obj['name']:
                    found_obj = obj
                    break

            if found_obj:
                action_plan['steps'] = [
                    {'type': 'navigate', 'target': found_obj['position'], 'description': f'Navigating to {target_obj}'},
                    {'type': 'grasp', 'target': found_obj['name'], 'description': f'Grasping {target_obj}'},
                    {'type': 'return', 'target': self.robot_position, 'description': 'Returning to user'}
                ]
            else:
                action_plan['steps'] = [
                    {'type': 'search', 'target': target_obj, 'description': f'Searching for {target_obj}'}
                ]

        elif parsed_command['action'] == 'navigate':
            target_location = parsed_command['location']
            if target_location in vision_context['locations']:
                action_plan['steps'] = [
                    {'type': 'navigate', 'target': vision_context['locations'][target_location],
                     'description': f'Navigating to {target_location}'}
                ]

        elif parsed_command['action'] == 'find':
            target_obj = parsed_command['object']
            found_obj = None
            for obj in vision_context['objects']:
                if target_obj in obj['name']:
                    found_obj = obj
                    break

            if found_obj:
                action_plan['steps'] = [
                    {'type': 'point_to', 'target': found_obj['position'], 'description': f'Pointing to {target_obj}'}
                ]
            else:
                action_plan['steps'] = [
                    {'type': 'search', 'target': target_obj, 'description': f'Searching for {target_obj}'}
                ]

        return action_plan

    def execute_action(self, action_plan: Dict):
        """Execute the planned action"""
        command = action_plan['command']
        self.respond(f"Starting task: {command['action']} {command.get('object', command.get('location', ''))}")

        for step in action_plan['steps']:
            self.get_logger().info(f"Executing: {step['description']}")

            # Simulate action execution
            if step['type'] == 'navigate':
                # In a real system, this would send navigation commands
                self.simulate_navigation(step['target'])
            elif step['type'] == 'grasp':
                # In a real system, this would send manipulation commands
                self.simulate_grasp(step['target'])
            elif step['type'] == 'return':
                # In a real system, this would navigate back to user
                self.simulate_navigation(step['target'])
            elif step['type'] == 'search':
                # Simulate searching
                self.get_logger().info(f"Searching for {step['target']}")
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3))

        self.respond(f"Completed task: {command['action']} {command.get('object', command.get('location', ''))}")

    def simulate_navigation(self, target_position):
        """Simulate navigation to a target position"""
        self.get_logger().info(f"Navigating to position: {target_position}")
        # In real system, this would send Twist commands to move the robot
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))

    def simulate_grasp(self, target_object):
        """Simulate grasping an object"""
        self.get_logger().info(f"Grasping object: {target_object}")
        # In real system, this would send manipulation commands
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.5))

    def respond(self, response: str):
        """Publish a response"""
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)
        self.get_logger().info(f"Response: {response}")


def main(args=None):
    rclpy.init(args=args)
    vla_processor = SimpleVLAProcessor()

    print("Simple VLA Command Processor Demo")
    print("=" * 35)
    print("This demo shows the basic VLA pipeline: Vision-Language-Action")
    print("Send commands to /vla/simple_command topic")
    print("Responses will be published to /vla/simple_response topic")
    print("")

    try:
        rclpy.spin(vla_processor)
    except KeyboardInterrupt:
        pass
    finally:
        vla_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Demo 2: Natural Language Command Interpreter

This demo focuses specifically on the language understanding component of VLA systems.

### Natural Language Interpreter (`docs/module4/demos/nlu_demo.py`):
```python
#!/usr/bin/env python3
"""
Natural Language Understanding Demo for VLA Systems
Focuses on interpreting natural language commands for robots.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re
from typing import Dict, List, Optional


class NaturalLanguageInterpreter(Node):
    def __init__(self):
        super().__init__('nlu_interpreter')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            '/nlu/command',
            self.command_callback,
            10
        )

        self.interpretation_pub = self.create_publisher(String, '/nlu/interpretation', 10)
        self.feedback_pub = self.create_publisher(String, '/nlu/feedback', 10)

        # Define command patterns
        self.patterns = {
            'navigation': [
                (r'go to (.*?)$', ['navigate_to']),
                (r'move to (.*?)$', ['navigate_to']),
                (r'go (.*?)$', ['navigate_to']),
                (r'move (.*?)$', ['navigate_to']),
            ],
            'fetch': [
                (r'bring me (.*?)$', ['fetch', 'target']),
                (r'get (.*?)$', ['fetch', 'target']),
                (r'fetch (.*?)$', ['fetch', 'target']),
                (r'pick up (.*?)$', ['fetch', 'target']),
            ],
            'find': [
                (r'find (.*?)$', ['find', 'target']),
                (r'locate (.*?)$', ['find', 'target']),
                (r'where is (.*?)$', ['find', 'target']),
                (r'look for (.*?)$', ['find', 'target']),
            ],
            'action': [
                (r'wait', ['wait']),
                (r'stop', ['stop']),
                (r'pause', ['pause']),
                (r'follow', ['follow']),
            ]
        }

        self.get_logger().info('Natural Language Interpreter Started')

    def command_callback(self, msg):
        """Process incoming natural language command"""
        raw_command = msg.data
        self.get_logger().info(f'Received command: {raw_command}')

        # Interpret the command
        interpretation = self.interpret_command(raw_command)

        if interpretation:
            # Publish interpretation
            interpretation_msg = String()
            interpretation_msg.data = json.dumps(interpretation)
            self.interpretation_pub.publish(interpretation_msg)

            # Provide feedback
            feedback = self.generate_feedback(interpretation)
            feedback_msg = String()
            feedback_msg.data = feedback
            self.feedback_pub.publish(feedback_msg)

            self.get_logger().info(f'Interpretation: {interpretation}')
            self.get_logger().info(f'Feedback: {feedback}')
        else:
            # Command not understood
            feedback_msg = String()
            feedback_msg.data = f"Sorry, I didn't understand: {raw_command}"
            self.feedback_pub.publish(feedback_msg)

    def interpret_command(self, command: str) -> Optional[Dict]:
        """Interpret a natural language command"""
        command_lower = command.lower().strip()

        # Try each pattern category
        for category, pattern_list in self.patterns.items():
            for pattern, tags in pattern_list:
                match = re.search(pattern, command_lower)
                if match:
                    interpretation = {
                        'category': category,
                        'action': tags[0],
                        'raw_command': command,
                        'confidence': 0.9  # High confidence for rule-based matching
                    }

                    # Extract additional information if available
                    if len(tags) > 1 and len(match.groups()) > 0:
                        interpretation[tags[1]] = match.group(1).strip()

                    return interpretation

        # If no pattern matches, return None
        return None

    def generate_feedback(self, interpretation: Dict) -> str:
        """Generate natural language feedback for the interpretation"""
        action = interpretation['action']

        if action == 'navigate_to':
            target = interpretation.get('target', 'unknown location')
            return f"I understand you want me to navigate to {target}. I will proceed with navigation."

        elif action == 'fetch':
            target = interpretation.get('target', 'unknown object')
            return f"I understand you want me to fetch {target}. I will look for it and bring it to you."

        elif action == 'find':
            target = interpretation.get('target', 'unknown object')
            return f"I understand you want me to find {target}. I will search for it in the environment."

        elif action == 'wait':
            return "I understand you want me to wait. I will remain in my current position."

        elif action == 'stop':
            return "I understand you want me to stop. I will halt all current activities."

        elif action == 'pause':
            return "I understand you want me to pause. I will temporarily stop and wait for further instructions."

        elif action == 'follow':
            return "I understand you want me to follow. I will maintain a safe distance and follow your movements."

        else:
            return f"I understand you want me to perform an action: {action}. I will proceed accordingly."

    def add_custom_pattern(self, pattern: str, tags: List[str], category: str):
        """Add a custom pattern for command interpretation"""
        if category not in self.patterns:
            self.patterns[category] = []
        self.patterns[category].append((pattern, tags))


def main(args=None):
    rclpy.init(args=args)
    nlu_interpreter = NaturalLanguageInterpreter()

    print("Natural Language Understanding Demo")
    print("=" * 35)
    print("This demo focuses on interpreting natural language commands.")
    print("Send commands to /nlu/command topic")
    print("Interpretations will be published to /nlu/interpretation topic")
    print("Feedback will be published to /nlu/feedback topic")
    print("")
    print("Example commands:")
    print("- 'Please go to the kitchen'")
    print("- 'Bring me the red cup'")
    print("- 'Find my keys'")
    print("- 'Wait here'")
    print("")

    try:
        rclpy.spin(nlu_interpreter)
    except KeyboardInterrupt:
        pass
    finally:
        nlu_interpreter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Demo 3: Vision-Grounded Language Understanding

This demo shows how visual context can help interpret ambiguous language commands.

### Vision-Grounded Understanding (`docs/module4/demos/vision_grounding_demo.py`):
```python
#!/usr/bin/env python3
"""
Vision-Grounded Language Understanding Demo
Shows how visual context helps interpret ambiguous language commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import json
from typing import Dict, List


class VisionGroundingDemo(Node):
    def __init__(self):
        super().__init__('vision_grounding_demo')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            '/grounding/command',
            self.command_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/perception/detections',
            self.detection_callback,
            10
        )

        self.response_pub = self.create_publisher(String, '/grounding/response', 10)

        # Store detected objects
        self.detected_objects = []

        self.get_logger().info('Vision Grounding Demo Started')

    def detection_callback(self, msg):
        """Process object detections from perception system"""
        self.detected_objects = []

        for detection in msg.detections:
            if detection.results:
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

        self.get_logger().info(f'Detected {len(self.detected_objects)} objects')

    def command_callback(self, msg):
        """Process command with visual grounding"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Ground the command in visual context
        grounded_interpretation = self.ground_command_in_vision(command)

        # Generate response
        response = self.generate_response(grounded_interpretation)

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

        self.get_logger().info(f'Response: {response}')

    def ground_command_in_vision(self, command: str) -> Dict:
        """Ground command in visual context to resolve ambiguities"""
        interpretation = {
            'original_command': command,
            'resolved_entities': [],
            'action': 'unknown',
            'confidence': 0.0
        }

        # Simple command parsing for demonstration
        command_lower = command.lower()

        if 'bring me the' in command_lower or 'get the' in command_lower:
            # Extract the object type (e.g., "cup", "book")
            obj_type_match = re.search(r'the (\w+)', command_lower)
            if obj_type_match:
                obj_type = obj_type_match.group(1)

                # Look for this object type in detected objects
                matching_objects = [obj for obj in self.detected_objects
                                  if obj_type.lower() in obj['label'].lower()]

                if matching_objects:
                    # Select the most confident match
                    best_match = max(matching_objects, key=lambda x: x['confidence'])
                    interpretation['action'] = 'fetch'
                    interpretation['target_object'] = best_match['label']
                    interpretation['target_position'] = (best_match['x'], best_match['y'])
                    interpretation['confidence'] = best_match['confidence']
                    interpretation['resolved_entities'] = [best_match]
                else:
                    interpretation['action'] = 'search'
                    interpretation['target_object'] = obj_type
                    interpretation['confidence'] = 0.3  # Lower confidence when object not found

        elif 'go to' in command_lower:
            # This would typically reference locations, not objects
            interpretation['action'] = 'navigate'
            interpretation['confidence'] = 0.8

        elif 'point to' in command_lower or 'show me' in command_lower:
            obj_type_match = re.search(r'(?:to|me) (?:the )?(\w+)', command_lower)
            if obj_type_match:
                obj_type = obj_type_match.group(1)
                matching_objects = [obj for obj in self.detected_objects
                                  if obj_type.lower() in obj['label'].lower()]

                if matching_objects:
                    best_match = max(matching_objects, key=lambda x: x['confidence'])
                    interpretation['action'] = 'point_to'
                    interpretation['target_object'] = best_match['label']
                    interpretation['target_position'] = (best_match['x'], best_match['y'])
                    interpretation['confidence'] = best_match['confidence']
                    interpretation['resolved_entities'] = [best_match]
                else:
                    interpretation['action'] = 'report_absence'
                    interpretation['target_object'] = obj_type
                    interpretation['confidence'] = 0.3

        return interpretation

    def generate_response(self, interpretation: Dict) -> str:
        """Generate a natural language response based on the interpretation"""
        action = interpretation['action']
        confidence = interpretation['confidence']

        if confidence < 0.5:
            return f"I'm not sure I understood correctly. For command '{interpretation['original_command']}', I think you want me to {action}, but I'm not confident in my interpretation."

        if action == 'fetch':
            obj = interpretation.get('target_object', 'unknown object')
            pos = interpretation.get('target_position', (0, 0))
            return f"I will fetch the {obj} located at position {pos}. I'm confident I can complete this task."

        elif action == 'search':
            obj = interpretation.get('target_object', 'unknown object')
            return f"I will search for the {obj}, but I don't currently see it in my field of view. I'll look around for it."

        elif action == 'navigate':
            return "I will navigate as requested. Please let me know if you need me to go to a specific location."

        elif action == 'point_to':
            obj = interpretation.get('target_object', 'unknown object')
            pos = interpretation.get('target_position', (0, 0))
            return f"I can see the {obj} at position {pos}. I'm pointing to it now."

        elif action == 'report_absence':
            obj = interpretation.get('target_object', 'unknown object')
            return f"I don't see the {obj} in my current view. Would you like me to search for it?"

        else:
            return f"I will attempt to {action}. Please let me know if this is what you intended."

    def get_object_by_label(self, label: str) -> Dict:
        """Helper to get object information by label"""
        for obj in self.detected_objects:
            if label.lower() in obj['label'].lower():
                return obj
        return None


def main(args=None):
    rclpy.init(args=args)
    grounding_demo = VisionGroundingDemo()

    print("Vision-Grounded Language Understanding Demo")
    print("=" * 45)
    print("This demo shows how visual context helps interpret ambiguous language.")
    print("Send commands to /grounding/command topic")
    print("The system will use object detections to ground language references.")
    print("")
    print("Example commands (try with different objects in view):")
    print("- 'Bring me the cup'")
    print("- 'Point to the book'")
    print("- 'Show me the red object'")
    print("")

    try:
        rclpy.spin(grounding_demo)
    except KeyboardInterrupt:
        pass
    finally:
        grounding_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import re  # Import re for regular expressions
    main()
```

## Running the Demos

### Demo 1: Simple VLA Processor
1. Run: `ros2 run your_package simple_vla_demo`
2. Send commands to `/vla/simple_command` topic
3. Observe responses on `/vla/simple_response` topic

### Demo 2: Natural Language Interpreter
1. Run: `ros2 run your_package nlu_demo`
2. Send commands to `/nlu/command` topic
3. Observe interpretations on `/nlu/interpretation` topic
4. Observe feedback on `/nlu/feedback` topic

### Demo 3: Vision-Grounded Understanding
1. Run: `ros2 run your_package vision_grounding_demo`
2. Ensure object detections are being published to `/perception/detections`
3. Send commands to `/grounding/command` topic
4. Observe responses on `/grounding/response` topic

## Key Learning Points

These practical demos reinforce important VLA concepts:
- The pipeline from language understanding to action execution
- How visual context helps resolve linguistic ambiguities
- The integration of multiple modalities for natural interaction
- Real-time processing considerations for natural interaction
- Error handling and feedback in VLA systems

Each demo builds upon the basic concepts learned in the module while introducing more sophisticated integration techniques used in real VLA robotics applications.