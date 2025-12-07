---
sidebar_position: 7
---

# Module 1 Practical Demos: ROS 2 Fundamentals

## Demo 1: Interactive Publisher/Subscriber System

This demo builds upon Project 1 to create a more interactive ROS 2 system that demonstrates real-time communication between nodes.

### Files to Create
- `interactive_talker.py` - Enhanced publisher that responds to user input
- `interactive_listener.py` - Enhanced subscriber with message processing

### Interactive Talker Code (`docs/module1/projects/interactive_talker.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios

class InteractiveTalker(Node):
    def __init__(self):
        super().__init__('interactive_talker')
        self.publisher = self.create_publisher(String, 'interactive_topic', 10)
        self.timer = self.create_timer(0.1, self.check_for_input)

        # Store terminal settings for raw input
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info("Interactive Talker Started - Type and press Enter to send messages")
        self.get_logger().info("Press 'q' to quit")

    def check_for_input(self):
        # Check if there's input available
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            if key == 'q':
                self.get_logger().info("Quitting...")
                rclpy.shutdown()
            else:
                msg = String()
                msg.data = f'Interactive message: {key} at {self.get_clock().now().seconds_nanoseconds()}'
                self.publisher.publish(msg)
                self.get_logger().info(f'Publishing: "{msg.data}"')

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    interactive_talker = InteractiveTalker()

    try:
        rclpy.spin(interactive_talker)
    except KeyboardInterrupt:
        pass
    finally:
        interactive_talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Interactive Listener Code (`docs/module1/projects/interactive_listener.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InteractiveListener(Node):
    def __init__(self):
        super().__init__('interactive_listener')
        self.subscription = self.create_subscription(
            String,
            'interactive_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.message_count = 0

        self.get_logger().info("Interactive Listener Started")

    def listener_callback(self, msg):
        self.message_count += 1
        self.get_logger().info(f'Received [{msg.data}] - Count: {self.message_count}')

def main(args=None):
    rclpy.init(args=args)
    interactive_listener = InteractiveListener()

    try:
        rclpy.spin(interactive_listener)
    except KeyboardInterrupt:
        pass
    finally:
        interactive_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Demo 2: Joint Trajectory Controller

This demo demonstrates more sophisticated joint control using trajectory messages.

### Joint Trajectory Controller (`docs/module1/projects/trajectory_controller.py`):
```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class JointTrajectoryController(Node):
    def __init__(self):
        super().__init__('joint_trajectory_controller')

        # Publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Create timer to send trajectory commands
        self.timer = self.create_timer(5.0, self.send_trajectory)

        # Define joint names
        self.joint_names = ['joint1', 'joint2', 'joint3']

        self.get_logger().info("Joint Trajectory Controller Started")

    def send_trajectory(self):
        # Create a joint trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        # Create trajectory points
        points = []

        # Point 1: Initial position
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]
        point1.velocities = [0.0, 0.0, 0.0]
        point1.accelerations = [0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)
        points.append(point1)

        # Point 2: Move to a new position smoothly
        point2 = JointTrajectoryPoint()
        point2.positions = [math.pi/4, -math.pi/6, math.pi/3]  # 45°, -30°, 60°
        point2.velocities = [0.1, -0.1, 0.15]
        point2.accelerations = [0.05, -0.05, 0.08]
        point2.time_from_start = Duration(sec=2, nanosec=0)
        points.append(point2)

        # Point 3: Return to initial position
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0]
        point3.velocities = [-0.1, 0.1, -0.15]
        point3.accelerations = [-0.05, 0.05, -0.08]
        point3.time_from_start = Duration(sec=4, nanosec=0)
        points.append(point3)

        traj_msg.points = points

        # Publish the trajectory
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info(f'Published trajectory with {len(points)} points')

def main(args=None):
    rclpy.init(args=args)
    controller = JointTrajectoryController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Demo 3: Service-Based Joint Control

This demo demonstrates using ROS 2 services for joint control, which is useful for operations that require confirmation or complex request/response patterns.

### Service Definition
First, create a custom service definition file `JointControl.srv` (conceptual - in practice, you'd use the ROS 2 service definition format):
```
# Request
string joint_name
float64 target_position
float64 max_velocity

---
# Response
bool success
string message
float64 execution_time
```

### Service Server (`docs/module1/projects/joint_control_server.py`):
```python
import rclpy
from rclpy.node import Node
from your_robot_interfaces.srv import JointControl  # Replace with actual service
import time

class JointControlServer(Node):
    def __init__(self):
        super().__init__('joint_control_server')

        # Create service
        self.srv = self.create_service(
            JointControl,
            'control_joint',
            self.control_joint_callback
        )

        self.get_logger().info("Joint Control Server Started")

    def control_joint_callback(self, request, response):
        self.get_logger().info(f'Received request to control {request.joint_name} to {request.target_position}')

        # Simulate joint movement
        start_time = time.time()

        # In a real implementation, this would interface with actual hardware/simulation
        # For this demo, we'll simulate the movement
        self.simulate_joint_movement(request.joint_name, request.target_position, request.max_velocity)

        execution_time = time.time() - start_time

        response.success = True
        response.message = f'Successfully moved {request.joint_name} to {request.target_position}'
        response.execution_time = execution_time

        self.get_logger().info(f'Response: {response.message}, Time: {response.execution_time:.2f}s')

        return response

    def simulate_joint_movement(self, joint_name, target_position, max_velocity):
        # Simulate the time it takes to move the joint
        # In a real system, this would control the actual joint
        self.get_logger().info(f'Simulating movement of {joint_name} to {target_position}')
        time.sleep(0.5)  # Simulate movement time

def main(args=None):
    rclpy.init(args=args)
    server = JointControlServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client (`docs/module1/projects/joint_control_client.py`):
```python
import rclpy
from rclpy.node import Node
from your_robot_interfaces.srv import JointControl  # Replace with actual service
import sys

class JointControlClient(Node):
    def __init__(self):
        super().__init__('joint_control_client')
        self.cli = self.create_client(JointControl, 'control_joint')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = JointControl.Request()

    def send_request(self, joint_name, target_position, max_velocity):
        self.req.joint_name = joint_name
        self.req.target_position = target_position
        self.req.max_velocity = max_velocity

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 4:
        print("Usage: ros2 run package joint_control_client <joint_name> <target_position> <max_velocity>")
        return

    client = JointControlClient()

    joint_name = sys.argv[1]
    target_position = float(sys.argv[2])
    max_velocity = float(sys.argv[3])

    response = client.send_request(joint_name, target_position, max_velocity)

    if response:
        print(f'Result: {response.message}, Execution Time: {response.execution_time:.2f}s')
    else:
        print('Service call failed')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Demos

### Demo 1: Interactive Publisher/Subscriber
1. Open two terminals
2. In Terminal 1: `ros2 run your_package interactive_talker`
3. In Terminal 2: `ros2 run your_package interactive_listener`
4. Type characters in the talker terminal and see them appear in the listener

### Demo 2: Joint Trajectory Controller
1. Run: `ros2 run your_package trajectory_controller`
2. Watch the trajectory messages being published to control robot joints

### Demo 3: Service-Based Joint Control
1. In Terminal 1: `ros2 run your_package joint_control_server`
2. In Terminal 2: `ros2 run your_package joint_control_client joint1 1.57 0.5`
3. Observe the service request/response interaction

## Key Learning Points

These practical demos reinforce important concepts:
- Real-time interaction with ROS 2 nodes
- Different communication patterns (topics vs services)
- Trajectory planning and execution
- Asynchronous vs synchronous communication
- Error handling and response validation

Each demo builds upon the basic concepts learned in the module while introducing more advanced patterns used in real robotics applications.