---
sidebar_position: 7
---

# Module 2 Practical Demos: Digital Twin Simulation

## Demo 1: Interactive Sensor Visualization

This demo creates a node that subscribes to multiple sensor streams from the simulated robot and provides real-time visualization of the data.

### Sensor Visualization Node (`docs/module2/nodes/sensor_visualizer.py`):
```python
#!/usr/bin/env python3
"""
Sensor Visualizer Node for Humanoid Robot Simulation
This node subscribes to sensor data and provides real-time visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import threading
import time

class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')

        # Create subscribers for different sensor types
        self.imu_subscription = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/humanoid/scan',
            self.scan_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/humanoid/camera/depth/image_raw',
            self.camera_callback,
            10
        )

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Data storage for visualization
        self.imu_data = {'orientation': None, 'angular_velocity': None, 'linear_acceleration': None}
        self.scan_data = {'ranges': [], 'intensities': [], 'angle_min': 0.0, 'angle_max': 0.0, 'angle_increment': 0.0}
        self.camera_data = {'image': None, 'width': 0, 'height': 0}

        # Visualization setup
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 10))
        self.visualize_thread = threading.Thread(target=self.run_visualization, daemon=True)
        self.visualize_thread.start()

        self.get_logger().info('Sensor Visualizer Node Started')

    def imu_callback(self, msg):
        self.imu_data['orientation'] = msg.orientation
        self.imu_data['angular_velocity'] = msg.angular_velocity
        self.imu_data['linear_acceleration'] = msg.linear_acceleration

    def scan_callback(self, msg):
        self.scan_data['ranges'] = np.array(msg.ranges)
        self.scan_data['intensities'] = np.array(msg.intensities) if msg.intensities else np.array([])
        self.scan_data['angle_min'] = msg.angle_min
        self.scan_data['angle_max'] = msg.angle_max
        self.scan_data['angle_increment'] = msg.angle_increment

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.camera_data['image'] = cv_image
            self.camera_data['width'] = msg.width
            self.camera_data['height'] = msg.height
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def run_visualization(self):
        """Run the visualization in a separate thread"""
        plt.ion()  # Turn on interactive mode

        while rclpy.ok():
            self.update_plots()
            time.sleep(0.1)  # Update every 100ms

    def update_plots(self):
        """Update all plots with current sensor data"""
        if not self.scan_data['ranges'].size == 0:
            # Clear and update LiDAR plot (top left)
            self.axs[0, 0].clear()
            angles = np.arange(self.scan_data['angle_min'], self.scan_data['angle_max'], self.scan_data['angle_increment'])
            if len(angles) == len(self.scan_data['ranges']):
                x = self.scan_data['ranges'] * np.cos(angles)
                y = self.scan_data['ranges'] * np.sin(angles)
                self.axs[0, 0].scatter(x, y, s=1, c='blue')
                self.axs[0, 0].set_title('LiDAR Scan (Top-Down View)')
                self.axs[0, 0].set_xlabel('X (m)')
                self.axs[0, 0].set_ylabel('Y (m)')
                self.axs[0, 0].grid(True)
                self.axs[0, 0].set_aspect('equal')

        if self.imu_data['orientation']:
            # Update IMU orientation plot (top right)
            self.axs[0, 1].clear()
            # Convert quaternion to Euler angles for simple display
            # (In a real implementation, you'd use proper quaternion-to-Euler conversion)
            self.axs[0, 1].bar(['Roll', 'Pitch', 'Yaw'], [0, 0, 0])  # Placeholder
            self.axs[0, 1].set_title('IMU Orientation (Placeholder)')
            self.axs[0, 1].set_ylabel('Angle (rad)')

        if self.camera_data['image'] is not None:
            # Update camera image plot (bottom left)
            self.axs[1, 0].clear()
            self.axs[1, 0].imshow(self.camera_data['image'], cmap='gray')
            self.axs[1, 0].set_title('Depth Camera Image')
            self.axs[1, 0].axis('off')

        if self.imu_data['angular_velocity']:
            # Update IMU angular velocity plot (bottom right)
            self.axs[1, 1].clear()
            ang_vel = self.imu_data['angular_velocity']
            self.axs[1, 1].bar(['X', 'Y', 'Z'], [ang_vel.x, ang_vel.y, ang_vel.z])
            self.axs[1, 1].set_title('IMU Angular Velocity')
            self.axs[1, 1].set_ylabel('Angular Velocity (rad/s)')

        plt.tight_layout()
        plt.pause(0.001)  # Small pause to update the plot

def main(args=None):
    rclpy.init(args=args)

    sensor_visualizer = SensorVisualizer()

    try:
        # Run the node but allow visualization thread to continue
        rclpy.spin(sensor_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Demo 2: Physics Parameter Tuning

This demo shows how to adjust physics parameters in simulation to match real-world robot behavior.

### Physics Tuning Script (`docs/module2/nodes/physics_tuner.py`):
```python
#!/usr/bin/env python3
"""
Physics Parameter Tuning Node
This node demonstrates how to adjust physics parameters in simulation to match real-world behavior.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class PhysicsTuner(Node):
    def __init__(self):
        super().__init__('physics_tuner')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)

        # Subscriber for odometry to measure actual movement
        self.odom_sub = self.create_subscription(
            Odometry,
            '/humanoid/odom',
            self.odom_callback,
            10
        )

        # Store initial position for movement calculations
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.command_step = 0
        self.command_sequence = [
            {'linear_x': 0.5, 'duration': 3.0},  # Move forward
            {'linear_x': 0.0, 'duration': 1.0},  # Stop
            {'linear_x': -0.5, 'duration': 3.0}, # Move backward
            {'linear_x': 0.0, 'duration': 1.0},  # Stop
        ]
        self.command_start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('Physics Tuner Node Started')

    def odom_callback(self, msg):
        """Store current position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.command_step == 0:  # Store initial position
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.command_step = 1

    def timer_callback(self):
        """Send velocity commands according to the sequence"""
        if self.command_step >= len(self.command_sequence):
            return  # All commands sent

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.command_start_time

        cmd = self.command_sequence[self.command_step]
        if elapsed < cmd['duration']:
            # Send command
            twist_msg = Twist()
            twist_msg.linear.x = cmd['linear_x']
            self.cmd_vel_pub.publish(twist_msg)
        else:
            # Move to next command
            distance_traveled = ((self.current_x - self.initial_x)**2 +
                                (self.current_y - self.initial_y)**2)**0.5

            self.get_logger().info(
                f'Completed command {self.command_step}: '
                f'Target: {cmd["linear_x"]}, '
                f'Distance: {distance_traveled:.3f}m, '
                f'Duration: {elapsed:.2f}s'
            )

            self.command_step += 1
            self.command_start_time = current_time

            # Stop robot between commands
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)

    physics_tuner = PhysicsTuner()

    try:
        rclpy.spin(physics_tuner)
    except KeyboardInterrupt:
        pass
    finally:
        physics_tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Demo 3: Multi-Robot Simulation

This demo shows how to simulate multiple robots in the same Gazebo environment.

### Multi-Robot Launch File (`docs/module2/simulations/multi_robot.launch.py`):
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots', default='2')

    # Path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('humanoid_robotics_book'),  # Replace with your package name
        'docs', 'module2', 'urdf', 'advanced_humanoid.urdf'
    )

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            'gazebo.launch.py'
        ])
    )

    # Robot State Publisher (shared)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file_path).read()
        }],
        output='screen'
    )

    # Create nodes for each robot
    launch_actions = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'num_robots',
            default_value='2',
            description='Number of robots to spawn'
        ),
        gazebo_launch,
        robot_state_publisher,
    ]

    # Spawn multiple robots with different names and positions
    for i in range(int(num_robots.perform(LaunchConfiguration('num_robots')))):
        x_pos = float(i) * 2.0  # Space robots 2m apart on x-axis

        # Joint State Publisher for each robot
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=f'joint_state_publisher_{i}',
            namespace=f'robot_{i}',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        )

        # Spawn robot in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'robot_{i}',
                '-file', urdf_file_path,
                '-x', str(x_pos),
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        )

        launch_actions.extend([joint_state_publisher, spawn_entity])

    return LaunchDescription(launch_actions)
```

## Running the Demos

### Demo 1: Interactive Sensor Visualization
1. Start the simulation with sensors: `ros2 launch your_package advanced_simulation.launch.py`
2. In another terminal: `ros2 run your_package sensor_visualizer`
3. Observe real-time visualization of sensor data

### Demo 2: Physics Parameter Tuning
1. Launch the simulation with a mobile robot model
2. Run the physics tuner: `ros2 run your_package physics_tuner`
3. Observe how the robot moves and compare to expected behavior

### Demo 3: Multi-Robot Simulation
1. Launch multiple robots: `ros2 launch your_package multi_robot.launch.py`
2. Observe multiple robots in the same simulation environment

## Key Learning Points

These practical demos reinforce important simulation concepts:
- Real-time sensor data visualization and processing
- Physics parameter tuning for realistic simulation
- Multi-robot simulation scenarios
- Integration of multiple sensor types
- Advanced launch file configurations

Each demo builds upon the basic concepts learned in the module while introducing more sophisticated simulation techniques used in real robotics applications.