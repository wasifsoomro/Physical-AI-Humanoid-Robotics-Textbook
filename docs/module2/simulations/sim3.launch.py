import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to the URDF file with sensors
    urdf_file_path = os.path.join(
        get_package_share_directory('humanoid_robotics_book'), # Replace with your package name
        'docs', 'module2', 'urdf', 'humanoid_with_sensors.urdf'
    )

    # Launch Gazebo itself
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            'gazebo.launch.py'
        ])
    )

    # Robot State Publisher node
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

    # Joint State Publisher (for non-fixed joints if any)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid_with_sensors', '-file', urdf_file_path, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # Example ROS 2 node to interact with the simulated robot
    # This could be a controller or sensor data processor
    robot_controller = Node(
        package='humanoid_robotics_book',  # Replace with your package name
        executable='simple_controller',  # This would be a custom node you create
        name='simple_controller',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # robot_controller  # Uncomment if you have the controller node
    ])