import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
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

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid_with_sensors', '-file', urdf_file_path, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity,
    ])