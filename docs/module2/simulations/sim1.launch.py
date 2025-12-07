import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the share directory for your robot description package (assuming you have one)
    # For this simple example, we'll use a placeholder. In a real project, this would be your robot's package.
    # urdf_file_name = 'simple_humanoid.urdf' # Assuming this URDF exists in a package
    # urdf_path = os.path.join(
    #     get_package_share_directory('your_robot_description_package'),
    #     'urdf',
    #     urdf_file_name
    # )

    # For now, let's directly use the URDF created in Module 1, Project 2
    # You would need to make sure this path is correct relative to where you run the launch file
    # For a proper ROS 2 setup, this URDF should be part of a package and installed.
    urdf_file_path = os.path.join(
        get_package_share_directory('humanoid_robotics_book'), # Replace with your package name
        'docs', 'module1', 'projects', 'project2.urdf'
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
        arguments=['-entity', 'simple_humanoid', '-file', urdf_file_path, '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity,
    ])