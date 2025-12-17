from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


     # Get the package directory of prius_description
    prius_pkg = get_package_share_directory('prius_description')

    # Build the full path to the URDF
    prius_path = os.path.join(prius_pkg, 'urdf', 'prius.urdf')


    # Read the URDF file
    with open(prius_path, 'r') as urdf_file:
        robot_description = urdf_file.read()
    
    return LaunchDescription([
        # Robot State Publisher - publishes TF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='prius',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
                'frame_prefix': 'prius/'  # Important for namespacing!
            }]
        ),
        

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            namespace='prius',
            arguments=['3.55', '0', '0.3', '0', '0', '0', 'prius/base_link', 'center_laser_link'],
            parameters=[{'use_sim_time': True}]
        ),

        # Spawn Prius vehicle in Gazebo
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-file', prius_path,
                 '-entity', 'prius',
                 '-robot_namespace', 'prius',
                 '-x', '240',
                 '-y', '7.5',
                 '-z', '0',
                 '-Y', '3.1416'],
            output='screen'
        ),
    ])