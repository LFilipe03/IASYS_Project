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


    """
    # Spawn entities
    robot1_description = os.path.join(pkg_path, 'models', 'robot1.urdf')
    robot2_description = os.path.join(pkg_path, 'models', 'robot2.urdf')
    """

    return LaunchDescription([

        # Spawn Prius vehicle
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-file', prius_path,
                    '-entity', 'prius',
                    '-x', '0',
                    '-y', '-5',
                    '-z', '0'],
                    output='screen'
        ),
        
    ])