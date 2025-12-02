from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    
    pkg_path = get_package_share_directory('gazebo_nodes')
    world_path = os.path.join(pkg_path, 'worlds', 'Autoestrada.world')


    """
    # Spawn entities
    robot1_description = os.path.join(pkg_path, 'models', 'robot1.urdf')
    robot2_description = os.path.join(pkg_path, 'models', 'robot2.urdf')
    """

    return LaunchDescription([

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'
        ),
        
    ])