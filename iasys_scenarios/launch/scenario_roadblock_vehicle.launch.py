from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    gazebo_ros = get_package_share_directory('gazebo_ros')
    prius_description = get_package_share_directory('prius_description')

    world_path = os.path.join(
        get_package_share_directory('gazebo_nodes'),
        'worlds',
        'Autoestrada.world'
    )


    return LaunchDescription([

        # Start Gazebo with ROS plugins
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

    ])
