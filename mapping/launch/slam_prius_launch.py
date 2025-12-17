import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Package Directories

    mapping_pkg_dir = get_package_share_directory('mapping')

    use_sim_time = LaunchConfiguration('use_sim_time')


    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Path to params file 
    slam_params_file = os.path.join( mapping_pkg_dir, 'slam', 'params_sync_slam_toolbox.yaml' ) 


    # Slam Node

    slam_node_cmd = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('scan', '/prius/base_scan')],
    )

    # Rviz Node

    rviz_node_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='prius',
        output='screen',
        arguments=['-d', os.path.join(mapping_pkg_dir, 'rviz', 'config_prius_mapping.rviz')]
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'prius/odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    map_to_prius_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'prius/base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    
    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(use_sim_time_arg)

    launch_description.add_action(slam_node_cmd)
    launch_description.add_action(rviz_node_cmd)
    #launch_description.add_action(map_to_odom_tf)
    launch_description.add_action(map_to_prius_tf)

    return launch_description