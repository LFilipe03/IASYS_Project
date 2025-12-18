# IASYS_Project

## Requirements

ROS2-Foxy, Gazebo 11, Yolov8

sudo apt install ros-foxy-gazebo-*

## Running the example

cd IASYS_Project

colcon build

. install/setup.bash

ros2 launch iasys_scenarios scenario_roadblock_vehicle.launch.py 

ros2 launch iasys_scenarios spawn_prius.py


## Mapping World with Prius

ros2 launch slam_toolbox online_async_launch.py 

ros2 run tf2_ros static_transform_publisher  0 0 0 0 0 0 prius/odom prius/base_link

## Save the Map

ros2 run nav2_map_server map_saver_cli -f ~/map


# Make Prius Move
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{ "linear": { "x": 1.0,"y": 0.0, "z": 0.0 }, "angular": { "x": 0.0, "y": 0.0, "z": 0.0 } }'