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

ros2 topic pub /atc/perception std_msgs/msg/String "{data: STOP}"



## Mapping World with Prius

ros2 run tf2_ros static_transform_publisher  0 0 0 0 0 0 prius/odom prius/base_link

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

rviz2 -d nav2_bringup/bringup/rviz/nav2_default_view.rviz 

## Move Prius

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/prius/cmd_vel



## Save the Map

ros2 run nav2_map_server map_saver_cli -f ~/map


# Make Prius Move
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{ "linear": { "x": 1.0,"y": 0.0, "z": 0.0 }, "angular": { "x": 0.0, "y": 0.0, "z": 0.0 } }'