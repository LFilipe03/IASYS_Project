# Launch SLAM toolbox with namespace
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True \
  namespace:=prius \
  scan_topic:=/prius/base_scan \
  odom_frame:=prius/odom \
  base_frame:=prius/base_link

# Drive around manually to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/prius/cmd_vel

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/my_map