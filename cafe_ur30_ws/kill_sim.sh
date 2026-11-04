#!/bin/sh
# Kill leftovers of a previous run (Ctrl+C often leaves the Gazebo server alive).
for p in "ign gazebo" "gz sim" ruby parameter_bridge move_group rviz2 robot_state_publisher pick_place_executor tray_detector_node task_manager_node static_transform_publisher; do
  pkill -9 -f "$p" 2>/dev/null
done
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
echo "cleaned. remaining nodes:"; ros2 node list
