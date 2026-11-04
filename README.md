Assessment: ROS Developer (Vision-Based Manipulator System)


A camera detects coloured food trays lying randomly on a kitchen table; a UR30 with a parallel gripper
picks each tray and puts it in the serving area that belongs to its colour. Handles missing trays,
failed grasps (retry → skip), trays that are moved while running, and colour priority.



Preoject Specification


Project Developed in ROS2 humble in ubuntu 22.04
Robot – UR30
Gripper- custom 2-finger parallel gripper
Camera – RGB-D camera
Tray – Red, Blue, Green (assume square shape tray)
Food-package – Red Blue, Green in Square shape
Simulator - Gazebo Fortress (`ign`, default) 
Planning - MoveIt 2 (OMPL RRTConnect for free motion, 
		Cartesian paths for approach/lift/place)
Perception - Overhead RGB-D camera, 
		OpenCV HSV segmentation + depth, TF2 camera→world



Architecture
 







Packages and Roles

Package	Role
cafe_interfaces	TrayDetection(.Array) msgs, PickPlace action (typed error codes)
cafe_sim	URDF/xacro (UR30 + gripper + ros2_control), Gazebo world, controllers, sim.launch.py, randomize_trays.py
cafe_moveit_config	SRDF, KDL kinematics, OMPL, joint limits, controller mapping
tray_detector	Perception node. core.py is ROS-free and unit-tested
pick_place_executor	Pick/place action server: planning, grasp verification, recovery
task_manager	Detect → prioritise → execute → retry/skip → idle
cafe_bringup	One parameter file, XML launch files, RViz config



Setup
	Run this in terminal
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-moveit ros-humble-ur-description \
  ros-humble-ros-ign ros-humble-ign-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-xacro ros-humble-cv-bridge \
  ros-humble-message-filters ros-humble-tf2-ros-py python3-opencv python3-colcon-common-extensions

	UR30 must exist in your ur_description:
ls /opt/ros/humble/share/ur_description/config/ur30 || \
  git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ~/cafe_ur30_ws/src/ur_description_src

	Open & extract this zip file- Souce it & then build this package
cd ~/cafe_ur30_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

Run the project

	Gazebo + RViz + everything (~20 s to come up)
	ros2 launch cafe_bringup cafe.launch.xml   

	headless Gazebo         
	ros2 launch cafe_bringup cafe.launch.xml gui:=false 


Checking different scenario

	new random layout 
	ros2 run cafe_sim randomize_trays.py --seed 4 

	move a tray (even mid-pick) -> dynamic update 
ros2 run cafe_sim randomize_trays.py --move red   

	tray disappears -> idle / re-scan   
	ros2 run cafe_sim randomize_trays.py --remove blue  

	force failures -> retry -> skip
	ros2 param set /pick_place_executor simulate_grasp_failure true   
	ros2 param set /pick_place_executor simulate_grasp_failure false

	clear skips
	ros2 service call /task_manager/reset std_srvs/srv/Trigger       

	restart node to apply- nearest-first
	ros2 param set /task_manager priority_enabled false  
	check
	ros2 topic echo /task_manager/stats

Note – Due to time constraint – containerized this whole application is still in process
