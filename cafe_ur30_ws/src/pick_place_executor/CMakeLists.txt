cmake_minimum_required(VERSION 3.8)
project(pick_place_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cafe_interfaces REQUIRED)

add_executable(pick_place_executor src/pick_place_executor.cpp)
ament_target_dependencies(pick_place_executor
  rclcpp rclcpp_action moveit_ros_planning_interface moveit_core geometric_shapes moveit_msgs
  control_msgs sensor_msgs geometry_msgs shape_msgs std_srvs tf2 trajectory_msgs cafe_interfaces)

install(TARGETS pick_place_executor DESTINATION lib/${PROJECT_NAME})
ament_package()
