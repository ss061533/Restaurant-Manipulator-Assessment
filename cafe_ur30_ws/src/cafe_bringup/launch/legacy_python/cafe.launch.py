"""Whole application: simulation + perception + MoveIt + executor + task manager (+ RViz).

  ros2 launch cafe_bringup cafe.launch.py                # everything
  ros2 launch cafe_bringup cafe.launch.py gui:=false     # headless Gazebo
  ros2 launch cafe_bringup cafe.launch.py autostart:=false   # do not start picking automatically
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('cafe_sim')
    bringup = get_package_share_directory('cafe_bringup')
    params = os.path.join(bringup, 'config', 'cafe_params.yaml')
    sim_time = {'use_sim_time': True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'legacy_python', 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    detector = Node(package='tray_detector', executable='tray_detector_node',
                    name='tray_detector', output='screen', parameters=[params, sim_time])

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'legacy_python', 'moveit.launch.py')))

    task_manager = Node(
        package='task_manager', executable='task_manager_node', name='task_manager',
        output='screen',
        parameters=[params, sim_time, {'autostart': LaunchConfiguration('autostart')}])

    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(bringup, 'rviz', 'cafe.rviz')],
                parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Task manager starts picking immediately'),
        sim,
        # stagger start-up so that controllers / clock exist before the dependants come up
        TimerAction(period=6.0, actions=[detector, rviz]),
        TimerAction(period=14.0, actions=[moveit]),
        TimerAction(period=18.0, actions=[task_manager]),
    ])
