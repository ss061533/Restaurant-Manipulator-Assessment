"""MoveIt 2 move_group + the pick_place_executor (they share the same MoveIt parameters)."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def _load_text(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return f.read()


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('cafe_sim'), 'urdf',
                              'cafe_ur30.urdf.xacro')
    params_file = os.path.join(get_package_share_directory('cafe_bringup'), 'config',
                               'cafe_params.yaml')

    robot_description = {'robot_description': ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)}
    robot_description_semantic = {
        'robot_description_semantic': _load_text('cafe_moveit_config', 'config/cafe_ur30.srdf')}
    kinematics = {'robot_description_kinematics':
                  _load_yaml('cafe_moveit_config', 'config/kinematics.yaml')}
    joint_limits = {'robot_description_planning':
                    _load_yaml('cafe_moveit_config', 'config/joint_limits.yaml')}

    ompl = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': ' '.join([
            'default_planner_request_adapters/AddTimeOptimalParameterization',
            'default_planner_request_adapters/FixWorkspaceBounds',
            'default_planner_request_adapters/FixStartStateBounds',
            'default_planner_request_adapters/FixStartStateCollision',
            'default_planner_request_adapters/FixStartStatePathConstraints']),
        'start_state_max_bounds_error': 0.1}}
    ompl['move_group'].update(_load_yaml('cafe_moveit_config', 'config/ompl_planning.yaml'))

    controllers = {
        'moveit_simple_controller_manager':
            _load_yaml('cafe_moveit_config', 'config/moveit_controllers.yaml'),
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.05}

    scene_monitor = {
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True}

    sim_time = {'use_sim_time': True}

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[robot_description, robot_description_semantic, kinematics, joint_limits,
                    ompl, controllers, trajectory_execution, scene_monitor, sim_time])

    executor = Node(
        package='pick_place_executor', executable='pick_place_executor',
        name='pick_place_executor', output='screen',
        parameters=[robot_description, robot_description_semantic, kinematics, joint_limits, params_file,
                    sim_time])

    return LaunchDescription([move_group, executor])
