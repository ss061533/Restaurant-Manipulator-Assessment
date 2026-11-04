"""Gazebo (Ignition Fortress) simulation: world, UR30 + gripper, controllers, camera bridge, TF."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cafe_sim = get_package_share_directory('cafe_sim')
    ur_share = get_package_share_directory('ur_description')
    ros_ign = get_package_share_directory('ros_ign_gazebo')

    world = os.path.join(cafe_sim, 'worlds', 'cafe.sdf')
    xacro_file = os.path.join(cafe_sim, 'urdf', 'cafe_ur30.urdf.xacro')
    gui = LaunchConfiguration('gui')

    # Let Gazebo resolve package://ur_description/... meshes and find the ros2_control plugin.
    res_path = os.pathsep.join(
        [os.path.dirname(ur_share), os.path.dirname(cafe_sim),
         os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    plugin_path = os.pathsep.join(
        ['/opt/ros/humble/lib', os.environ.get('LD_LIBRARY_PATH', ''),
         os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_ign, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': [PythonExpression(
                ["'-r -v 2 ' if '", gui, "' == 'true' else '-r -s -v 2 '"]), world],
            'on_exit_shutdown': 'true',
        }.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='ros_ign_gazebo', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'ur30',
                            '-allow_renaming', 'true'])

    bridge = Node(package='ros_ign_bridge', executable='parameter_bridge', output='screen',
                  arguments=[
                      '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                      '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                      '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                      '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                  ])

    # Camera extrinsics: same pose as the <model overhead_camera> in the world file.
    # Optical frame (z forward/down, x = image right = world -Y, y = image down = world -X):
    # quaternion = rotation by pi about (1,-1,0)/sqrt2.
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.75', '--y', '0.0', '--z', '1.75',
                   '--qx', '0.7071068', '--qy', '-0.7071068', '--qz', '0.0', '--qw', '0.0',
                   '--frame-id', 'world', '--child-frame-id', 'camera_optical_frame'],
        parameters=[{'use_sim_time': True}])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '120'])

    jsb, arm, gripper = (spawner('joint_state_broadcaster'), spawner('joint_trajectory_controller'),
                         spawner('gripper_controller'))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo GUI'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', res_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_path),
        gz, rsp, bridge, camera_tf, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, gripper])),
    ])
