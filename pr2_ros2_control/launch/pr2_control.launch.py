"""Bring up the PR2 motor chain under ros2_control (read-only milestone).

Starts robot_state_publisher with the IAI PR2 description + <ros2_control> block, the realtime
controller manager (pr2_control_node) and the joint_state_broadcaster.

pr2_control_node needs file capabilities after every build:
  sudo setcap cap_net_raw,cap_ipc_lock,cap_sys_nice+ep \
    $(readlink -f install/pr2_ros2_control/lib/pr2_ros2_control/pr2_control_node)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    interface = LaunchConfiguration('ethercat_interface')
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([FindPackageShare('pr2_ros2_control'), 'urdf', 'pr2.urdf.xacro']),
            ' ethercat_interface:=', interface,
        ]),
        value_type=str)
    controllers = PathJoinSubstitution([FindPackageShare('pr2_ros2_control'), 'config', 'controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('ethercat_interface', default_value='ecat0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='pr2_ros2_control',
            executable='pr2_control_node',
            parameters=[controllers],
            output='screen'),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                       '--param-file', controllers],
            output='screen'),
    ])
