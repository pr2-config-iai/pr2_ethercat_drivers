"""Bring up the PR2 motor chain under ros2_control (read-only milestone).

Starts robot_state_publisher with the IAI PR2 description + <ros2_control> block, the realtime
controller manager (pr2_control_node) and the joint_state_broadcaster.

pr2_control_node is started through pr2-rt, which grants it the realtime capabilities (raw
socket, mlockall, SCHED_FIFO). Install pr2-rt once per machine/container:
  src/pr2_ethercat_drivers/pr2_ros2_control/tools/install_pr2_rt.sh
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    interface = LaunchConfiguration('ethercat_interface')
    rt_prefix = LaunchConfiguration('rt_prefix')
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
        DeclareLaunchArgument('rt_prefix', default_value='pr2-rt',
                              description='Launch prefix that grants the realtime capabilities'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='pr2_ros2_control',
            executable='pr2_control_node',
            prefix=rt_prefix,
            parameters=[controllers],
            output='screen'),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                       '--param-file', controllers],
            output='screen'),
    ])
