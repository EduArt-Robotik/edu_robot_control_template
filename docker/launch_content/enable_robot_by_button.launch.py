import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  # Launch File Arguments
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', 'eduard'))

  # Bring Up Enable Robot By Button Node
  enable_robot_by_button = Node(
    package='edu_robot_control_template',
    executable=' enable-robot-by-button',
    name='enable_robot_by_button',
    namespace=edu_robot_namespace,
    output='screen'
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    enable_robot_by_button
  ])