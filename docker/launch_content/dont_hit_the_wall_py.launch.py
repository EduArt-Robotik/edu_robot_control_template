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
  dont_hit_the_wall = Node(
    package='edu_robot_control_template',
    executable='dont_hit_the_wall.py',
    name='dont_hit_the_wall',
    namespace=edu_robot_namespace,
    output='screen'
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    dont_hit_the_wall
  ])
