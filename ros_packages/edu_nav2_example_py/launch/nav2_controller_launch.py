from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edu_nav2_example_py',
            executable='nav2_py_controller',
            namespace='eduard/simulation',
            output='screen'
        )
    ])
