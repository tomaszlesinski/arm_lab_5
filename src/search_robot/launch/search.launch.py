from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='search_robot',
            executable='detector',
            output='screen',
        ),
        Node(
            package='search_robot',
            executable='search_server',
            output='screen',
        ),
    ])
