from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_weeble',
            executable='motor',
            name='motor'
        ),
        Node(
            package='controller',
            executable='motor_publisher',
            name='motor_publisher'
        )
    ])
