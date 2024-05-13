from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor',
            executable='relay.py',
            output='screen',
            emulate_tty=True,
        ),
    ])