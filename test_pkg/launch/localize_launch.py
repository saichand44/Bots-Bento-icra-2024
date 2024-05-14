from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # robot_localization EKF Node
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='ekf_localization_node',
            output='screen',
            parameters=['path/to/your_package_name/config/ekf.yaml']
        )
    ])
