from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare the path to the configuration file as a launch argument
        DeclareLaunchArgument(
            'config_file',
            default_value='path/to/your_package_name/config/apriltag_config.yaml',
            description='Path to the parameter configuration file'
        ),

        # AprilTag Detection Node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        ),

        # Pose Conversion Node
        Node(
            package='your_package_name',
            executable='tag_pose_publisher',
            name='tag_pose_publisher',
            output='screen'
        ),

        # Tag Marker Publisher
        Node(
            package='your_package_name',
            executable='tag_marker_publisher',
            name='tag_marker_publisher',
            output='screen'
        )
    ])
