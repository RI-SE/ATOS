from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import launch.substitutions

def generate_launch_description():
    DeclareLaunchArgument("use_ssl"),

    return LaunchDescription([
        Node(
            package='gui',
            namespace='atos',
            executable='gui',
            output='screen',
            arguments = [launch.substitutions.LaunchConfiguration('use_ssl')]
        )
    ])