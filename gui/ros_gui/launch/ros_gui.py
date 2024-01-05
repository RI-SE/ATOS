from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui',
            namespace='atos',
            executable='config_panel',
            name='config_panel',
            output='screen',
        ),
        Node(
            package='gui',
            namespace='atos',
            executable='control_panel',
            name='control_panel',
            output='screen',
        )
    ])