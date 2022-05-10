import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maestro',
            namespace='maestro',
            executable='maestro_base',
            name='maestro_base'
        ),
        Node(
            package='maestro',
            namespace='maestro',
            executable='system_control',
            name='system_control'
        ),
        Node(
            package='maestro',
            namespace='maestro',
            executable='object_control',
            name='object_control'
        ),
        Node(
            package='maestro',
            namespace='maestro',
            executable='trajectorylet_streamer',
            name='trajectorylet_streamer'
        ),
        #Node(
        #    package='maestro',
        #    namespace='maestro',
        #    executable='time_control',
        #    name='time_control'
        #),
        Node(
            package='maestro',
            namespace='maestro',
            executable='direct_control',
            name='direct_control'
        ),
        Node(
            package='maestro',
            namespace='maestro',
            executable='scenario_control',
            name='scenario_control'
        ),
        Node(
            package='maestro',
            namespace='maestro',
            executable='journal_control',
            name='journal_control'
        ),
        Node(
            package='rviz2',
            namespace='maestro',
            executable='rviz2',
            name='rviz2'
        )
    ])
