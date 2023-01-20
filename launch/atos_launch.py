import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atos',
            namespace='atos',
            executable='atos_base',
            name='atos_base'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='system_control',
            name='system_control'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='object_control',
            name='object_control'
            #,prefix="xterm -e gdb --args" #Useful for debugging
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='trajectorylet_streamer',
            name='trajectorylet_streamer'
        ),
        #Node(
        #    package='atos',
        #    namespace='atos',
        #    executable='time_control',
        #    name='time_control'
        #),
        Node(
            package='atos',
            namespace='atos',
            executable='direct_control',
            name='direct_control'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='scenario_control',
            name='scenario_control'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='journal_control',
            name='journal_control'
        ),
        Node(
            package='rviz2',
            namespace='atos',
            executable='rviz2',
            name='rviz2'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='osi_adapter',
            name='osi_adapter'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='esmini_adapter',
            name='esmini_adapter'
        )
    ])
