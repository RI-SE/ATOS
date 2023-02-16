import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join( # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
import launch_utils.launch_base as launch_base
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():   
    base_nodes = launch_base.get_base_nodes()
    base_nodes.append([Node(
        package='rviz2',
        namespace='atos',
        executable='rviz2',
        name='rviz2'
    )])
    return LaunchDescription(base_nodes)
