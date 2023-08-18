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
    files = launch_base.get_files()
    base_nodes = launch_base.get_base_nodes()
    graphical_nodes = [
        Node(
            package='rviz2',
            namespace='atos',
            executable='rviz2',
            name='rviz2'
        ), 
        Node(
            package='atos',
            namespace='atos',
            executable='pointcloud_publisher',
            name='pointcloud_publisher',
            parameters=[files["params"]]
        )
    ]
    
    for node in graphical_nodes: # I don't know why base_nodes.append(graphical_nodes) doesn't work, but this does
        base_nodes.append(node)

    return LaunchDescription(base_nodes)
