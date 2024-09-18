import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join( # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
import launch_utils.launch_base as launch_base
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_graphical import get_graphical_nodes
from launch_experimental import get_experimental_nodes


def generate_launch_description():
    files = launch_base.get_files()
    base_nodes = launch_base.get_base_nodes()
    graphical_nodes = get_graphical_nodes()
    experimental_nodes = get_experimental_nodes()

    for node in graphical_nodes: 
      base_nodes.append(node)
		
    for node in experimental_nodes:
      base_nodes.append(node)

    return LaunchDescription(base_nodes)
