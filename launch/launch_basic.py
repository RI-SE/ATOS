import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join(  # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
import launch_utils.launch_base as launch_base
from launch import LaunchDescription

def generate_launch_description():
    base_nodes = launch_base.get_base_nodes()
    return LaunchDescription(base_nodes)
