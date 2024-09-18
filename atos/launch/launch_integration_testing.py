import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join( # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
from launch_ros.actions import Node
from launch import LaunchDescription
import launch_utils.launch_base as launch_base


def get_integration_test_nodes():
    files = launch_base.get_files()
    return [
        Node(
            package='atos',
            namespace='atos',
            executable='integration_testing_handler',
            parameters=[files["params"]]
        )
    ]

def generate_launch_description():
    base_nodes = launch_base.get_base_nodes()
    
    integration_test_nodes = get_integration_test_nodes()

    for node in integration_test_nodes:
        base_nodes.append(node)
        
    return LaunchDescription(base_nodes)
