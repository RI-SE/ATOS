import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join( # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
from launch_ros.actions import Node
from launch import LaunchDescription
import launch_utils.launch_base as launch_base


def get_experimental_nodes():
    files = launch_base.get_files()
    return [
        Node(
            package='atos',
            namespace='atos',
            executable='trajectorylet_streamer',
            name='trajectorylet_streamer',
            parameters=[files["params"]]
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            # prefix=['gdbserver localhost:3000'], ## To use with VSC debugger
            parameters=[files["params"]],
            # arguments=['--ros-args', '--log-level', "debug"] # To get RCL_DEBUG prints
        ),
        # Node(
        #     package='atos',
        #     namespace='atos',
        #     executable='osi_adapter',
        #     name='osi_adapter',
        #     parameters=[files["params"]]
        # ),
        Node(
            package='atos',
            namespace='atos',
            executable='back_to_start',
            name='back_to_start',
        ),
        Node(
            package='gui',
            namespace='atos',
            executable='gui_node',
            name='ros_gui',
            output='screen',
        )
    ]

def generate_launch_description():
    base_nodes = launch_base.get_base_nodes()
    
    experimental_nodes = get_experimental_nodes()

    for node in experimental_nodes:
        base_nodes.append(node)
        
    return LaunchDescription(base_nodes)
