from launch_ros.actions import Node
from launch import LaunchDescription
import launch_utils.launch_base as launch_base
import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join(  # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))


def generate_launch_description():
    base_nodes = launch_base.get_base_nodes()
    base_nodes.append(
        Node(
            package='atos',
            namespace='atos',
            executable='trajectorylet_streamer',
            name='trajectorylet_streamer'
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
        Node(
            package='atos',
            namespace='atos',
            executable='osi_adapter',
            name='osi_adapter',
            parameters=[files["params"]]
        ))
    return LaunchDescription(base_nodes)
