import os
import sys

from ament_index_python.packages import get_package_prefix

sys.path.insert(
    0,
    os.path.join(  # Need to modify the sys.path since we launch from the ros2 installed path
        get_package_prefix("atos"), "share", "atos", "launch"
    ),
)

import launch_utils.launch_base as launch_base
from launch import LaunchDescription
from launch_ros.actions import Node


def get_mqtt_nodes():
    files = launch_base.get_files()
    return [
        Node(
            package="atos",
            namespace="atos",
            executable="mqtt_bridge",
            name="mqtt_bridge",
            # prefix=['gdbserver localhost:3000'], ## To use with VSC debugger
            parameters=[files["params"]],
            # arguments=["--ros-args", "--log-level", "debug"],  # To get RCL_DEBUG prints
        ),
    ]


def generate_launch_description():
    base_nodes = launch_base.get_base_nodes()

    mqtt_nodes = get_mqtt_nodes()

    for node in mqtt_nodes:
        base_nodes.append(node)

    return LaunchDescription(base_nodes)
