import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join(
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
import launch_utils.launch_base as launch_base
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    files = launch_base.get_files()
    return LaunchDescription([
        Node(
            package="atos",
            namespace="atos",
            executable="cot_receiver",
            name="cot_receiver",
            parameters=[files["params"]],
        ),
    ])
