# Don't launch this file directly, rather use the launch files one level up instead
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from .validate_files import validate_atos_dir
import rclpy.logging as logging
import copy

def print_version():
    atos_install_dir = get_package_prefix('atos')
    version_file = open(atos_install_dir / Path(".VERSION"), 'r')
    logging.get_logger('launch').info("ATOS version: " + version_file.read())

def get_files():
    return validate_atos_dir()

def get_base_nodes():
    files = get_files()

    insecure_websockets = LaunchConfiguration('insecure')
    foxbridge = LaunchConfiguration('foxbridge')

    insecure_launch_arg = DeclareLaunchArgument('insecure', default_value='False')
    foxbridge_launch_arg = DeclareLaunchArgument('foxbridge', default_value='True')

    # Parameters for the various bridges
    fox_tls_bridge_params = [{"port": 8765},
                {"retry_startup_delay": 5.0},
                {"tls": True},
                {"certfile": str(files["cert"])},
                {"keyfile": str(files["key"])},
                {"fragment_timeout": 600},
                {"max_message_size": 10000000},
                {"unregister_timeout": 10.0},
                {"use_compression": False}]
    ros_tls_bridge_params = copy.deepcopy(fox_tls_bridge_params)
    ros_tls_bridge_params[0] = {"port": 9090}
    fox_bridge_params = copy.deepcopy(fox_tls_bridge_params)
    fox_bridge_params[2] = {"tls": False}
    ros_bridge_params = copy.deepcopy(fox_bridge_params)
    ros_bridge_params[0] = {"port": 9090}

    return [
        foxbridge_launch_arg,
        insecure_launch_arg,
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('atos_gui'),
                'launch/gui.py'))
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='object_control',
            parameters=[files["params"]]
            # ,prefix="xterm -e gdb --args" #Useful for debugging
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='scenario_module',
            name='scenario_module',
            parameters=[files["params"]]
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='journal_control',
            name='journal_control',
            parameters=[files["params"]]
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='esmini_adapter',
            name='esmini_adapter',
            parameters=[files["params"]]
        ),
        Node(
            condition=IfCondition(PythonExpression(['not ', foxbridge])),
            name='rosapi',
            package='rosapi',
            executable='rosapi_node'
        ),
        Node(
            condition=IfCondition(PythonExpression([insecure_websockets, ' and ', foxbridge])),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output={'both': 'log'}, #print to log to avoid cluttering the terminal
            parameters=fox_bridge_params
        ),
        Node(
            condition=IfCondition(PythonExpression(['not ', insecure_websockets, ' and ', foxbridge])),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output={'both': 'log'}, #print to log to avoid cluttering the terminal
            parameters=fox_tls_bridge_params
        ),
        Node(
            condition=IfCondition(PythonExpression([insecure_websockets, ' and not ', foxbridge])),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ros_bridge',
            output={'both': 'log'}, #print to log to avoid cluttering the terminal
            parameters=ros_bridge_params
        ),
        Node(
            condition=IfCondition(PythonExpression(['not ', insecure_websockets, ' and not ', foxbridge])),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ros_bridge',
            output={'both': 'log'}, #print to log to avoid cluttering the terminal
            parameters=ros_tls_bridge_params
        ),
        
    ]
