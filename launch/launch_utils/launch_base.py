# Don't launch this file directly, rather use the launch files one level up instead
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import subprocess
from pathlib import Path
from .validate_files import validate_files
import rclpy.logging as logging

def print_version():
    atos_install_dir = get_package_prefix('atos')
    version_file = open(atos_install_dir / Path(".VERSION"), 'r')
    logging.get_logger('launch').info("ATOS version: " + version_file.read())

def get_files():
    return validate_files()

def get_base_nodes():
    files = get_files()
    atos_conf_dir = os.path.join(os.path.expanduser('~'), '.astazero', 'ATOS')
    atos_install_dir = get_package_prefix('atos')

    # control-gui logging
    control_gui_log = open(atos_conf_dir / Path("webgui.log"), 'w')
    # start control-gui server
    control_gui_dir = Path(atos_install_dir) / Path("controlpanel/")
    subprocess.Popen("npm start --prefix " + str(control_gui_dir),shell=True, stdout=control_gui_log, stderr=control_gui_log)

    insecure_websockets = LaunchConfiguration('insecure')
    foxbridge = LaunchConfiguration('foxbridge')

    insecure_launch_arg = DeclareLaunchArgument('insecure', default_value='False')
    foxbridge_launch_arg = DeclareLaunchArgument('foxbridge', default_value='True')

    return [
        foxbridge_launch_arg,
        insecure_launch_arg,
        Node(
            package='atos',
            namespace='atos',
            executable='atos_base',
            name='atos_base',
            parameters=[files["params"]]
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
            parameters=[
                {"port": 8765},
                {"retry_startup_delay": 5.0},
                {"fragment_timeout": 600},
                {"max_message_size": 10000000},
                {"unregister_timeout": 10.0},
                {"use_compression": False}]
        ),
        Node(
            condition=IfCondition(PythonExpression(['not ', insecure_websockets, ' and ', foxbridge])),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output={'both': 'log'}, #print to log to avoid cluttering the terminal
            parameters=[
                {"port": 8765},
                {"retry_startup_delay": 5.0},
                {"tls": True},
                {"certfile": str(files["cert"])},
                {"keyfile": str(files["key"])},
                {"fragment_timeout": 600},
                {"max_message_size": 10000000},
                {"unregister_timeout": 10.0},
                {"use_compression": False}]
        ),
        Node(
            condition=IfCondition(PythonExpression([insecure_websockets, ' and not ', foxbridge])),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ros_bridge',
            output={'both': 'log'}, #print to log to avoid cluttering the terminal
            parameters=[
                {"port": 9090},
                {"retry_startup_delay": 5.0},
                {"fragment_timeout": 600},
                {"max_message_size": 10000000},
                {"unregister_timeout": 10.0},
                {"use_compression": False}]
        ),
            Node(
            condition=IfCondition(PythonExpression(['not ', insecure_websockets, ' and not ', foxbridge])),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ros_bridge',
            output={'both': 'log'}, #print to log to avoid cluttering the terminal
            parameters=[
                {"port": 9090},
                {"retry_startup_delay": 5.0},
                {"certfile": str(files["cert"])},
                {"keyfile": str(files["key"])},
                {"fragment_timeout": 600},
                {"max_message_size": 10000000},
                {"unregister_timeout": 10.0},
                {"use_compression": False}]
        )
    ]
