# Don't launch this file directly, rather use the launch files one level up instead
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
import subprocess
from pathlib import Path
from .validate_files import validate_files

def get_base_nodes():
    files = validate_files()
    atos_conf_dir = os.path.join(os.path.expanduser('~'), '.astazero', 'ATOS')
    atos_install_dir = get_package_prefix('atos')

    # control-gui logging
    control_gui_log = open(atos_conf_dir / Path("webgui.log"), 'w')
    # start control-gui server
    control_gui_dir = Path(atos_install_dir) / Path("simplecontrol/")
    subprocess.Popen("/usr/bin/npm start --prefix " + str(control_gui_dir),shell=True, stdout=control_gui_log, stderr=control_gui_log)

    return [
        Node(
            package='atos',
            namespace='atos',
            executable='atos_base',
            name='atos_base',
            parameters=[files["params"]]
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ros_bridge',
            parameters=[
                {"port": 9090},
                {"retry_startup_delay": 5.0},
                {"certfile": str(files["cert"])},
                {"keyfile": str(files["key"])},
                {"fragment_timeout": 600},
                {"max_message_size": 10000000},
                {"unregister_timeout": 10.0},
                {"use_compression": False}]
        ),
        Node(
            name='rosapi',
            package='rosapi',
            executable='rosapi_node'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='object_control',
            name='object_control',
            parameters=[files["params"]]
            # ,prefix="xterm -e gdb --args" #Useful for debugging
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='journal_control',
            name='journal_control'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='osi_adapter',
            name='osi_adapter',
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
            package='atos',
            namespace='atos',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            # prefix=['gdbserver localhost:3000'], ## To use with VSC debugger
            parameters=[files["params"]],
            # arguments=['--ros-args', '--log-level', "debug"] # To get RCL_DEBUG prints
        ),
    ]
