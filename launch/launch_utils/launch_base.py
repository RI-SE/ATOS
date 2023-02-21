# Don't launch this file directly, rather use the launch files one level up instead
import os
from launch_ros.actions import Node

def get_base_nodes():
    atos_dir = os.path.join(os.path.expanduser('~'), '.astazero', 'ATOS')
    params = os.path.join(atos_dir, 'conf', 'params.yaml')

    return [
        Node(
            package='atos',
            namespace='atos',
            executable='atos_base',
            name='atos_base',
            parameters=[params]
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='system_control',
            name='system_control'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='object_control',
            name='object_control',
            parameters=[params]
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
            parameters=[params]
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='esmini_adapter',
            name='esmini_adapter',
            parameters=[params]
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            # prefix=['gdbserver localhost:3000'], ## To use with VSC debugger
            parameters=[params]
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='trajectorylet_streamer',
            name='trajectorylet_streamer',
            parameters=[params]
        ),
    ]
