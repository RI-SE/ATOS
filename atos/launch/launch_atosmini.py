import os
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Need to modify sys.path since we launch from the ros2 installed path.
sys.path.insert(0, os.path.join(get_package_prefix('atos'), 'share', 'atos', 'launch'))


def generate_launch_description():
    insecure_websockets = LaunchConfiguration('insecure')
    foxbridge = LaunchConfiguration('foxbridge')

    insecure_launch_arg = DeclareLaunchArgument('insecure', default_value='False')
    foxbridge_launch_arg = DeclareLaunchArgument('foxbridge', default_value='True')

    fox_tls_bridge_params = [
        {'port': 8765},
        {'retry_startup_delay': 5.0},
        {'tls': True},
        {'fragment_timeout': 600},
        {'max_message_size': 10000000},
        {'unregister_timeout': 10.0},
        {'use_compression': False},
    ]
    ros_tls_bridge_params = [
        {'port': 9090},
        {'retry_startup_delay': 5.0},
        {'tls': True},
        {'fragment_timeout': 600},
        {'max_message_size': 10000000},
        {'unregister_timeout': 10.0},
        {'use_compression': False},
    ]

    fox_bridge_params = [dict(item) for item in fox_tls_bridge_params]
    fox_bridge_params[2] = {'tls': False}
    ros_bridge_params = [dict(item) for item in ros_tls_bridge_params]
    ros_bridge_params[1] = {'retry_startup_delay': 5.0}
    ros_bridge_params[2] = {'tls': False}

    return LaunchDescription([
        foxbridge_launch_arg,
        insecure_launch_arg,
        Node(
            condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('insecure')])),
            package='atos_gui',
            namespace='atos',
            executable='truck_object_gui',
            name='truck_object_gui',
            output='screen',
            arguments=['True', 'atosmini'],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('insecure')),
            package='atos_gui',
            namespace='atos',
            executable='truck_object_gui',
            name='truck_object_gui',
            output='screen',
            arguments=['False', 'atosmini'],
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='truck_object_control',
            name='truck_object_control',
            output='screen',
        ),
        Node(
            condition=IfCondition(PythonExpression(['not ', foxbridge])),
            name='rosapi',
            package='rosapi',
            executable='rosapi_node',
        ),
        Node(
            condition=IfCondition(PythonExpression([insecure_websockets, ' and ', foxbridge])),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output={'both': 'log'},
            parameters=fox_bridge_params,
        ),
        Node(
            condition=IfCondition(PythonExpression(['not ', insecure_websockets, ' and ', foxbridge])),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output={'both': 'log'},
            parameters=fox_tls_bridge_params,
        ),
        Node(
            condition=IfCondition(PythonExpression([insecure_websockets, ' and not ', foxbridge])),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ros_bridge',
            output={'both': 'log'},
            parameters=ros_bridge_params,
        ),
        Node(
            condition=IfCondition(PythonExpression(['not ', insecure_websockets, ' and not ', foxbridge])),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ros_bridge',
            output={'both': 'log'},
            parameters=ros_tls_bridge_params,
        ),
    ])
