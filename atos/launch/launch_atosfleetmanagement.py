import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Need to modify sys.path since we launch from the ros2 installed path.
sys.path.insert(0, os.path.join(get_package_prefix("atos"), "share", "atos", "launch"))


def get_pythonpath_setup_action():
    venv_site_packages = Path.home() / ".local" / "share" / "atos" / "venv" / "lib" / (
        f"python{os.sys.version_info.major}.{os.sys.version_info.minor}"
    ) / "site-packages"
    if not venv_site_packages.exists():
        return None

    return SetEnvironmentVariable(
        name="PYTHONPATH",
        value=[
            str(venv_site_packages),
            os.pathsep,
            EnvironmentVariable("PYTHONPATH", default_value=""),
        ],
    )


def get_default_trajectory_path():
    return str(
        Path(get_package_prefix("atos"))
        / "etc"
        / "conf"
        / "RuralRoad_center_of_driving_lane_ccw.geojson"
    )


def generate_launch_description():
    insecure_websockets = LaunchConfiguration("insecure")
    foxbridge = LaunchConfiguration("foxbridge")
    with_simulator = LaunchConfiguration("with_truck_simulator")
    cot_tls_require_client_cert = LaunchConfiguration("cot_tls_require_client_cert")
    cot_tls_cert_path = LaunchConfiguration("cot_tls_cert_path")
    cot_tls_key_path = LaunchConfiguration("cot_tls_key_path")
    cot_tls_ca_path = LaunchConfiguration("cot_tls_ca_path")

    insecure_launch_arg = DeclareLaunchArgument("insecure", default_value="False")
    foxbridge_launch_arg = DeclareLaunchArgument("foxbridge", default_value="True")
    simulator_launch_arg = DeclareLaunchArgument(
        "with_truck_simulator", default_value="False"
    )
    cot_tls_require_client_cert_launch_arg = DeclareLaunchArgument(
        "cot_tls_require_client_cert", default_value="False"
    )
    cot_tls_cert_path_launch_arg = DeclareLaunchArgument(
        "cot_tls_cert_path", default_value=""
    )
    cot_tls_key_path_launch_arg = DeclareLaunchArgument(
        "cot_tls_key_path", default_value=""
    )
    cot_tls_ca_path_launch_arg = DeclareLaunchArgument(
        "cot_tls_ca_path", default_value=""
    )
    default_trajectory_path = get_default_trajectory_path()

    fox_tls_bridge_params = [
        {"port": 8765},
        {"retry_startup_delay": 5.0},
        {"tls": True},
        {"fragment_timeout": 600},
        {"max_message_size": 10000000},
        {"unregister_timeout": 10.0},
        {"use_compression": False},
    ]
    ros_tls_bridge_params = [
        {"port": 9090},
        {"retry_startup_delay": 5.0},
        {"tls": True},
        {"fragment_timeout": 600},
        {"max_message_size": 10000000},
        {"unregister_timeout": 10.0},
        {"use_compression": False},
    ]

    fox_bridge_params = [dict(item) for item in fox_tls_bridge_params]
    fox_bridge_params[2] = {"tls": False}
    ros_bridge_params = [dict(item) for item in ros_tls_bridge_params]
    ros_bridge_params[1] = {"retry_startup_delay": 5.0}
    ros_bridge_params[2] = {"tls": False}

    actions = [
            foxbridge_launch_arg,
            insecure_launch_arg,
            simulator_launch_arg,
            cot_tls_require_client_cert_launch_arg,
            cot_tls_cert_path_launch_arg,
            cot_tls_key_path_launch_arg,
            cot_tls_ca_path_launch_arg,
            Node(
                condition=IfCondition(
                    PythonExpression(["not ", LaunchConfiguration("insecure")])
                ),
                package="atos_gui",
                namespace="atos",
                executable="truck_object_gui",
                name="truck_object_gui",
                output="screen",
                arguments=["True", "atosfleetmanagement"],
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("insecure")),
                package="atos_gui",
                namespace="atos",
                executable="truck_object_gui",
                name="truck_object_gui",
                output="screen",
                arguments=["False", "atosfleetmanagement"],
            ),
            Node(
                package="atos",
                namespace="atos",
                executable="truck_object_control",
                name="truck_object_control",
                output="screen",
                parameters=[
                    {"cot_tls_require_client_cert": cot_tls_require_client_cert},
                    {"cot_tls_cert_path": cot_tls_cert_path},
                    {"cot_tls_key_path": cot_tls_key_path},
                    {"cot_tls_ca_path": cot_tls_ca_path},
                    {"trajectory_geojson_path": default_trajectory_path},
                ],
            ),
            Node(
                condition=IfCondition(with_simulator),
                package="atos",
                namespace="atos",
                executable="atos_truck_simulator",
                name="atos_truck_simulator_1",
                output="screen",
                parameters=[
                    {"uid": "L5S-TRUCK-SIM-1"},
                    {"start_index": 0},
                    {"target_speed_kmh": 80.0},
                    {"acceleration_mps2": 2.0},
                    {"ignore_warning_speed_commands": True},
                    {"trajectory_geojson_path": default_trajectory_path},
                ],
            ),
            Node(
                condition=IfCondition(with_simulator),
                package="atos",
                namespace="atos",
                executable="atos_truck_simulator",
                name="atos_truck_simulator_2",
                output="screen",
                parameters=[
                    {"uid": "L5S-TRUCK-SIM-2"},
                    {"start_index": 250},
                    {"target_speed_kmh": 40.0},
                    {"acceleration_mps2": 2.0},
                    {"trajectory_geojson_path": default_trajectory_path},
                ],
            ),
            Node(
                condition=IfCondition(with_simulator),
                package="atos",
                namespace="atos",
                executable="atos_truck_simulator",
                name="atos_truck_simulator_3",
                output="screen",
                parameters=[
                    {"uid": "L5S-TRUCK-SIM-3"},
                    {"start_index": 500},
                    {"target_speed_kmh": 40.0},
                    {"acceleration_mps2": 2.0},
                    {"trajectory_geojson_path": default_trajectory_path},
                ],
            ),
            Node(
                condition=IfCondition(PythonExpression(["not ", foxbridge])),
                name="rosapi",
                package="rosapi",
                executable="rosapi_node",
            ),
            Node(
                condition=IfCondition(
                    PythonExpression([insecure_websockets, " and ", foxbridge])
                ),
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output={"both": "log"},
                parameters=fox_bridge_params,
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["not ", insecure_websockets, " and ", foxbridge])
                ),
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output={"both": "log"},
                parameters=fox_tls_bridge_params,
            ),
            Node(
                condition=IfCondition(
                    PythonExpression([insecure_websockets, " and not ", foxbridge])
                ),
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="ros_bridge",
                output={"both": "log"},
                parameters=ros_bridge_params,
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(
                        ["not ", insecure_websockets, " and not ", foxbridge]
                    )
                ),
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="ros_bridge",
                output={"both": "log"},
                parameters=ros_tls_bridge_params,
            ),
        ]

    pythonpath_setup = get_pythonpath_setup_action()
    if pythonpath_setup is not None:
        actions.insert(7, pythonpath_setup)

    return LaunchDescription(actions)
