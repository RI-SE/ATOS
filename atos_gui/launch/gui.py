import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def get_pythonpath_setup_action():
    venv_site_packages = (
        Path.home()
        / ".local"
        / "share"
        / "atos"
        / "venv"
        / "lib"
        / (f"python{os.sys.version_info.major}.{os.sys.version_info.minor}")
        / "site-packages"
    )
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


def generate_launch_description():
    insecure_launch_arg = DeclareLaunchArgument("insecure", default_value="False")

    actions = [insecure_launch_arg]
    pythonpath_setup = get_pythonpath_setup_action()
    if pythonpath_setup is not None:
        actions.append(pythonpath_setup)

    actions.extend(
        [
            Node(
                condition=IfCondition(
                    PythonExpression(["not ", LaunchConfiguration("insecure")])
                ),
                package="atos_gui",
                namespace="atos",
                executable="gui",
                output="screen",
                arguments=["True"],  # Use ssl
            ),
            Node(
                condition=IfCondition(LaunchConfiguration("insecure")),
                package="atos_gui",
                namespace="atos",
                executable="gui",
                output="screen",
                arguments=["False"],  # Don't use ssl
            ),
        ]
    )

    return LaunchDescription(actions)
