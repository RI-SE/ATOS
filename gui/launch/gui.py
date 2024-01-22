from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    insecure_launch_arg = DeclareLaunchArgument('insecure', default_value='False')

    return LaunchDescription([
        insecure_launch_arg,
        Node(
            condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('insecure')])),
            package='atos_gui',
            namespace='atos',
            executable='gui',
            output='screen',
            arguments = ["True"] # Use ssl
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('insecure')),
            package='atos_gui',
            namespace='atos',
            executable='gui',
            output='screen',
            arguments = ["False"] # Don't use ssl
        )
    ])