from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui',
<<<<<<< HEAD
            executable='gui_node',
            name='object_panel',
=======
            namespace='atos',
            executable='gui_node',
            name='ros_gui',
>>>>>>> feature/configpanel_python
            output='screen',
        )
    ])