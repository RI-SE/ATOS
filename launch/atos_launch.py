import os

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    atosConfig = os.path.join(
        get_package_prefix('atos'),
        'etc',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='atos',
            namespace='atos',
            executable='atos_base',
            name='atos_base'
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
            name='object_control'
            #,prefix="xterm -e gdb --args" #Useful for debugging
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='trajectorylet_streamer',
            name='trajectorylet_streamer'
        ),
<<<<<<< HEAD:launch/maestro_launch.py
        # Node(
        #    package='maestro',
        #    namespace='maestro',
=======
        #Node(
        #    package='atos',
        #    namespace='atos',
>>>>>>> 5abeb48dbeefebf3814b3c64d105ecde7e75c4ed:launch/atos_launch.py
        #    executable='time_control',
        #    name='time_control'
        # ),
        Node(
            package='atos',
            namespace='atos',
            executable='direct_control',
            name='direct_control'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='scenario_control',
            name='scenario_control'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='journal_control',
            name='journal_control'
        ),
<<<<<<< HEAD:launch/maestro_launch.py
        # Node(
        #     package='rviz2',
        #     namespace='maestro',
        #     executable='rviz2',
        #     name='rviz2'
        # ),
=======
        Node(
            package='rviz2',
            namespace='atos',
            executable='rviz2',
            name='rviz2'
        ),
>>>>>>> 5abeb48dbeefebf3814b3c64d105ecde7e75c4ed:launch/atos_launch.py
        Node(
            package='atos',
            namespace='atos',
            executable='osi_adapter',
            name='osi_adapter'
        ),
        Node(
            package='atos',
            namespace='atos',
            executable='esmini_adapter',
            name='esmini_adapter'
        ),
        Node(
<<<<<<< HEAD:launch/maestro_launch.py
            package='maestro',
            namespace='maestro',
            executable='mqtt_bridge',
            name='mqtt_bridge'
=======
            package='atos',
            namespace='atos',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            # prefix=['gdbserver localhost:3000'], ## To use with VSC debugger
            parameters=[atosConfig]
>>>>>>> 5abeb48dbeefebf3814b3c64d105ecde7e75c4ed:launch/atos_launch.py
        )
        #Node(
        #    package='atos',
        #    namespace='atos',
        #    executable='mqtt_bridge',
        #    name='mqtt_bridge',
        #    # prefix=['gdbserver localhost:3000'], ## To use with VSC debugger
        #    parameters=[atosConfig]
        #)
    ])
