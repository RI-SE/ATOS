import threading
from pathlib import Path
from atos_interfaces.srv import *

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from .controlpanel.controlpanel import ControlPanelNode
from .configpanel.configpanel import ConfigPanelNode
from .objectpanel.object_panel import ObjectPanelNode

from nicegui import app, ui, ui_run

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    ui.link('Control Panel', '/control')
    ui.link('Config Panel', '/config')
    ui.link('Object Panel', '/object')
    rclpy.init()
    control_panel = ControlPanelNode()
    config_panel = ConfigPanelNode()
    object_panel = ObjectPanelNode()
    executor = MultiThreadedExecutor()
    executor.add_node(control_panel)
    executor.add_node(config_panel)
    executor.add_node(object_panel)
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        executor.shutdown()
        control_panel.destroy_node()
        config_panel.destroy_node()
        object_panel.destroy_node()

#Starting the ros node in a thread managed by nicegui. It will restarted with "on_startup" after a reload.
#It has to be in a thread, since NiceGUI wants the main thread for itself.
app.on_startup(lambda: threading.Thread(target=ros_main).start())

ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3000, title="ATOS GUI")