import threading
from pathlib import Path
from atos_interfaces.srv import *

import rclpy
from rclpy.executors import ExternalShutdownException
from .controlpanel.controlpanel import ControlPanelNode
from .configpanel.configpanel import ConfigPanelNode

from nicegui import app, ui, ui_run

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    ui.link('Control Panel', '/control')
    ui.link('Config Panel', '/config')
    rclpy.init()
    control_panel = ControlPanelNode()
    config_panel = ConfigPanelNode()
    try:
        rclpy.spin(control_panel)
        rclpy.spin(config_panel)
    except ExternalShutdownException:
        pass

#Starting the ros node in a thread managed by nicegui. It will restarted with "on_startup" after a reload.
#It has to be in a thread, since NiceGUI wants the main thread for itself.
app.on_startup(lambda: threading.Thread(target=ros_main).start())

ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3000)