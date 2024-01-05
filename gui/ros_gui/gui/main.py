import threading
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException

from nicegui import Client, app, ui, ui_run, APIRouter

from .configpanel import ConfigPanelNode

def ros_main() -> None:
    rclpy.init()
    node = ConfigPanelNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass



app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3000, title='Config Panel')