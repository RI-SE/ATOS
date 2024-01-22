import threading
from pathlib import Path
from atos_interfaces.srv import *
import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from .controlpanel.controlpanel import ControlPanelNode
from .configpanel.configpanel import ConfigPanelNode

from nicegui import app, ui, ui_run

USE_SSL = sys.argv[1] == "True"

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    ui.link('Control Panel', '/control')
    ui.link('Config Panel', '/config')
    rclpy.init()
    control_panel = ControlPanelNode()
    config_panel = ConfigPanelNode()
    executor = MultiThreadedExecutor()
    executor.add_node(control_panel)
    executor.add_node(config_panel)
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        executor.shutdown()
        control_panel.destroy_node()
        config_panel.destroy_node()

#Starting the ros node in a thread managed by nicegui. It will be restarted with "on_startup" after a reload.
#It has to be in a thread, since NiceGUI wants the main thread for itself.
app.on_startup(lambda: threading.Thread(target=ros_main).start())

ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here

# Prepare the arguments for ui.run()
uvicorn_args = {
    'uvicorn_reload_dirs': str(Path(__file__).parent.resolve()),
    'favicon': '/images/favicon.ico',
    'port': 3000,
    'show': False # Disable auto-opening the browser
}

# If use_ssl is True, add the SSL arguments
if USE_SSL:
    uvicorn_args['ssl_keyfile'] = Path.home() / ".astazero/ATOS/certs/selfsigned.key"
    uvicorn_args['ssl_certfile'] = Path.home() / ".astazero/ATOS/certs/selfsigned.crt"

# Call ui.run() with the prepared arguments
ui.run(**uvicorn_args)

# If print is above ui.run(), it will be printed twice for some reason
if USE_SSL:
    print("ATTENTION: Using SSL, use https://localhost:3000 to access the GUI instead", flush=True)
