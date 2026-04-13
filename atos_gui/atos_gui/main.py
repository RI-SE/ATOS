"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

import sys
import threading
from pathlib import Path

import nicegui
import rclpy
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from atos_gui.configpanel.configpanel import ConfigPanelNode
from atos_gui.controlpanel.controlpanel import ControlPanelNode
from atos_gui.objectpanel.objectpanel import ObjectPanelNode

USE_SSL = len(sys.argv) > 1 and sys.argv[1] == "True"
FLEET_MODE = len(sys.argv) > 2 and sys.argv[2].lower() == "atosfleetmanagement"


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py,
    # but it's empty to enable NiceGUI auto-reloading.
    pass


def render_atosfleetmanagement_pages() -> None:
    nicegui.ui.link("ATOSFleetManagement Home", "/")

    @ui.page(path="/", title="TruckObjectGUI")
    def render_home() -> None:
        ui.label("TruckObjectGUI (ATOSFleetManagement mode)").classes("text-h4")
        ui.markdown(
            """
This GUI is running in **ATOSFleetManagement mode**.

Active runtime components:
- `truck_object_control`
- `foxglove_bridge` / `rosbridge`

Expected COT topic:
- `/atos/truck_objects/cot`

Speed command topic:
- `/atos/truck_objects/speed_command`

Placeholder COT payload format:
- `id=<truck_id>;distance_m=<value>;tcp_connected=<0|1>`
            """.strip()
        )


def ros_main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []

    if FLEET_MODE:
        render_atosfleetmanagement_pages()
    else:
        nicegui.ui.link("Control Panel", "/control")
        nicegui.ui.link("Config Panel", "/config")
        nicegui.ui.link("Object Panel", "/object")

        control_panel = ControlPanelNode()
        config_panel = ConfigPanelNode()
        object_panel = ObjectPanelNode()
        nodes = [control_panel, config_panel, object_panel]

        for node in nodes:
            executor.add_node(node)

    try:
        if nodes:
            executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def print_access_hint() -> None:
    scheme = "https" if USE_SSL else "http"
    print(f"TruckObjectGUI ready. Open {scheme}://localhost:3000", flush=True)


# Start the ROS node logic in a thread managed by nicegui.
app.on_startup(lambda: threading.Thread(target=ros_main, daemon=True).start())
app.on_startup(print_access_hint)

ui_run.APP_IMPORT_STRING = f"{__name__}:app"  # ROS2 uses non-standard module naming.

uvicorn_args = {
    "uvicorn_reload_dirs": str(Path(__file__).parent.resolve()),
    "port": 3000,
    "show": False,
    "title": "TruckObjectGUI" if FLEET_MODE else "ATOS GUI",
}

if USE_SSL:
    uvicorn_args["ssl_keyfile"] = Path.home() / ".astazero/ATOS/certs/selfsigned.key"
    uvicorn_args["ssl_certfile"] = Path.home() / ".astazero/ATOS/certs/selfsigned.crt"

ui.run(**uvicorn_args)

if USE_SSL:
    print("ATTENTION: Using SSL, use https://localhost:3000 to access the GUI instead", flush=True)
