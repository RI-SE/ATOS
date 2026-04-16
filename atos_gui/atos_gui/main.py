"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

import json
import sys
import threading
from pathlib import Path

import nicegui
import rclpy
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from atos_gui.configpanel.configpanel import ConfigPanelNode
from atos_gui.controlpanel.controlpanel import ControlPanelNode
from atos_gui.objectpanel.objectpanel import ObjectPanelNode

USE_SSL = len(sys.argv) > 1 and sys.argv[1] == "True"
FLEET_MODE = len(sys.argv) > 2 and sys.argv[2].lower() == "atosfleetmanagement"
GEOJSON_NAME = "RuralRoad_center_of_driving_lane_ccw.geojson"
FLEET_STATIC_ROUTE = "/atos_gui_static"
FLEET_STATIC_DIR = Path(__file__).parent / "static"
FLEET_STATE_LOCK = threading.Lock()
FLEET_TRUCK_STATES: dict[str, dict] = {}



def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py,
    # but it's empty to enable NiceGUI auto-reloading.
    pass


def _candidate_geojson_paths() -> list[Path]:
    candidates = [
        Path.home() / ".astazero/ATOS/conf" / GEOJSON_NAME,
        Path.home() / "atos_ws/src/atos/conf/conf" / GEOJSON_NAME,
        Path.home() / "Documents/repos/ATOS/conf/conf" / GEOJSON_NAME,
    ]

    try:
        from ament_index_python.packages import get_package_prefix

        atos_prefix = Path(get_package_prefix("atos"))
        candidates.append(atos_prefix / "etc/conf" / GEOJSON_NAME)
    except Exception:
        pass

    unique_candidates = []
    seen = set()
    for path in candidates:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        unique_candidates.append(path)

    return unique_candidates


def _load_fleet_geojson() -> tuple[dict | None, Path | None]:
    for path in _candidate_geojson_paths():
        if path.exists():
            try:
                return json.loads(path.read_text()), path
            except Exception:
                continue
    return None, None


def _fleet_snapshot_payload() -> str:
    with FLEET_STATE_LOCK:
        trucks = list(FLEET_TRUCK_STATES.values())
    return json.dumps(trucks).replace("</", "<\\/")


class FleetStateNode(Node):
    def __init__(self) -> None:
        super().__init__("truck_object_gui_state_bridge")
        self._state_sub = self.create_subscription(String, "truck_objects/state", self._on_state, 100)

    def _on_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warning("Failed to parse truck_objects/state payload as JSON")
            return

        uid = payload.get("uid")
        if not uid:
            return

        with FLEET_STATE_LOCK:
            FLEET_TRUCK_STATES[str(uid)] = payload


def render_atosfleetmanagement_pages() -> None:
    if FLEET_STATIC_DIR.exists():
        app.add_static_files(FLEET_STATIC_ROUTE, str(FLEET_STATIC_DIR))

    @ui.page(path="/", title="TruckObjectGUI")
    def render_home() -> None:
        ui.add_head_html(f'<script src="{FLEET_STATIC_ROUTE}/fleet_map.js"></script>')
        ui.label("TruckObjectGUI (ATOSFleetManagement mode)").classes("text-h4")

        with ui.tabs().classes("w-full") as tabs:
            home_tab = ui.tab("Overview", icon="dashboard")
            road_tab = ui.tab("RuralRoad Map", icon="map")

        with ui.tab_panels(tabs, value=home_tab).classes("w-full"):
            with ui.tab_panel(home_tab):
                ui.markdown(
                    """
This GUI is running in **ATOSFleetManagement mode**.

Active runtime components:
- `truck_object_control`
- `foxglove_bridge` / `rosbridge`

Expected COT topic:
- `/atos/truck_objects/cot`

Expected COT TCP endpoint:
- `tcp://0.0.0.0:8114` (TruckObjectControl listener)

Speed command topic:
- `/atos/truck_objects/speed_command`

Live truck state topic for map overlay:
- `/atos/truck_objects/state`
                    """.strip()
                )

            with ui.tab_panel(road_tab):
                geojson, source_path = _load_fleet_geojson()
                ui.label("RuralRoad centerline visualization").classes("text-h5")

                if not geojson:
                    ui.label("Could not load geojson file.").classes("text-red-600")
                    ui.markdown("Searched:\n" + "\n".join([f"- `{p}`" for p in _candidate_geojson_paths()]))
                    return

                ui.label(f"Source: {source_path}").classes("text-sm text-gray-600")
                map_id = "rural-road-map"
                ui.html(f'<div id="{map_id}" style="height:75vh;width:100%;border-radius:8px;"></div>')

                geojson_payload = json.dumps(geojson).replace("</", "<\\/")
                ui.timer(
                    0.2,
                    lambda: ui.run_javascript(
                        f"""
(() => {{
  const payload = {geojson_payload};
  const containerId = "{map_id}";
  let attempts = 0;
  const maxAttempts = 60;
  const timer = setInterval(() => {{
    attempts += 1;
    const hasFn = typeof window.renderFleetRoadMap === "function";
    const hasEl = !!document.getElementById(containerId);
    if (hasFn && hasEl) {{
      clearInterval(timer);
      window.renderFleetRoadMap(containerId, payload, {_fleet_snapshot_payload()});
      return;
    }}
    if (attempts >= maxAttempts) {{
      clearInterval(timer);
      const el = document.getElementById(containerId);
      if (el) {{
        el.innerHTML = "<div style='padding:12px;color:#b91c1c;font-weight:600;'>Map renderer not available. Check browser console.</div>";
      }}
    }}
  }}, 250);
}})();
                        """.strip()
                    ),
                    once=True,
                )
                ui.timer(
                    0.5,
                    lambda: ui.run_javascript(
                        f"window.updateFleetTruckStates('{map_id}', {_fleet_snapshot_payload()});"
                    ),
                )


def ros_main() -> None:
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []

    if FLEET_MODE:
        render_atosfleetmanagement_pages()
        nodes = [FleetStateNode()]
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
    print(f"TruckObjectGUI ready. Open {scheme}://localhost:8420", flush=True)


# Start the ROS node logic in a thread managed by nicegui.
app.on_startup(lambda: threading.Thread(target=ros_main, daemon=True).start())
app.on_startup(print_access_hint)

ui_run.APP_IMPORT_STRING = f"{__name__}:app"  # ROS2 uses non-standard module naming.

uvicorn_args = {
    "uvicorn_reload_dirs": str(Path(__file__).parent.resolve()),
    "host": "0.0.0.0",
    "port": 8420,
    "show": False,
    "title": "TruckObjectGUI" if FLEET_MODE else "ATOS GUI",
}

if USE_SSL:
    uvicorn_args["ssl_keyfile"] = Path.home() / ".astazero/ATOS/certs/selfsigned.key"
    uvicorn_args["ssl_certfile"] = Path.home() / ".astazero/ATOS/certs/selfsigned.crt"

ui.run(**uvicorn_args)

if USE_SSL:
    print("ATTENTION: Using SSL, use https://localhost:8420 to access the GUI instead", flush=True)
