"""This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""

import rclpy
from nicegui import ui
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import Empty, String

from atos_interfaces.srv import *

QOS = rclpy.qos.QoSProfile(depth=10)

OBC_STATES = {
    0: "UNDEFINED",
    1: "IDLE",
    2: "INITIALIZED",
    3: "CONNECTED",
    4: "ARMED",
    5: "DISARMING",
    6: "RUNNING",
    7: "REMOTECTRL",
    8: "ERROR",
    9: "ABORTING",
    10: "CLEARING",
}


class ControlPanelNode(Node):
    def __init__(self) -> None:
        super().__init__("control_panel")
        self.initPub = self.create_publisher(Empty, "/atos/init", QOS)
        self.connectPub = self.create_publisher(Empty, "/atos/connect", QOS)
        self.disconnectPub = self.create_publisher(Empty, "/atos/disconnect", QOS)
        self.armPub = self.create_publisher(Empty, "/atos/arm", QOS)
        self.disarmPub = self.create_publisher(Empty, "/atos/disarm", QOS)
        self.startPub = self.create_publisher(Empty, "/atos/start", QOS)
        self.abortPub = self.create_publisher(Empty, "/atos/abort", QOS)
        self.allClearPub = self.create_publisher(Empty, "/atos/all_clear", QOS)
        self.resetTestObjectsPub = self.create_publisher(
            Empty, "/atos/reset_test_objects", QOS
        )
        self.reloadObjectSettingsPub = self.create_publisher(
            Empty, "/atos/reload_object_settings", QOS
        )

        self.get_object_control_state_client = self.create_client(
            GetObjectControlState, "/atos/get_object_control_state"
        )
        self.OBC_state_req = GetObjectControlState.Request()
        self.get_object_control_state_timer = self.create_timer(
            0.5, self.get_object_control_state_callback
        )

        self.scenario_names = []
        self.selected_scenario = ""
        self._scenario_select = None
        self.active_scenario_pub = self.create_publisher(
            String, "/atos/active_scenario", QOS
        )
        self.get_scenario_names_client = self.create_client(
            GetParameters,
            "/atos/open_scenario_gateway/get_parameters",
        )

        self.OBC_state = {"state": "UNDEFINED"}
        self.lost_connection = True

        @ui.page(path="/control", title="ATOS Control Panel")
        def render_page():
            with ui.row().bind_visibility_from(self, "lost_connection"):
                ui.label("Lost connection to ATOS...").tailwind.text_color("red")
            with ui.row():
                ui.button(
                    "Abort",
                    on_click=lambda: self.abortPub.publish(Empty()),
                    color="red",
                ).props("size=large ")
            with ui.row():
                ui.button(
                    "Init",
                    on_click=lambda: [
                        self.initPub.publish(Empty()),
                        self.fetch_scenario_names(),
                    ],
                    color="blue",
                )
                ui.button(
                    "Connect",
                    on_click=lambda: self.connectPub.publish(Empty()),
                    color="blue",
                )
                ui.button(
                    "Disconnect",
                    on_click=lambda: self.disconnectPub.publish(Empty()),
                    color="grey",
                )
                ui.button(
                    "Arm", on_click=lambda: self.armPub.publish(Empty()), color="orange"
                )
                ui.button(
                    "Disarm",
                    on_click=lambda: self.disarmPub.publish(Empty()),
                    color="orange",
                )
                ui.button(
                    "Start",
                    on_click=lambda: self.startPub.publish(Empty()),
                    color="green",
                )
                ui.button(
                    "All Clear",
                    on_click=lambda: self.allClearPub.publish(Empty()),
                    color="grey",
                )
            with ui.row().classes("items-center"):
                self._scenario_select = (
                    ui.select(
                        options=self.scenario_names,
                        label="Active scenario",
                        on_change=lambda e: [
                            self.set_active_scenario(e.value),
                            self.connectPub.publish(Empty()),
                        ],
                    )
                    .bind_value(self, "selected_scenario")
                    .bind_enabled_from(
                        self.OBC_state, "state", backward=lambda s: s == "CONNECTED"
                    )
                    .props("outlined dense")
                )
                ui.label(
                    "Changing scenario can ony be done in CONNECTED state"
                ).classes("text-sm text-grey-7 italic")
            with ui.row():
                ui.label().bind_text_from(
                    self.OBC_state, "state", backward=lambda n: f"State: {n}"
                ).classes("text-lg")
            with ui.row():
                ui.button(
                    "Reset Test Objects",
                    on_click=lambda: self.resetTestObjectsPub.publish(Empty()),
                    color="grey",
                )
                ui.button(
                    "Reload Object Settings",
                    on_click=lambda: self.reloadObjectSettingsPub.publish(Empty()),
                    color="grey",
                )

    def fetch_scenario_names(self) -> None:
        if not self.get_scenario_names_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("open_scenario_gateway get_parameters not available")
            return
        req = GetParameters.Request()
        req.names = ["open_scenario_file"]
        self.get_scenario_names_client.call_async(req).add_done_callback(
            self.on_scenario_names_fetched
        )

    def on_scenario_names_fetched(self, future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to fetch scenario names: {e}")
            return
        if (
            response.values
            and response.values[0].type == ParameterType.PARAMETER_STRING_ARRAY
        ):
            names = list(response.values[0].string_array_value)
            self.scenario_names = names
            if self._scenario_select is not None:
                self._scenario_select.options = names
                if not self.selected_scenario and names:
                    self.selected_scenario = names[0]
                    self._scenario_select.set_value(names[0])
                self._scenario_select.update()

    def set_active_scenario(self, scenario_name: str) -> None:
        if not isinstance(scenario_name, str) or not scenario_name:
            return
        msg = String()
        msg.data = scenario_name
        self.active_scenario_pub.publish(msg)

    def get_object_control_state_callback(self):
        # Call the service
        while not self.get_object_control_state_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("service not available, waiting again...")
            self.lost_connection = True
        self.lost_connection = False
        future = self.get_object_control_state_client.call_async(self.OBC_state_req)
        future.add_done_callback(
            lambda future: self.get_object_control_state_callback_done(future)
        )

    def get_object_control_state_callback_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))
        else:
            self.OBC_state["state"] = OBC_STATES[response.state]
