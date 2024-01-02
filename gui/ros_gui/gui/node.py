import threading
from pathlib import Path
from atos_interfaces.srv import *

import rclpy
from std_msgs.msg import Empty
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run

QOS = rclpy.qos.QoSProfile(depth=10)


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.initPub = self.create_publisher(Empty, '/atos/init', QOS)
        self.connectPub = self.create_publisher(Empty, '/atos/connect', QOS)
        self.disconnectPub = self.create_publisher(Empty, '/atos/disconnect', QOS)
        self.armPub = self.create_publisher(Empty, '/atos/arm', QOS)
        self.disarmPub = self.create_publisher(Empty, '/atos/disarm', QOS)
        self.startPub = self.create_publisher(Empty, '/atos/start', QOS)
        self.abortPub = self.create_publisher(Empty, '/atos/abort', QOS)
        self.allClearPub = self.create_publisher(Empty, '/atos/all_clear', QOS)
        self.resetTestObjectsPub = self.create_publisher(Empty, '/atos/reset_test_objects', QOS)
        self.reloadObjectSettingsPub = self.create_publisher(Empty, '/atos/reload_object_settings', QOS)
        # # Create a timer service call and put in variable
        # self.timer = self.create_timer(1, self.get_object_control_state_callback)
        # # Create a service client for each service
        # self.get_object_control_state_client = self.create_client(GetObjectControlState, '/atos/get_object_control_state')
        # while not self.get_object_control_state_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.OBC_state_req = GetObjectControlState.Request()
        # self.OBC_state = None

        with Client.auto_index_client:
            with ui.row():
                ui.button('Abort', on_click=lambda: self.abortPub.publish(Empty()), color='red')
            with ui.row():
                ui.button('Init', on_click=lambda: self.initPub.publish(Empty()), color='blue')
                ui.button('Connect', on_click=lambda: self.connectPub.publish(Empty()), color='blue')
                ui.button('Disconnect', on_click=lambda: self.disconnectPub.publish(Empty()), color='blue')
                ui.button('Arm', on_click=lambda: self.armPub.publish(Empty()), color='yellow')
                ui.button('Disarm', on_click=lambda: self.disarmPub.publish(Empty()), color='grey')
                ui.button('Start', on_click=lambda: self.startPub.publish(Empty()), color='green')
                ui.button('All Clear', on_click=lambda: self.allClearPub.publish(Empty()), color='blue')
            with ui.row():
                ui.label('Object Control State: Placeholder')
            with ui.row():
                ui.button('Reset Test Objects', on_click=lambda: self.resetTestObjectsPub.publish(Empty()), color='grey')
                ui.button('Reload Object Settings', on_click=lambda: self.reloadObjectSettingsPub.publish(Empty()), color='grey')


    # def get_object_control_state_callback(self):
    #     # Call the service
    #     self.new_OBC_state = self.get_object_control_state_client.call_async(self.OBC_state_req)
    #     # If the service is available
    #     if self.new_OBC_state.done():
    #         # Get the response
    #         self.OBC_state = self.new_OBC_state.result()
    #         # Update the label
    #         self.OBC_status_label().bind_text_from(self.OBC_state, 'state')
    #     else:
    #         self.get_logger().info('service not available, waiting again...')
    
    # class OBC_status_label(ui.label):
    #     def _handle_text_change(self, text: str) -> None:
    #         super()._handle_text_change(text)
    #         if text == 'ok':
    #             self.classes(replace='text-positive')
    #         else:
    #             self.classes(replace='text-negative')


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3000)