import threading
from pathlib import Path
from atos_interfaces.srv import *

import rclpy
from std_msgs.msg import Empty
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run, run

QOS = rclpy.qos.QoSProfile(depth=10)

OBC_STATES = {
        0: 'UNDEFINED',
        1: 'IDLE',
        2: 'INITIALIZED',
        3: 'CONNECTED',
        4: 'ARMED',
        5: 'RUNNING',
        6: 'REMOTE_CTRL',
        7: 'ERROR',
        8: 'ABORTING'
    }
class ControlPanelNode(Node):

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
        
        self.get_object_control_state_client = self.create_client(GetObjectControlState, '/atos/get_object_control_state')
        self.OBC_state_req = GetObjectControlState.Request()
        self.get_object_control_state_timer = self.create_timer(0.5, self.get_object_control_state_callback)

        self.OBC_state = {'state': "UNDEFINED"}
        self.lost_connection = True

        with Client.auto_index_client:
            with ui.row().bind_visibility_from(self, 'lost_connection'):
                ui.label('Lost connection to ATOS...')
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
                ui.label().bind_text_from(self.OBC_state, 'state', backward=lambda n: f'Object Control State: {n}')
            with ui.row():
                ui.button('Reset Test Objects', on_click=lambda: self.resetTestObjectsPub.publish(Empty()), color='grey')
                ui.button('Reload Object Settings', on_click=lambda: self.reloadObjectSettingsPub.publish(Empty()), color='grey')

    def get_object_control_state_callback(self):
        # Call the service
        while not self.get_object_control_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            self.lost_connection = True
        self.lost_connection = False
        future = self.get_object_control_state_client.call_async(self.OBC_state_req)
        future.add_done_callback(lambda future: self.get_object_control_state_callback_done(future))

    def get_object_control_state_callback_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        else:
            self.OBC_state["state"] = OBC_STATES[response.state]
    


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = ControlPanelNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

#Starting the ros node in a thread managed by nicegui. It will restarted with "on_startup" after a reload.
#It has to be in a thread, since NiceGUI wants the main thread for itself.
app.on_startup(lambda: threading.Thread(target=ros_main).start())

ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3000)