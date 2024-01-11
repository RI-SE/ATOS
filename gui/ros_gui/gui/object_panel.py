import threading
from pathlib import Path
from atos_interfaces.srv import *

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run

MAX_TIMEOUT = 3

class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.id_client = self.create_client(GetObjectIds, '/atos/get_object_ids')
        self.ip_client = self.create_client(GetObjectIp, '/atos/get_object_ip')
        self.object_ids_req = GetObjectIds.Request()
        self.object_id_ip_map = {}

        with Client.auto_index_client as self.ui_client:
            with ui.row() as self.first_row:
                with ui.button(text='Refresh', on_click=self.refresh):
                    ui.icon('refresh')
                
        self.get_object_ids()
        self.render_ui()

    def get_object_ids(self):
        # Call the service
        service_timeout_counter = 0
        while not self.id_client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().debug('Get object ID service not available, waiting again...')
            if service_timeout_counter > MAX_TIMEOUT:
                with self.ui_client:
                    ui.notify(f'Get object ID service not available after {MAX_TIMEOUT} seconds')
                self.get_logger().info(f'Get object ID service not available after {MAX_TIMEOUT} seconds')
                return
        future = self.id_client.call_async(self.object_ids_req)
        future.add_done_callback(lambda future: self.get_object_ips(future.result().ids))

    def get_object_ips(self, object_ids):
        # Call the service
        service_timeout_counter = 0
        while not self.ip_client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().debug('Get object IP service not available, waiting again...')
            if service_timeout_counter > MAX_TIMEOUT:
                with self.ui_client:
                    ui.notify(f'Get object IP service not available after {MAX_TIMEOUT} seconds')
                self.get_logger().info(f'Get object IP service not available after {MAX_TIMEOUT} seconds')
                return
        for object_id in object_ids:
            self.object_id_ip_map[object_id] = ""
            future = self.ip_client.call_async(GetObjectIp.Request(id=object_id))
            future.add_done_callback(lambda future: self.save_ips_locally(future.result()))

    def save_ips_locally(self, object_ip):
        if object_ip.ip != "":
            self.object_id_ip_map[object_ip.id] = object_ip.ip
            with self.ui_client:
                ui.notify(f'Object {object_ip.id} IP: {object_ip.ip}')
            self.get_logger().debug(f'Object {object_ip.id} IP: {object_ip.ip}')
            # with open(f'/home/atos/atos_ws/src/atos/atos_interfaces/config/objects/object{object_ip.id}.yaml', 'r+') as file:
            #     data = file.readlines()
            #     data[2] = f'  ip: {object_ip.ip}\n'
            #     file.seek(0)
            #     file.writelines(data)
            #     file.truncate()

    def update_object_ip(self, object_id, object_ip):
        with self.ui_client:
            ui.notify(f'Setting object {object_id} IP: {object_ip}')

    def render_ui(self):
        with self.ui_client:
            with ui.row() as self.second_row:
                for object_id in self.object_id_ip_map.keys():
                    ui.input(label=f'Object {object_id}').bind_value_from(self.object_id_ip_map[object_id]
                            ).on('keydown.enter', lambda result, object_id=object_id: self.update_object_ip(object_id, result.sender.value))

    def refresh(self):
        self.get_object_ids()
        self.second_row.delete()
        self.render_ui()

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

#Starting the ros node in a thread managed by nicegui. It will restarted with "on_startup" after a reload.
#It has to be in a thread, since NiceGUI wants the main thread for itself.
app.on_startup(lambda: threading.Thread(target=ros_main).start())

ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3000, title='Object Panel')