import threading
from pathlib import Path
from atos_interfaces.srv import *

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, ui

MAX_TIMEOUT = 3

class ObjectPanelNode(Node):
    def __init__(self) -> None:
        super().__init__('object_panel')
        self.id_client = self.create_client(GetObjectIds, '/atos/get_object_ids')
        self.ip_client = self.create_client(GetObjectIp, '/atos/get_object_ip')
        self.object_ids_req = GetObjectIds.Request()
        self.object_id_ip_map = {}                
        self.get_object_ids()
        
        with Client.auto_index_client:
            pass
    
        @ui.page(path='/object', title="ATOS Object Panel")
        def render_objectpanel() -> None:
            with ui.row() as self.first_row:
                with ui.button(text='Refresh', on_click=self.refresh):
                    ui.icon('refresh')
            with ui.row() as self.second_row:
                for object_id in self.object_id_ip_map.keys():
                    ui.input(label=f'Object {object_id}').bind_value_from(self.object_id_ip_map[object_id]
                            ).on('keydown.enter', lambda result, object_id=object_id: self.update_object_ip(object_id, result.sender.value))

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

    def refresh(self):
        self.get_object_ids()
        ui.open('/object')
