from atos_interfaces.srv import GetObjectIds, GetObjectIp, SetObjectIp

from rclpy.node import Node

from nicegui import Client, ui

MAX_TIMEOUT = 3

class ObjectPanelNode(Node):
    """ This node is responsible for rendering the object panel and visualize the IP address for each object ID in the scenario.
    """
    def __init__(self) -> None:
        """ Initializes the node, fetches all object IDs and their IPs in the current scenario and visualizes them.
        """
        super().__init__('object_panel')
        self.get_id_client = self.create_client(GetObjectIds, '/atos/get_object_ids')
        self.get_ip_client = self.create_client(GetObjectIp, '/atos/get_object_ip')
        self.set_ip_client = self.create_client(SetObjectIp, SetObjectIp.Request.SERVICE_NAME)
        
        self.object_ids_req = GetObjectIds.Request()
        self.object_id_ip_map = {}
        self.get_object_ids()

        with Client.auto_index_client:
            pass
    
        @ui.page(path='/object', title="ATOS Object Panel")
        def render_objectpanel() -> None:
            """ Render the object panel page. Generates a list of objects and their IPs.
            """
            with ui.row() as self.refresh_row:
                with ui.button(text='Refresh', on_click=self.refresh):
                    ui.icon('refresh')
            for object_id in self.object_id_ip_map.keys():
                with ui.element('div').classes('p-2 bg-blue-100'):
                    with ui.row():
                        ui.label(f'Object {object_id}: ')
                        ui.label(self.object_id_ip_map[object_id]) 
                        # Will be used in the future
                        # ui.input(label=f'Object {object_id}').bind_value(self.object_id_ip_map, object_id
                        #         ).on('keydown.enter', lambda result, object_id=object_id: self.update_object_ip(object_id, result.sender.value))

    def get_object_ids(self):
        """ Fetches all object IDs in the current scenario and calls the get_object_ips method for each object to fetch their IPs.
        """
        service_timeout_counter = 0
        while not self.get_id_client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().debug('Get object ID service not available, waiting again...')
            if service_timeout_counter > MAX_TIMEOUT:
                ui.notify(f'Get object ID service not available after {MAX_TIMEOUT} seconds')
                self.get_logger().info(f'Get object ID service not available after {MAX_TIMEOUT} seconds')
                return
        future = self.get_id_client.call_async(self.object_ids_req)
        future.add_done_callback(lambda future: self.get_object_ips(future.result().ids))

    def get_object_ips(self, object_ids):
        """ Fetches the IP address for each object ID in the current scenario.
        
        Args:
            object_ids (list): List of object IDs in the current scenario.
        
        """
        service_timeout_counter = 0
        while not self.get_ip_client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().debug('Get object IP service not available, waiting again...')
            if service_timeout_counter > MAX_TIMEOUT:
                ui.notify(f'Get object IP service not available after {MAX_TIMEOUT} seconds')
                self.get_logger().info(f'Get object IP service not available after {MAX_TIMEOUT} seconds')
                return
        for object_id in object_ids:
            self.object_id_ip_map[object_id] = ""
            future = self.get_ip_client.call_async(GetObjectIp.Request(id=object_id))
            future.add_done_callback(lambda future: self.save_ips_locally(future.result()))

    def save_ips_locally(self, object_ip):
        """ Saves the IP address for each object ID in the current scenario locally.

        Args:
            object_ip (GetObjectIp.Response): IP address for the given object ID.
        
        """
        if object_ip.ip != "":
            self.object_id_ip_map[object_ip.id] = object_ip.ip
            self.get_logger().debug(f'Object {object_ip.id} IP: {object_ip.ip}')

    def update_object_ip(self, object_id, object_ip):
        """ Updates the IP address for a given object ID in the current scenario.
        
        Args:
            object_id (int): Object ID to update the IP address for.
            object_ip (str): New IP address for the given object ID.
        
        """
        ui.notify(f'Setting object {object_id} IP: {object_ip}')
        service_timeout_counter = 0
        while not self.set_ip_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug('Set object IP service not available, waiting again...')
            if service_timeout_counter > MAX_TIMEOUT:
                ui.notify(f'Set object IP service not available after {MAX_TIMEOUT} seconds')
                self.get_logger().info(f'Set object IP service not available after {MAX_TIMEOUT} seconds')
                return
        future = self.set_ip_client.call_async(SetObjectIp.Request(id=object_id, ip=object_ip))
        future.add_done_callback(lambda future: self.update_object_ip_result(future.result()))
    
    def update_object_ip_result(self, result):
        """ Updates the IP address for a given object ID in the current scenario.

        Args:
            result (SetObjectIp.Response): Response from the set object IP service.
        
        """
        if result.success:
            with self.refresh_row:
                ui.notify(f'IP set to {result.ip} for object {result.id} was successful')
            self.get_logger().info(f'IP set to {result.ip} for object {result.id} was successful')
            self.object_id_ip_map[result.id] = result.ip
        else:
            with self.refresh_row:
                ui.notify(f'Failed to set object {result.id} IP to {result.ip}')
            self.get_logger().info(f'Failed to set object {result.id} IP to {result.ip}')

    def refresh(self):
        """ Refreshes the object panel by fetching all object IDs and their IPs in the current scenario.
        """
        self.get_object_ids()
        ui.open('/object')
