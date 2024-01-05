import threading
from pathlib import Path

import rclpy
from rcl_interfaces.msg import ParameterType, Parameter
from rcl_interfaces.srv import SetParametersAtomically, GetParameters, ListParameters
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nicegui import Client, ui, app, ui_run
from .local_file_picker import local_file_picker

QOS = rclpy.qos.QoSProfile(depth=10)

MAX_TIMEOUT = 1

class ConfigPanelNode(Node):

    def __init__(self) -> None:
        super().__init__('config_panel')
        banned_nodes = ["config_panel", "control_panel", "foxglove_bridge"]
        self.nodes_and_namespaces = self.get_node_names_and_namespaces()
        self.node_list = [node for node, namespace in self.nodes_and_namespaces if node not in banned_nodes and "ros2cli" not in node]
        self.get_logger().info(f'Discovered nodes: {self.node_list}')

        self.client_list = self.init_clients()
        self.parameters = {}
        threading.Thread(target=self.get_parameters_list, args=(self.client_list,)).start()
        self.render_configpanel()

    def render_configpanel(self) -> None:
        with Client.auto_index_client:
            with ui.splitter(value=30).classes('w-1/2') as splitter:
                with splitter.before:
                    with ui.tabs().props('vertical').classes('w-fit') as tabs:
                        ui.tab('Home', icon='🏠')
                        for module in self.node_list:
                            ui.tab(module.replace("_", " "))
                with splitter.after:
                    with ui.tab_panels(tabs, value='Home'):
                        with ui.tab_panel('Home'):
                            ui.label('Welcome to ATOS config panel!').classes('text-h4')
                            if self.node_list:
                                ui.label('Select a node to configure')
                            else:
                                ui.label('No nodes were discovered')
                        with ui.tab_panel('journal control'):
                            ui.input(label="Scenario name", 
                                    on_change=lambda result: self.set_parameter("journal_control", "scenario_name", result.value)).bind_value(self.parameters, "scenario_name")
                        with ui.tab_panel('esmini adapter'):
                            ui.label('Openscenario file:')
                            ui.button('Choose new file', on_click=self.pick_scenario_file, icon='📂')
                        with ui.tab_panel('object control'):
                            ui.number(label="Max missing heartbeats", 
                                    on_change=lambda result: self.set_parameter("object_control", "max_missing_heartbeats", result.value)).bind_value(self.parameters, "max_missing_heartbeats")
                            ui.number(label="Transmitter ID", 
                                    on_change=lambda result: self.set_parameter("object_control", "transmitter_id", result.value)).bind_value(self.parameters, "transmitter_id")
                        with ui.tab_panel('osi adapter'):
                            ui.input(label="Address", 
                                    on_change=lambda result: self.set_parameter("osi_adapter", "address", result.value)).bind_value(self.parameters, "address").classes('w-40')
                            ui.number(label="Port", 
                                    on_change=lambda result: self.set_parameter("osi_adapter", "port", result.value)).bind_value(self.parameters, "port").classes('w-40')
                            ui.select(options=["tcp", "udp"], 
                                    label="Protocol", 
                                    on_change=lambda result: self.set_parameter("osi_adapter", "protocol", result.value)).bind_value(self.parameters, "protocol").classes('w-40')
                            ui.number(label="Frequency", 
                                    on_change=lambda result: self.set_parameter("osi_adapter", "frequency", result.value)).bind_value(self.parameters, "frequency").classes('w-40')
                        with ui.tab_panel('mqtt bridge'):
                            ui.input(label="Broker IP", 
                                    on_change=lambda result: self.set_parameter("mqtt_bridge", "broker_ip", result.value)).bind_value(self.parameters, "broker_ip")
                            ui.number(label="Pub client ID", 
                                    on_change=lambda result: self.set_parameter("mqtt_bridge", "pub_client_id", result.value)).bind_value(self.parameters, "pub_client_id")
                            ui.input(label="Username", 
                                    on_change=lambda result: self.set_parameter("mqtt_bridge", "username", result.value)).bind_value(self.parameters, "username")
                            ui.input(label="Password", 
                                    on_change=lambda result: self.set_parameter("mqtt_bridge", "password", result.value)).bind_value(self.parameters, "password")
                            ui.input(label="Topic", 
                                    on_change=lambda result: self.set_parameter("mqtt_bridge", "topic", result.value)).bind_value(self.parameters, "topic")
                            ui.input(label="Quality of Service", 
                                    on_change=lambda result: self.set_parameter("mqtt_bridge", "quality_of_service", result.value)).bind_value(self.parameters, "quality_of_service")
                        with ui.tab_panel("trajectorylet streamer"):
                            ui.number(label="Chunk duration", 
                                    on_change=lambda result: self.set_parameter("trajectorylet_streamer", "chunk_duration", result.value)).bind_value(self.parameters, "chunk_duration")
                        with ui.tab_panel("pointcloud publisher"):
                            ui.label('Pointcloud files:')
                            ui.button('Choose new file', on_click=self.pick_pointcloud_file, icon='📂')
                        with ui.tab_panel("back to start"):
                            ui.number(label="Turn radius", 
                                    on_change=lambda result: self.set_parameter("back_to_start", "turn_radius", result.value)).bind_value(self.parameters, "turn_radius")
                        with ui.tab_panel("integration testing handler"):
                            ui.switch(text="Scenario execution", 
                                    on_change=lambda result: self.set_parameter("integration_testing_handler", "scenario_execution", result.value)).bind_value(self.parameters, "scenario_execution")

    def init_clients(self) -> dict:
        client_list = {}
        for module in self.node_list:
            get_params_client = self.create_client(ListParameters, f'/atos/{module}/list_parameters')
            get_param_value_client = self.create_client(GetParameters, f'/atos/{module}/get_parameters')
            set_params_client = self.create_client(SetParametersAtomically, f'/atos/{module}/set_parameters_atomically')
            client_list[module] = {"get_params_client": get_params_client, "get_param_value_client": get_param_value_client, "set_params_client": set_params_client}
        return client_list

    def get_parameters_list(self, client_list) -> None:
        self.get_logger().info('Retrieving all parameters...')
        for module in client_list.keys():
            get_params_client = client_list[module]["get_params_client"]
            get_param_value_client = client_list[module]["get_param_value_client"]
            service_timeout_counter = 0
            while not get_params_client.wait_for_service(timeout_sec=0.1) and service_timeout_counter < MAX_TIMEOUT:
                service_timeout_counter += 1
                self.get_logger().info(f'{module} not available, waiting again...')
            if service_timeout_counter >= MAX_TIMEOUT:
                self.get_logger().info(f'{module} not available after {MAX_TIMEOUT} seconds, please try again later')
                continue
            self.get_logger().info(f'Retrieving parameter values from {module}...')
            threading.Thread(target=self.call_service, args=(get_params_client, get_param_value_client)).start()

    def call_service(self, get_params_client, get_param_value_client) -> None:
        get_params_client.call_async(ListParameters.Request()).add_done_callback(lambda future: self.get_parameter_values(future, get_param_value_client))

    def get_parameter_values(self, future, client) -> None:
        response = future.result()
        get_param_req = GetParameters.Request()
        param_names = response.result.names
        get_param_req.names = param_names
        self.get_logger().debug(f'Retrieving following parameters: {param_names}')
        client.call_async(get_param_req).add_done_callback(lambda future: self.save_params_locally(future, param_names))

    def save_params_locally(self, future, param_names) -> None:
        self.get_logger().info(f'Saving parameters locally...')
        for idx, param_name in enumerate(param_names):
            param_type = future.result().values[idx].type
            match param_type:
                case ParameterType.PARAMETER_BOOL:
                    self.parameters[param_name] = future.result().values[idx].bool_value
                case ParameterType.PARAMETER_INTEGER:
                    self.parameters[param_name] = future.result().values[idx].integer_value
                case ParameterType.PARAMETER_DOUBLE:
                    self.parameters[param_name] = future.result().values[idx].double_value
                case ParameterType.PARAMETER_STRING:
                    self.parameters[param_name] = future.result().values[idx].string_value
                case _:
                    self.get_logger().info(f'Parameter {param_name} has an unsupported type {param_type}')

    async def pick_scenario_file(self) -> None:
        result = await local_file_picker('~/.astazero/ATOS', multiple=False, show_hidden_files=True)
        ui.notify(f'You selected {result[0]}')
        if not result:
            return
        self.set_parameter("esmini_adapter", "open_scenario_file", result[0])

    async def pick_pointcloud_file(self) -> None:
        result = await local_file_picker('~/.astazero/ATOS/pointclouds', multiple=True, show_hidden_files=True)
        ui.notify(f'You selected {result}')
        if not result:
            return
        self.set_parameter("pointcloud_publisher", "pointcloud_files", str(result))

    def set_param_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result of set_parameters was {"successful" if response.result.successful else "unsuccesful"}')
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    def set_parameter(self, node_name, param_name, param_value) -> None:
        self.get_logger().info(f'Setting parameter {param_name} in {node_name} to {param_value}')
        ui.notify(f'Setting parameter {param_name} in {node_name} to {param_value}')

        service_timeout_counter = 0
        client = self.client_list[node_name]["set_params_client"]
        while not client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().info('service not available, waiting again...')
            if service_timeout_counter >= MAX_TIMEOUT:
                ui.notify(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                self.get_logger().info(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                return

        scenario_param = Parameter()
        scenario_param.name = param_name
        scenario_param.value.type = ParameterType.PARAMETER_STRING
        scenario_param.value.string_value = str(param_value)

        esmini_param_req = SetParametersAtomically.Request()
        esmini_param_req.parameters.append(scenario_param)

        future = client.call_async(esmini_param_req)
        future.add_done_callback(lambda future: self.set_param_callback(future))

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

#Starting the ros node in a thread managed by nicegui. It will restarted with "on_startup" after a reload.
#It has to be in a thread, since NiceGUI wants the main thread for itself.
app.on_startup(lambda: threading.Thread(target=ros_main).start())

ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3001, title='Config Panel')