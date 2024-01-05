import threading
from pathlib import Path
from atos_interfaces.srv import *

import rclpy
from std_msgs.msg import Empty
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically, GetParameters, ListParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run
from .local_file_picker import local_file_picker

QOS = rclpy.qos.QoSProfile(depth=10)

MAX_TIMEOUT = 1

class ConfigPanelNode(Node):

    def __init__(self) -> None:
        super().__init__('config_panel')
        self.module_list = ["journal_control", "esmini_adapter", "object_control", "osi_adapter", "mqtt_bridge",
                            "trajectorylet_streamer", "pointcloud_publisher", "back_to_start", "integration_testing_handler"]
        self.client_list = self.init_clients()
        self.parameters = {}
        self.get_parameters_list(self.client_list)
        self.render_ui()

    def render_ui(self) -> None:
        with Client.auto_index_client:
            with ui.tabs() as tabs:
                ui.tab('Home', icon='🏠')
                ui.tab('Esmini_adapter')
                ui.tab('Object_control')
                ui.tab('Osi_adapter')
                ui.tab('Mqtt_bridge')
                ui.tab('Trajectorylet Streamer')
                ui.tab('Pointcloud Publisher')
                ui.tab('Back to Start')
                ui.tab('Integration Testing Handler')
            with ui.tab_panels(tabs, value='Home'):
                with ui.tab_panel('Home'):
                    ui.label('Welcome to the ATOS Configuration Panel!')
                with ui.tab_panel('Journal_control'):
                    ui.input(label="Scenario name").bind_value_from(self.parameters, "open_scenario_file")
                with ui.tab_panel('Esmini_adapter'):
                    ui.label('Openscenario file:')
                    ui.button('Choose new file', on_click=self.pick_scenario_file, icon='📂')
                with ui.tab_panel('Object_control'):
                    ui.input(label="Max missing heartbeats").bind_value_from(self.parameters, "max_missing_heartbeats")
                    ui.input(label="Transmitter ID").bind_value_from(self.parameters, "transmitter_id")
                with ui.tab_panel('Osi_adapter'):
                    ui.input(label="Address").bind_value_from(self.parameters, "address")
                    ui.input(label="Port").bind_value_from(self.parameters, "port")
                    ui.input(label="Protocol").bind_value_from(self.parameters, "protocol")
                    ui.input(label="Frequency").bind_value_from(self.parameters, "frequency")
                with ui.tab_panel('Mqtt_bridge'):
                    ui.input(label="Broker IP").bind_value_from(self.parameters, "broker_ip")
                    ui.input(label="Pub client ID").bind_value_from(self.parameters, "pub_client_id")
                    ui.input(label="Username").bind_value_from(self.parameters, "username")
                    ui.input(label="Password").bind_value_from(self.parameters, "password")
                    ui.input(label="Topic").bind_value_from(self.parameters, "topic")
                    ui.input(label="Quality of Service").bind_value_from(self.parameters, "quality_of_service")
                with ui.tab_panel("Trajectorylet Streamer"):
                    ui.input(label="Chunk duration").bind_value_from(self.parameters, "chunk_duration")
                with ui.tab_panel("Pointcloud Publisher"):
                    ui.button('Choose new file', on_click=self.pick_pointcloud_file, icon='📂')
                with ui.tab_panel("Back to Start"):
                    ui.input(label="Turn radius").bind_value_from(self.parameters, "turn_radius")
                # with ui.tab_panel("Integration Testing Handler"):
                #     ui.switch(label="Scenario execution", on_change=self.get_logger().info(self.parameters["scenario_execution"])).bind_value_from(self.parameters, "scenario_execution")

    def init_clients(self) -> dict:
        client_list = {}
        for module in self.module_list:
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
            while not get_params_client.wait_for_service(timeout_sec=1.0) and service_timeout_counter < MAX_TIMEOUT:
                service_timeout_counter += 1
                self.get_logger().info(f'{module} not available, waiting again...')
            if service_timeout_counter >= MAX_TIMEOUT:
                self.get_logger().info(f'{module} not available after {MAX_TIMEOUT} seconds, please try again later')
                continue
            self.get_logger().info(f'Retrieving parameter values from {module}...')
            threading.Thread(target=self.call_service, args=(get_params_client, get_param_value_client)).start()
            # get_params_client.call_async(ListParameters.Request()).add_done_callback(lambda future: self.get_parameter_values(future, get_param_value_client))

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
            # self.get_logger().info(f'Parameter {param_name} has value {self.parameters[param_name]}')
        # self.get_logger().info(f'Parameters saved: {self.parameters}')

    async def pick_scenario_file(self) -> None:
        result = await local_file_picker('~/.astazero/ATOS', multiple=False, show_hidden_files=True)
        ui.notify(f'You selected {result}')
        if not result:
            return
        service_timeout_counter = 0
        client = self.client_list["esmini_adapter"]["set_params_client"]
        while not client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().info('service not available, waiting again...')
            if service_timeout_counter >= MAX_TIMEOUT:
                ui.notify(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                self.get_logger().info(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                return

        scenario_param = Parameter()
        scenario_param.name = 'open_scenario_file'
        scenario_param.value.type = ParameterType.PARAMETER_STRING
        scenario_param.value.string_value = str(result)

        esmini_param_req = SetParametersAtomically.Request()
        esmini_param_req.parameters.append(scenario_param)

        future = client.call_async(esmini_param_req)
        future.add_done_callback(lambda future: self.set_param_callback(future))

    async def pick_pointcloud_file(self) -> None:
        result = await local_file_picker('~/.astazero/ATOS/pointclouds', multiple=True, show_hidden_files=True)
        ui.notify(f'You selected {result}')
        if not result:
            return
        service_timeout_counter = 0
        client = self.client_list["pointcloud_publisher"]["set_params_client"]
        while not client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().info('service not available, waiting again...')
            if service_timeout_counter >= MAX_TIMEOUT:
                ui.notify(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                self.get_logger().info(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                return

        scenario_param = Parameter()
        scenario_param.name = 'pointcloud_files'
        scenario_param.value.type = ParameterType.PARAMETER_STRING_ARRAY
        scenario_param.value.string_value = str(result)

        esmini_param_req = SetParametersAtomically.Request()
        esmini_param_req.parameters.append(scenario_param)

        future = client.call_async(esmini_param_req)
        future.add_done_callback(lambda future: self.set_param_callback(future))

    def set_param_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result of set_parameters was {"successful" if response.result.successful else "unsuccesful"}')
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
            

def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = ConfigPanelNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=3000, title='Config Panel')