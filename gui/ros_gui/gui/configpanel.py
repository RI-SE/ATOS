import threading
from pathlib import Path
import os
import time
import json

import rclpy
from rcl_interfaces.msg import ParameterType, Parameter
from rcl_interfaces.srv import SetParametersAtomically, GetParameters, ListParameters
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nicegui import Client, ui, app, ui_run
from .local_file_picker import local_file_picker

CONF_PATH = os.path.join(os.path.expanduser('~'), ".astazero/ATOS/conf")
MAX_TIMEOUT = 1

class ConfigPanelNode(Node):

    def __init__(self) -> None:
        super().__init__('config_panel')
        schema_path = os.path.join(CONF_PATH, "atos-param-schema.json")
        self.json_schema = json.load(open(schema_path, "r"))
        modules = self.json_schema["modules"].keys()
        time.sleep(0.5) # Allow time for this node to initialize before discovering other nodes, otherwise it will fail to find the others.
        self.active_nodes_and_namespaces = self.get_node_names_and_namespaces()
        self.active_node_list = [node for node, namespace in self.active_nodes_and_namespaces if node in modules and "atos" in namespace]

        self.client_list = self.init_clients()
        self.parameters = {}
        threading.Thread(target=self.get_parameters_list, args=(self.client_list,)).start()
        self.render_configpanel()

    def init_clients(self) -> dict:
        client_list = {}
        for module in self.active_node_list:
            get_params_client = self.create_client(ListParameters, f'/atos/{module}/list_parameters')
            get_param_value_client = self.create_client(GetParameters, f'/atos/{module}/get_parameters')
            set_params_client = self.create_client(SetParametersAtomically, f'/atos/{module}/set_parameters_atomically')
            client_list[module] = {"get_params_client": get_params_client, "get_param_value_client": get_param_value_client, "set_params_client": set_params_client}
        return client_list

    def get_parameters_list(self, client_list) -> None:
        self.get_logger().debug(f'Retrieving all parameters in {[node for node in client_list.keys()]}')
        for node in client_list.keys():
            get_params_client = client_list[node]["get_params_client"]
            get_param_value_client = client_list[node]["get_param_value_client"]
            service_timeout_counter = 0
            while not get_params_client.wait_for_service(timeout_sec=0.1) and service_timeout_counter < MAX_TIMEOUT:
                service_timeout_counter += 1
                self.get_logger().debug(f'{node} not available, waiting again...')
            if service_timeout_counter >= MAX_TIMEOUT:
                self.get_logger().info(f'{node} not available after {MAX_TIMEOUT} seconds, please try again later')
                ui.notify(f'{node} not available after {MAX_TIMEOUT} seconds, please try again later')
                continue
            threading.Thread(target=self.call_service, args=(get_params_client, get_param_value_client)).start()

    def call_service(self, get_params_client, get_param_value_client) -> None:
        get_params_client.call_async(ListParameters.Request()).add_done_callback(lambda future: self.get_parameter_values(future, get_param_value_client))

    def get_parameter_values(self, future, client) -> None:
        response = future.result()
        get_param_req = GetParameters.Request()
        param_names = response.result.names
        get_param_req.names = param_names
        client.call_async(get_param_req).add_done_callback(lambda future: self.save_params_locally(future, param_names))

    def save_params_locally(self, future, param_names) -> None:
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
                case ParameterType.PARAMETER_BYTE_ARRAY:
                    self.parameters[param_name] = future.result().values[idx].byte_array_value
                case ParameterType.PARAMETER_BOOL_ARRAY:
                    self.parameters[param_name] = future.result().values[idx].bool_array_value
                case ParameterType.PARAMETER_INTEGER_ARRAY:
                    self.parameters[param_name] = future.result().values[idx].integer_array_value
                case ParameterType.PARAMETER_DOUBLE_ARRAY:
                    self.parameters[param_name] = future.result().values[idx].double_array_value
                case ParameterType.PARAMETER_STRING_ARRAY:
                    self.parameters[param_name] = future.result().values[idx].string_array_value
                case _:
                    self.get_logger().info(f'Parameter {param_name} has an unsupported type {param_type}')

    async def pick_file(self, node_name, param_name) -> None:
        result = await local_file_picker('~/.astazero/ATOS', multiple=False, show_hidden_files=True)
        ui.notify(f'You selected {result}')
        if not result:
            return
        self.parameters[param_name] = str(result)
        self.set_parameter(node_name, param_name, str(result))

    async def pick_files(self, node_name, param_name) -> None:
        result = await local_file_picker('~/.astazero/ATOS', multiple=True, show_hidden_files=True)
        ui.notify(f'You selected {result}')
        if not result:
            return
        self.parameters[param_name] = str(result)
        self.set_parameter(node_name, param_name, str(result))

    def set_param_callback(self, future, param_name):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
            return
        self.get_logger().debug(f'Setting parameter {param_name} was {"successful" if response.result.successful else "unsuccesful"}')
        with self.splitter:
            ui.notify(f'Setting parameter {param_name} was {"successful" if response.result.successful else "unsuccesful"}')

    def set_parameter(self, node_name, param_name, param_value) -> None:
        self.get_logger().debug(f'Setting parameter {param_name} in {node_name} to {param_value}')
        ui.notify(f'Setting parameter {param_name} in {node_name} to {param_value}')

        service_timeout_counter = 0
        client = self.client_list[node_name]["set_params_client"]
        while not client.wait_for_service(timeout_sec=1.0):
            service_timeout_counter += 1
            self.get_logger().debug('service not available, waiting again...')
            if service_timeout_counter >= MAX_TIMEOUT:
                ui.notify(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                self.get_logger().info(f'Service not available after {MAX_TIMEOUT} seconds, please try again later')
                return

        parameter = Parameter()
        parameter.name = param_name
        param_type = self.json_schema["modules"][node_name]["ros__parameters"][param_name]["type"]
        self.assign_value_and_type_to_parameter(parameter, param_type, param_value) # Checks param_value type and assigns it to the parameter
        param_req = SetParametersAtomically.Request()
        param_req.parameters.append(parameter)

        future = client.call_async(param_req)
        future.add_done_callback(lambda future: self.set_param_callback(future, param_name))

    def assign_value_and_type_to_parameter(self, parameter, param_type, param_value) -> None:
        match param_type:
            case "file":
                if type(param_value) != str:
                    ui.notify(f'Please select a file')
                    return
                parameter.value.type = ParameterType.PARAMETER_STRING
                parameter.value.string_value = param_value
            case "bool":
                if type(param_value) != bool:
                    ui.notify(f'Please select a boolean')
                    return
                parameter.value.type = ParameterType.PARAMETER_BOOL
                parameter.value.bool_value = param_value
            case "int":
                if type(param_value) != int:
                    ui.notify(f'Please select an integer')
                    return
                parameter.value.type = ParameterType.PARAMETER_INTEGER
                parameter.value.integer_value = param_value
            case "double":
                if type(param_value) != float:
                    ui.notify(f'Please select a double')
                    return
                parameter.value.type = ParameterType.PARAMETER_DOUBLE
                parameter.value.double_value = param_value
            case "string":
                if type(param_value) != str:
                    ui.notify(f'Please select a string')
                    return
                parameter.value.type = ParameterType.PARAMETER_STRING
                parameter.value.string_value = param_value
            case "file_array":
                if type(param_value) != list or type(param_value[0]) != str:
                    ui.notify(f'Please select one or more files')
                    return
                parameter.value.type = ParameterType.PARAMETER_STRING_ARRAY
                parameter.value.string_array_value = param_value
            case "byte_array":
                if type(param_value) != list or type(param_value[0]) != int:
                    ui.notify(f'Please select one or more bytes')
                    return
                parameter.value.type = ParameterType.PARAMETER_BYTE_ARRAY
                parameter.value.byte_array_value = param_value
            case "bool_array":
                if type(param_value) != list or type(param_value[0]) != bool:
                    ui.notify(f'Please select one or more booleans')
                    return
                parameter.value.type = ParameterType.PARAMETER_BOOL_ARRAY
                parameter.value.bool_array_value = param_value
            case "int_array":
                if type(param_value) != list or type(param_value[0]) != int:
                    ui.notify(f'Please select one or more integers')
                    return
                parameter.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                parameter.value.integer_array_value = param_value
            case "double_array":
                if type(param_value) != list or type(param_value[0]) != float:
                    ui.notify(f'Please select one or more doubles')
                    return
                parameter.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                parameter.value.double_array_value = param_value
            case "string_array":
                if type(param_value) != list or type(param_value[0]) != str:
                    ui.notify(f'Please select one or more strings')
                    return
                parameter.value.type = ParameterType.PARAMETER_STRING_ARRAY
                parameter.value.string_array_value = param_value
            case _:
                self.get_logger().info(f'Unsupported type {param_type}')
                ui.notify(f'Unsupported type {param_type}')
                return

    def render_configpanel(self) -> None:
        with Client.auto_index_client:
            with ui.splitter(value=30).classes('w-1/2') as self.splitter:
                with self.splitter.before:
                    with ui.tabs().props('vertical').classes('w-fit') as tabs:
                        ui.tab('Home', icon='🏠')
                        for node in self.active_node_list:
                            ui.tab(node.replace("_", " "))
                with self.splitter.after:
                    with ui.tab_panels(tabs, value='Home'):
                        with ui.tab_panel('Home'):
                            ui.label('Welcome to ATOS config panel!').classes('text-h4')
                            if self.active_node_list:
                                ui.label("Select a node to configure. Press the refresh button if you can't find the node you're looking for.")
                            else:
                                ui.label("No nodes were discovered. Press the refresh button to try again.")
                            ui.button('Refresh', on_click=lambda: self.refresh(self.splitter), icon='🔄')
                        for node in self.active_node_list:
                            with ui.tab_panel(node.replace("_", " ")):
                                for param_name in self.json_schema["modules"][node]["ros__parameters"]: # Should this loop through the active parameters instead?
                                    param_text = param_name.replace("_", " ")
                                    param_type = self.json_schema["modules"][node]["ros__parameters"][param_name]["type"]
                                    param_description = self.json_schema["modules"][node]["ros__parameters"][param_name]["description"]
                                    with ui.row():
                                        match param_type:
                                            case "file":
                                                with ui.column():
                                                    ui.button(f'Select {param_name.replace("_", " ")}:', 
                                                             on_click=lambda node=node, param_name=param_name: self.pick_file(node, param_name), 
                                                             icon='📂')
                                            case "bool":
                                                ui.checkbox(param_text).bind_value(self.parameters, param_name).on('change', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.value))
                                            case "int":
                                                ui.number(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, int(result.sender.value)))
                                            case "double":
                                                ui.number(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.sender.value))
                                            case "string":
                                                ui.input(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.sender.value))
                                            case "file_array":
                                                with ui.column():
                                                    ui.label(param_name.replace("_", " ") + ":")
                                                    ui.button('Choose new file:', on_click=lambda node=node, param_name=param_name: self.pick_files(node, param_name), icon='📂')
                                            case "byte_array":
                                                ui.input(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.sender.value))
                                            case "bool_array":
                                                ui.input(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.sender.value))
                                            case "int_array":
                                                ui.input(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.sender.value))
                                            case "double_array":
                                                ui.input(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.sender.value))
                                            case "string_array":
                                                ui.input(param_text).bind_value(self.parameters, param_name).on('keydown.enter', lambda result, node=node, param_name=param_name: self.set_parameter(node, param_name, result.sender.value))
                                            case _:
                                                ui.label(f"Unsupported type {param_type}").classes('text-red-500')
                                        help_switch = ui.switch('Help')
                                    ui.label(param_description).bind_visibility_from(help_switch, 'value')

    def refresh(self, splitter) -> None:
        splitter.delete()
        modules = self.json_schema["modules"].keys()
        self.active_nodes_and_namespaces = self.get_node_names_and_namespaces()
        self.active_node_list = [node for node, namespace in self.active_nodes_and_namespaces if node in modules and "atos" in namespace]

        self.client_list = self.init_clients()
        self.parameters = {}
        with self.splitter:
            ui.notify('Retrieving parameters again...')
        threading.Thread(target=self.get_parameters_list, args=(self.client_list,)).start()
        self.render_configpanel()

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