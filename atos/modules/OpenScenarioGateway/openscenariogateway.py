#!/usr/bin/env python3

import hashlib
import atos_interfaces.srv
from os import path
import rclpy
import rclpy.logging
import atos_interfaces.msg
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Empty
from scenariogeneration import xosc
from modules.OpenScenarioGateway.storyboard_handler import StoryBoardHandler
from typing import List


ROOT_FOLDER_PATH_PARAMETER = "root_folder_path"
ACTIVE_OBJECT_NAME_PARAMETER = "active_object_names"
SCENARIO_FILE_PARAMETER = "open_scenario_file"
DEFAULT_FOLDER_PATH = path.expanduser("~/.astazero/ATOS/")


class ScenarioObject:
    def __init__(self, name: str, catalog_ref: xosc.CatalogReference):
        self.name = name
        self.catalog_ref: xosc.CatalogReference = catalog_ref
        self.ip: str = None


class OpenScenarioGateway(Node):

    def __init__(self):
        super().__init__("open_scenario_gateway")

        # Class variables
        self.active_objects = {}
        self.vehicle_catalog = None
        self.follow_traj_to_obj_name = {}
        self.custom_command_map = {}

        self.scenario_file_md5hash = None

        # ROS parameters
        self.declare_parameter(ROOT_FOLDER_PATH_PARAMETER, DEFAULT_FOLDER_PATH)
        self.declare_parameter(SCENARIO_FILE_PARAMETER, "")
        self.declare_parameter(
            ACTIVE_OBJECT_NAME_PARAMETER, rclpy.Parameter.Type.STRING_ARRAY
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ROS subscriptions/publishers
        self.init_ = self.create_subscription(Empty, "init", self.init_callback, 10)

        self.story_board_element_sub_ = self.create_subscription(
            atos_interfaces.msg.StoryBoardElementStateChange,
            "story_board_element_state_change",
            self.story_board_element_state_change_callback,
            10,
        )
        self.start_object_pub_ = self.create_publisher(
            atos_interfaces.msg.ObjectTriggerStart, "start_object", 10
        )

        # ROS services
        self.object_ids_pub_ = self.create_service(
            atos_interfaces.srv.GetObjectIds,
            "get_object_ids",
            self.srv_get_object_id_array,
        )
        self.get_object_ip_ = self.create_service(
            atos_interfaces.srv.GetObjectIp, "get_object_ip", self.srv_get_object_ip
        )
        self.set_object_ip_ = self.create_service(
            atos_interfaces.srv.SetObjectIp, "set_object_ip", self.srv_set_object_ip
        )
        self.get_open_scenario_file_path_ = self.create_service(
            atos_interfaces.srv.GetOpenScenarioFilePath,
            "get_open_scenario_file_path",
            self.srv_get_open_scenario_file_path,
        )

    def init_callback(self, msg):
        self.update_scenario(self.get_parameter(SCENARIO_FILE_PARAMETER).value)
        self.update_active_scenario_objects(
            self.get_parameter(ACTIVE_OBJECT_NAME_PARAMETER).value
        )

    def story_board_element_state_change_callback(self, story_board_element):
        if (
            story_board_element.name in self.follow_traj_to_obj_name.keys()
            and story_board_element.state == 2  # 2 is a running state
        ):
            self.handle_follow_trajectory_action(story_board_element)
        elif (
            story_board_element.name in self.custom_command_map
            and story_board_element.state == 2  # 2 is a running state
        ):
            self.handle_custom_command_action(story_board_element)

    def handle_follow_trajectory_action(self, story_board_element):
        # Iterate through active objects to send the start command for the target objects
        for object_id, object in self.active_objects.items():
            target_object_name = self.follow_traj_to_obj_name[story_board_element.name]
            if object.name in target_object_name:

                self.get_logger().info(
                    f"Starting object {object.name} with id {object_id}"
                )
                start_object_msg = atos_interfaces.msg.ObjectTriggerStart()
                start_object_msg.id = object_id
                self.start_object_pub_.publish(start_object_msg)
                break

    def handle_custom_command_action(self, story_board_element):
        self.get_logger().info("Custom command action received")
        custom_command = self.custom_command_map[story_board_element.name]
        if custom_command.type == "V2X":
            self.get_logger().info("Sending V2X message")
            # Send V2X message
            self.get_logger().info(custom_command.content)

    def parameter_callback(self, params):
        for param in params:
            if param.name == SCENARIO_FILE_PARAMETER and param.value:
                self.update_scenario(file_name=param.value)
            elif param.name == ACTIVE_OBJECT_NAME_PARAMETER:
                self.update_active_scenario_objects(active_objects_name=param.value)
        return SetParametersResult(successful=True)

    def update_scenario(self, file_name):
        scenario_file = self.getAbsoluteOSCPath(file_name)
        # Check if the file exists, else throw an error
        if not path.exists(scenario_file):
            self.get_logger().error("File does not exist: {}".format(scenario_file))
            return
        self.get_logger().info("Loading scenario file: {}".format(scenario_file))
        self.follow_traj_to_obj_name = StoryBoardHandler(
            scenario_file
        ).get_follow_trajectory_actions_to_actors_map()
        self.custom_command_map = StoryBoardHandler(
            self.scenario_file
        ).get_custom_command_actions_map()

    def update_active_scenario_objects(self, active_objects_name: List[str]):
        if len(active_objects_name) != len(set(active_objects_name)):
            self.get_logger().error("Active object names contain duplicates")
        scenario_objects = self.get_all_objects_in_scenario(self.getScenarioFilePath())

        # Remove any object that are (now) inactive
        for id, obj in self.active_objects.items():
            if obj.name not in active_objects_name:
                self.active_objects.pop(id)

        # Add objects that are active
        for id, obj in scenario_objects.items():
            if obj.name in active_objects_name and id not in self.active_objects:
                self.active_objects[id] = obj
                # Update the ip property of the object unless it is already set
                if obj.ip is None:
                    obj.ip = self.get_ip_property_for_object(obj)
                self.get_logger().info(
                    "Scenario object {} is assigned transmitter id: {}".format(
                        obj.name, id
                    )
                )

    def get_all_objects_in_scenario(self, scenario_file) -> List[ScenarioObject]:
        scenario = xosc.ParseOpenScenario(scenario_file)
        scenario_objects = {
            id
            + 1: ScenarioObject(
                name=scenario_object.name,
                catalog_ref=xosc.CatalogReference(
                    scenario_object.entityobject.catalogname,
                    scenario_object.entityobject.entryname,
                ),
            )
            for id, scenario_object in enumerate(scenario.entities.scenario_objects)
        }
        return scenario_objects

    def get_ip_property_for_object(self, obj: ScenarioObject) -> str:
        properties = self.get_properties_for_object(obj)
        # Convert the list of tuples to a dict to get the ip property easily
        return dict(properties)["ip"]

    def get_properties_for_object(self, obj: ScenarioObject):
        catalog_path = xosc.ParseOpenScenario(
            self.getScenarioFilePath()
        ).catalog.catalogs.get(obj.catalog_ref.catalogname)
        catalog_object = xosc.xosc_reader.CatalogReader(
            obj.catalog_ref,
            self.getAbsoluteOSCPath(catalog_path),
        )
        return catalog_object.properties.properties

    def srv_get_object_id_array(self, request, response):
        for id, object in self.active_objects.items():
            response.ids.append(id)
            response.names.append(object.name)
        response.success = True
        return response

    def srv_get_object_ip(self, request, response):
        scenario_object = self.active_objects.get(request.id)
        if scenario_object is None:
            response.success = False
            return response
        response.ip = scenario_object.ip
        response.success = True
        return response

    def srv_set_object_ip(self, request, response):
        scenario_object = self.active_objects.get(request.id)
        if scenario_object is None:
            response.success = False
            return response
        scenario_object.ip = request.ip
        response.success = True
        return response

    def srv_get_open_scenario_file_path(self, request, response):
        response.path = self.getScenarioFilePath()
        response.md5hash = self.calcMD5HashForFile(self.getScenarioFilePath())
        response.success = True
        return response

    def getScenarioFilePath(self) -> str:
        return self.getAbsoluteOSCPath(
            self.get_parameter(SCENARIO_FILE_PARAMETER).value
        )

    def getAbsoluteOSCPath(self, file_name: str) -> str:
        return path.join(
            self.get_parameter(ROOT_FOLDER_PATH_PARAMETER).value, "osc", file_name
        )

    def calcMD5HashForFile(self, file_path):
        hash_md5 = hashlib.md5()
        try:
            with open(file_path, "rb") as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    hash_md5.update(chunk)
            return hash_md5.hexdigest()
        except FileNotFoundError:
            return "File not found"
        except Exception as e:
            return str(e)


def main(args=None):
    rclpy.init(args=args)

    open_scenario_gw = OpenScenarioGateway()

    rclpy.spin(open_scenario_gw)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    open_scenario_gw.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
