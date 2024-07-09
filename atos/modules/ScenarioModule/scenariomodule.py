#!/usr/bin/env python3

import atos_interfaces.srv
from os import path
import rclpy
import rclpy.logging
import atos_interfaces.msg
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Empty
from scenariogeneration import xosc
from typing import List


ATOS_XOSC_DIR = path.expanduser("~/.astazero/ATOS/osc/")
ACTIVE_OBJECT_NAME_PARAMETER = "active_object_names"
SCENARIO_FILE_PARAMETER = "open_scenario_file"


class ScenarioObject:
    def __init__(self, name: str, catalog_ref: xosc.CatalogReference):
        self.name = name
        self.catalog_ref: xosc.CatalogReference = catalog_ref
        self.id: int
        self.ip: str


class ScenarioModule(Node):

    def __init__(self):
        super().__init__("scenario_module")
        self.init_subscription_ = self.create_subscription(
            Empty, "init", self.init_callback, 10
        )
        self.object_ids_pub_ = self.create_service(
            atos_interfaces.srv.GetObjectIds,
            "get_object_ids",
            self.srv_get_object_id_array,
        )
        self.get_object_ip_ = self.create_service(
            atos_interfaces.srv.GetObjectIp, "get_object_ip", self.srv_get_object_ip
        )

        self.declare_parameter(SCENARIO_FILE_PARAMETER, "")
        self.declare_parameter(
            ACTIVE_OBJECT_NAME_PARAMETER, rclpy.Parameter.Type.STRING_ARRAY
        )
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.scenario_objects = []
        self.vehicle_catalog = None

    def init_callback(self, msg):
        self.get_logger().info("Init callback called")

    def parameter_callback(self, params):
        for param in params:
            if param.name == SCENARIO_FILE_PARAMETER and param.value:
                self.update_scenario(file_name=param.value)
            elif param.name == ACTIVE_OBJECT_NAME_PARAMETER:
                self.update_active_scenario_objects(active_objects=param.value)
        return SetParametersResult(successful=True)

    def update_scenario(self, file_name: str):
        self.scenario_file = path.join(ATOS_XOSC_DIR, file_name)
        self.get_logger().info("Loading scenario file: {}".format(self.scenario_file))

    def update_active_scenario_objects(self, active_objects):
        if len(active_objects) != len(set(active_objects)):
            raise ValueError("Names must be unique")
        self.scenario_objects = self.get_all_objects_in_scenario(self.scenario_file)
        for name in active_objects:
            if name not in [obj.name for obj in self.scenario_objects]:
                raise ValueError("Name {} not found in scenario file".format(name))
        # Set an unique id and get IP for each object
        for i, obj in enumerate(self.scenario_objects):
            obj.id = i + 1
            self.get_logger().info(
                "Assigning ID {} to object with name {}".format(obj.id, obj.name)
            )
            obj.ip = self.get_ip_property_for_object(obj)

    def get_all_objects_in_scenario(self, scenario_file) -> List[ScenarioObject]:
        scenario = xosc.ParseOpenScenario(scenario_file)
        scenario_objects = [
            ScenarioObject(
                name=scenario_object.name,
                catalog_ref=xosc.CatalogReference(
                    scenario_object.entityobject.catalogname,
                    scenario_object.entityobject.entryname,
                ),
            )
            for scenario_object in scenario.entities.scenario_objects
        ]
        return scenario_objects

    def get_ip_property_for_object(self, obj: ScenarioObject) -> str:
        catalog_path = xosc.ParseOpenScenario(self.scenario_file).catalog.catalogs.get(
            obj.catalog_ref.catalogname
        )
        catalog_object = xosc.xosc_reader.CatalogReader(
            obj.catalog_ref, path.join(ATOS_XOSC_DIR, catalog_path)
        )
        # Convert the list of tuples to a dict to get the ip property easily
        return dict(catalog_object.properties.properties)["ip"]

    def srv_get_object_id_array(self, request, response):
        self.get_logger().info("Service get_object_id_array callback called")
        active_object_names = self.get_parameter(ACTIVE_OBJECT_NAME_PARAMETER).value
        ids = [
            obj.id for obj in self.scenario_objects if obj.name in active_object_names
        ]
        response.ids = ids
        response.success = True
        return response

    def srv_get_object_ip(self, request, response):
        id = request.id
        response.ip = [obj.ip for obj in self.scenario_objects if obj.id == id][0]


def main(args=None):
    rclpy.init(args=args)

    scenario_module = ScenarioModule()

    rclpy.spin(scenario_module)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scenario_module.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
