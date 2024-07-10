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
        self.ip: str = None


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
        self.get_object_ip_ = self.create_service(
            atos_interfaces.srv.SetObjectIp, "set_object_ip", self.srv_set_object_ip
        )

        self.declare_parameter(SCENARIO_FILE_PARAMETER, "")
        self.declare_parameter(
            ACTIVE_OBJECT_NAME_PARAMETER, rclpy.Parameter.Type.STRING_ARRAY
        )
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.active_objects = {}
        self.vehicle_catalog = None

    def init_callback(self, msg):
        self.get_logger().info("Init callback called")
        self.update_scenario(self.get_parameter(SCENARIO_FILE_PARAMETER).value)
        self.update_active_scenario_objects(
            self.get_parameter(ACTIVE_OBJECT_NAME_PARAMETER).value
        )

    def parameter_callback(self, params):
        for param in params:
            if param.name == SCENARIO_FILE_PARAMETER and param.value:
                self.update_scenario(file_name=param.value)
            elif param.name == ACTIVE_OBJECT_NAME_PARAMETER:
                self.update_active_scenario_objects(active_objects_name=param.value)
        return SetParametersResult(successful=True)

    def update_scenario(self, file_name: str):
        self.scenario_file = path.join(ATOS_XOSC_DIR, file_name)
        self.get_logger().info("Loading scenario file: {}".format(self.scenario_file))

    def update_active_scenario_objects(self, active_objects_name: List[str]):
        if len(active_objects_name) != len(set(active_objects_name)):
            raise ValueError("Names must be unique")
        scenario_objects = self.get_all_objects_in_scenario(self.scenario_file)

        # Remove any object that have (now) inactive
        for id, obj in self.active_objects.items():
            if obj.name not in active_objects_name:
                self.active_objects.pop(id)

        # Add any object that have become active
        for id, obj in scenario_objects.items():
            if obj.name in active_objects_name and id not in self.active_objects:
                self.active_objects[id] = obj
                # Update the ip property of the object unless it is already set
                if obj.ip is None:
                    obj.ip = self.get_ip_property_for_object(obj)

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
        catalog_path = xosc.ParseOpenScenario(self.scenario_file).catalog.catalogs.get(
            obj.catalog_ref.catalogname
        )
        catalog_object = xosc.xosc_reader.CatalogReader(
            obj.catalog_ref, path.join(ATOS_XOSC_DIR, catalog_path)
        )
        return catalog_object.properties.properties

    def srv_get_object_id_array(self, request, response):
        self.get_logger().info("Service get_object_id_array callback called")
        ids = [id for id, _ in self.active_objects.items()]
        response.ids = ids
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
