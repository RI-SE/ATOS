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


class ScenarioObject:
    def __init__(self, name: str):
        self.name = name
        self.id: int = None
        self.ip = None


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
        # self.get_object_ip_ = self.create_service(
        #     atos_interfaces.srv.GetObjectIp, "get_object_ip", self.srv_get_object_ip
        # )

        self.declare_parameter("open_scenario_file", "")
        self.declare_parameter("active_object_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.scenario_objects = []
        self.vehicle_catalog = None

    def init_callback(self, msg):
        self.get_logger().info("Init callback called")

    def parameter_callback(self, params):
        for param in params:
            if param.name == "open_scenario_file":
                self.scenario_file = path.join(ATOS_XOSC_DIR, param.value)
                self.get_logger().info(
                    "Loading scenario file: {}".format(self.scenario_file)
                )
                vehicle_catalog_path = xosc.ParseOpenScenario(
                    self.scenario_file
                ).catalog.catalogs.get("VehicleCatalog")
                if vehicle_catalog_path:
                    self.vehicle_catalog = xosc.ParseOpenScenario(
                        path.join(
                            ATOS_XOSC_DIR, vehicle_catalog_path, "VehicleCatalog.xosc"
                        )
                    )
            elif param.name == "active_object_names":
                # Check so that the names are unique and they exist in the scenario file
                names = param.value
                if len(names) != len(set(names)):
                    raise ValueError("Names must be unique")
                self.scenario_objects = self.get_all_objects_in_scenario(
                    self.scenario_file
                )
                for name in names:
                    if name not in [obj.name for obj in self.scenario_objects]:
                        raise ValueError(
                            "Name {} not found in scenario file".format(name)
                        )
                # Set an unique id and get IP for each object
                for i, obj in enumerate(self.scenario_objects):
                    obj.id = i + 1
        return SetParametersResult(successful=True)

    # def get_ip_from_name(self, name: str) -> str:
    #     scenario_objects = xosc.ParseOpenScenario(
    #         self.scenario_file
    #     ).entities.scenario_objects
    #     for obj in scenario_objects:
    #         if name == obj.name:
    #             et = obj.get_element()
    #             print(et)
    #             # Print the entire element tree to see what is available
    #             for e in et.iter():
    #                 print(e.tag, e.attrib)

    def get_all_objects_in_scenario(self, scenario_file) -> List[ScenarioObject]:
        scenario = xosc.ParseOpenScenario(scenario_file)
        scenario_objects = []
        try:
            scenario_objects = [
                ScenarioObject(name=scenario_object.name)
                for scenario_object in scenario.entities.scenario_objects
            ]
        except AttributeError:
            raise AttributeError(
                'Missing "name" attribute for ScenarioObject in scenario file {}'.format(
                    scenario_file
                )
            )
        return scenario_objects

    def srv_get_object_id_array(self, request, response):
        active_object_names = self.get_parameter("active_object_names").value
        response.ids = [
            obj.id for obj in self.scenario_objects if obj.name in active_object_names
        ]
        response.success = True

    # def srv_get_object_ip(
    #     self, request, response
    # ) -> atos_interfaces.srv.GetObjectIp.Response:
    #     object_id = request.id


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
