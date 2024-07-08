#!/usr/bin/env python3

import atos_interfaces.srv
from os import path
import rclpy
import rclpy.logging
import atos_interfaces.msg
from rclpy.node import Node
from std_msgs.msg import Empty
from scenariogeneration import xosc

ATOS_DIR = "~/.astazero/ATOS/osc/"


class ScenarioModule(Node):

    def __init__(self):
        super().__init__("scenario_module")
        self.init_subscription_ = self.create_subscription(
            Empty, "init", self.init_callback, 10
        )
        self.object_ids_pub_ = self.create_service(
            atos_interfaces.srv.GetObjectIds, "get_object_ids", self.getObjectIdArray
        )

        self.declare_parameter("open_scenario_file", "")
        self.declare_parameter("active_object_ids", rclpy.Parameter.Type.INTEGER_ARRAY)

    def init_callback(self, msg):
        self.get_logger().info("Init callback called")

    def get_objects_in_scenario(self, scenario_file):
        scenario = xosc.ParseOpenScenario(scenario_file)
        scenario_objects = scenario.entities.scenario_objects
        try:
            names = [scenario_object.name for scenario_object in scenario_objects]
        except AttributeError:
            raise AttributeError(
                'Missing "name" attribute for ScenarioObject in scenario file {}'.format(
                    scenario_file
                )
            )
        return names

    def getObjectIdArray(self, request, response):
        object_ids = (
            self.get_parameter("active_object_ids")
            .get_parameter_value()
            .integer_array_value.tolist()
        )
        response.ids = object_ids
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
