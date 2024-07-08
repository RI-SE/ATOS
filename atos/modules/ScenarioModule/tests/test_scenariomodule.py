import modules.ScenarioModule.scenariomodule as sm
import atos_interfaces.srv as srv
import rclpy
import rclpy.parameter


def test_get_objects_in_scenario(get_file_path):
    rclpy.init()
    object_names = ["1", "2", "4", "3", "6", "5"]
    scenario_module = sm.ScenarioModule()
    scenario_path = get_file_path("../../conf/osc/GaragePlanScenario.xosc")
    print(scenario_path)
    object_ids = scenario_module.get_objects_in_scenario(scenario_path)
    assert object_ids == object_names

    rclpy.shutdown()


def test_get_object_ids():
    rclpy.init()
    active_objects = ["1"]
    scenario_module = sm.ScenarioModule()
    scenario_module.set_parameters(
        [
            rclpy.parameter.Parameter(
                "open_scenario_file",
                rclpy.Parameter.Type.STRING,
                "GaragePlanScenario.xosc",
            )
        ]
    )
    scenario_module.set_parameters(
        [
            rclpy.parameter.Parameter(
                "active_object_names", rclpy.Parameter.Type.STRING_ARRAY, active_objects
            )
        ]
    )
    request = srv.GetObjectIds.Request()
    response = srv.GetObjectIds.Response()
    scenario_module.get_object_id_array(request, response)
    assert response.success == True
    assert list(response.ids) == [1]
    rclpy.shutdown()
