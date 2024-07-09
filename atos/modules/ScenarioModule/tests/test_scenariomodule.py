import modules.ScenarioModule.scenariomodule as sm
import atos_interfaces.srv as srv
import rclpy
import rclpy.parameter


def test_get_six_objects_in_scenario(get_file_path):
    rclpy.init()
    scenario_module = sm.ScenarioModule()
    scenario_path = get_file_path("../../conf/osc/GaragePlanScenario.xosc")
    objects = scenario_module.get_all_objects_in_scenario(scenario_path)
    assert len(objects) == 6

    rclpy.shutdown()


def test_get_object_ids():
    rclpy.init()
    active_objects = ["1", "2"]
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
    scenario_module.srv_get_object_id_array(request, response)
    assert list(response.ids) == [1, 2]
    assert response.success == True
    rclpy.shutdown()


def test_get_object_ip():
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
    request = srv.GetObjectIp.Request(id=1)
    response = srv.GetObjectIp.Response()
    scenario_module.srv_get_object_ip(request, response)
    assert response.ip == "127.0.0.1"
    assert response.success == True
    rclpy.shutdown()
