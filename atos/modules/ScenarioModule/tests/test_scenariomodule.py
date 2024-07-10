import pytest
import rclpy

import modules.ScenarioModule.scenariomodule as sm
import atos_interfaces.srv as srv
import rclpy.parameter


@pytest.fixture(scope="module")
def scenario_module():
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
    yield scenario_module
    rclpy.shutdown()


def test_get_six_objects_in_scenario(get_file_path, scenario_module):
    scenario_path = get_file_path("../../conf/osc/GaragePlanScenario.xosc")
    objects = scenario_module.get_all_objects_in_scenario(scenario_path)
    assert len(objects) == 6


def test_get_object_ids(scenario_module):
    request = srv.GetObjectIds.Request()
    response = srv.GetObjectIds.Response()
    scenario_module.srv_get_object_id_array(request, response)
    assert list(response.ids) == [1, 2]
    assert response.success == True


def test_get_object_ip(scenario_module):
    request = srv.GetObjectIp.Request(id=1)
    response = srv.GetObjectIp.Response()
    scenario_module.srv_get_object_ip(request, response)
    assert response.ip == "127.0.0.1"
    assert response.success == True


def test_set_object_ip(scenario_module):
    request = srv.SetObjectIp.Request(id=1, ip="6.6.6.6")
    response = srv.SetObjectIp.Response()
    scenario_module.srv_set_object_ip(request, response)
    assert response.success == True

    request = srv.GetObjectIp.Request(id=1)
    response = srv.GetObjectIp.Response()
    scenario_module.srv_get_object_ip(request, response)
    assert response.ip == "6.6.6.6"
    assert response.success == True


def test_ip_is_still_user_defined_after_init(scenario_module):
    request = srv.SetObjectIp.Request(id=1, ip="6.6.6.6")
    response = srv.SetObjectIp.Response()
    scenario_module.srv_set_object_ip(request, response)
    assert response.success == True

    scenario_module.init_callback(None)

    request = srv.GetObjectIp.Request(id=1)
    response = srv.GetObjectIp.Response()
    scenario_module.srv_get_object_ip(request, response)
    assert response.ip == "6.6.6.6"
    assert response.success == True
