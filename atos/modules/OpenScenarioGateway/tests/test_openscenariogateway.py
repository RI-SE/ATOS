import os
import pytest
import rclpy

import modules.OpenScenarioGateway.openscenariogateway as openxgw
import atos_interfaces.srv as srv
import rclpy.parameter


@pytest.fixture(scope="module")
def open_scenario_gw():
    rclpy.init()
    active_objects = ["1", "2"]
    open_scenario_gw = openxgw.OpenScenarioGateway()
    open_scenario_gw.set_parameters(
        [
            rclpy.parameter.Parameter(
                "root_folder_path",
                rclpy.Parameter.Type.STRING,
                os.path.join(os.path.dirname(__file__), "resources"),
            ),
            rclpy.parameter.Parameter(
                "open_scenario_file",
                rclpy.Parameter.Type.STRING,
                "GaragePlanScenario.xosc",
            ),
            rclpy.parameter.Parameter(
                "active_object_names", rclpy.Parameter.Type.STRING_ARRAY, active_objects
            ),
        ]
    )
    yield open_scenario_gw
    rclpy.shutdown()


def test_get_six_objects_in_scenario(get_file_path, open_scenario_gw):
    scenario_path = os.path.join(
        os.path.dirname(__file__), "resources", "osc", "GaragePlanScenario.xosc"
    )
    objects = open_scenario_gw.get_all_objects_in_scenario(scenario_path)
    assert len(objects) == 6


def test_get_object_ids(open_scenario_gw):
    request = srv.GetObjectIds.Request()
    response = srv.GetObjectIds.Response()
    open_scenario_gw.srv_get_object_id_array(request, response)
    assert list(response.ids) == [1, 2]
    assert list(response.names) == ["1", "2"]
    assert response.success == True


def test_get_object_ip(open_scenario_gw):
    request = srv.GetObjectIp.Request(id=1)
    response = srv.GetObjectIp.Response()
    open_scenario_gw.srv_get_object_ip(request, response)
    assert response.ip == "127.0.0.1"
    assert response.success == True


def test_set_object_ip(open_scenario_gw):
    request = srv.SetObjectIp.Request(id=1, ip="6.6.6.6")
    response = srv.SetObjectIp.Response()
    open_scenario_gw.srv_set_object_ip(request, response)
    assert response.success == True

    request = srv.GetObjectIp.Request(id=1)
    response = srv.GetObjectIp.Response()
    open_scenario_gw.srv_get_object_ip(request, response)
    assert response.ip == "6.6.6.6"
    assert response.success == True


def test_ip_is_still_user_defined_after_init(open_scenario_gw):
    request = srv.SetObjectIp.Request(id=1, ip="6.6.6.6")
    response = srv.SetObjectIp.Response()
    open_scenario_gw.srv_set_object_ip(request, response)
    assert response.success == True

    open_scenario_gw.init_callback(None)

    request = srv.GetObjectIp.Request(id=1)
    response = srv.GetObjectIp.Response()
    open_scenario_gw.srv_get_object_ip(request, response)
    assert response.ip == "6.6.6.6"
    assert response.success == True


def test_srv_get_open_scenario_file_path(open_scenario_gw):
    request = srv.GetOpenScenarioFilePath.Request()
    response = srv.GetOpenScenarioFilePath.Response()
    open_scenario_gw.srv_get_open_scenario_file_path(request, response)
    assert response.path == os.path.join(
        os.path.dirname(__file__), "resources", "osc", "GaragePlanScenario.xosc"
    )
    assert response.success == True
