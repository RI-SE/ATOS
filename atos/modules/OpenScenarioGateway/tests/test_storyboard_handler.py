import os
import pytest
import modules.OpenScenarioGateway.storyboard_handler as sh


def test_init_storyboard_handler(get_file_path):
    scenario_path = os.path.join(
        os.path.dirname(__file__), "resources", "osc", "GaragePlanScenario.xosc"
    )
    storyboard_handler = sh.StoryBoardHandler(scenario_path)
    assert storyboard_handler


def test_get_follow_trajectory_actions_to_actors_map(get_file_path):
    scenario_path = os.path.join(
        os.path.dirname(__file__), "resources", "osc", "GaragePlanScenario.xosc"
    )
    storyboard_handler = sh.StoryBoardHandler(scenario_path)

    # Call the method under test
    result = storyboard_handler.get_follow_trajectory_actions_to_actors_map()

    # Assert the expected result
    expected_result = {
        "1,init_follow_trajectory": ["1"],
        "2,start_follow_trajectory": ["2"],
        "3,start_follow_trajectory": ["3"],
        "4,start_follow_trajectory": ["4"],
        "5,start_follow_trajectory": ["5"],
        "6,start_follow_trajectory": ["6"],
    }
    assert result == expected_result


def test_denm_custom_command_action(get_file_path):
    scenario_path = os.path.join(
        os.path.dirname(__file__), "resources", "osc", "GaragePlanScenario.xosc"
    )
    storyboard_handler = sh.StoryBoardHandler(scenario_path)

    # Call the method under test
    result = storyboard_handler.get_custom_command_actions_map()

    # Assert the expected result
    expected_result = {
        "1,send_denm": {
            "type": "V2X",
            "content": '{"message_type": "DENM", "event_id": "ATOSEvent1", "cause_code": 12, "latitude": 0.0, "longitude": 0.0, "altitude": 0.0, "detection_time": 0}',
        }
    }
    assert result == expected_result
