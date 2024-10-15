import os
import pytest
import modules.OpenScenarioGateway.storyboard_handler as sh


def test_init_storyboard_handler(get_file_path):
    scenario_path = os.path.join(
        os.path.dirname(__file__), "resources", "osc", "GaragePlanScenario.xosc"
    )
    storyboard_handler = sh.StoryBoardHandler(scenario_path)
    assert storyboard_handler


def test_get_start_actions_to_actors_map(get_file_path):
    scenario_path = os.path.join(
        os.path.dirname(__file__), "resources", "osc", "GaragePlanScenario.xosc"
    )
    storyboard_handler = sh.StoryBoardHandler(scenario_path)

    # Call the method under test
    result = storyboard_handler.get_follow_trajectory_actions_to_actors_map()

    # Assert the expected result
    expected_result = {
        "1,init_follow_trajectory": ["1"],
        "1,slow_down": ["1"],
        "1,mondeo_accelerate": ["1"],
        "1,brake_to_stop": ["1"],
        "5,brake_to_stop": ["5"],
        "5,start_follow_trajectory": ["5"],
        "5,set_speed": ["5"],
        "2,brake_to_stop": ["2"],
        "2,start_follow_trajectory": ["2"],
        "2,set_speed": ["2"],
        "4,brake_to_stop": ["4"],
        "4,start_follow_trajectory": ["4"],
        "4,set_speed": ["4"],
        "3,brake_to_stop": ["3"],
        "3,start_follow_trajectory": ["3"],
        "3,set_speed": ["3"],
        "6,brake_to_stop": ["6"],
        "6,start_follow_trajectory": ["6"],
        "6,set_speed": ["6"],
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
        "story_start::start::1,maneuver_group::1,maneuver::1,high_speed_event::1,send_denm": sh.CustomCommandAction(
            type="V2X",
            content='{"message_type": "DENM", "event_id": "ATOSEvent1", "cause_code": 12, "latitude": 0.0, "longitude": 0.0, "altitude": 0.0, "detection_time": 0}',
        ),
        "story_start::start::1,maneuver_group::1,maneuver::1,high_speed_event_2::1,send_denm_2": sh.CustomCommandAction(
            type="V2X",
            content='{"message_type": "DENM", "event_id": "ATOSEvent2", "cause_code": 12, "latitude": 0.0, "longitude": 0.0, "altitude": 0.0, "detection_time": 0}',
        ),
    }

    assert result == expected_result
