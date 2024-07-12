import os
import pytest
import modules.ScenarioModule.storyboard_handler as sh


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
