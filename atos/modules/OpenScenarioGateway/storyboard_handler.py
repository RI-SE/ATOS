from scenariogeneration import xosc


class CustomCommandAction:
    def __init__(self, type: str, content: str):
        self.type = type
        self.content = content


class StoryBoardHandler:

    def __init__(self, scenario_file: str):
        self.scenario_file = scenario_file
        self.xosc = xosc.ParseOpenScenario(scenario_file)
        self.follow_trajectory_action_actor_map_ = {}
        self.custom_command_action_map_ = {}
        self.collect_action_maps()

    def collect_action_maps(self):
        follow_trajectory_actions = {}
        custom_command_actions = {}
        for story in self.xosc.storyboard.stories:
            for act in story.acts:
                for manuever_group in act.maneuvergroup:
                    actors = [actor.entity for actor in manuever_group.actors.actors]
                    for manuever in manuever_group.maneuvers:
                        for event in manuever.events:
                            for action in event.action:
                                if isinstance(
                                    action.action, xosc.actions.FollowTrajectoryAction
                                ):
                                    follow_trajectory_actions[action.name] = actors
                                elif (
                                    isinstance(
                                        action.action, xosc.actions.UserDefinedAction
                                    )
                                    and action.action.custom_command_action is not None
                                ):
                                    cc_action = action.action.custom_command_action
                                    custom_command_actions[action.name] = (
                                        CustomCommandAction(
                                            type=cc_action.type,
                                            content=cc_action.content,
                                        )
                                    )

        self.follow_trajectory_action_actor_map_ = follow_trajectory_actions
        self.custom_command_action_map_ = custom_command_actions

    def get_follow_trajectory_actions_to_actors_map(self):
        return self.follow_trajectory_action_actor_map_

    def get_custom_command_actions_map(self):
        return self.custom_command_action_map_
