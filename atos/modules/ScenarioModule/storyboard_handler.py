from scenariogeneration import xosc


class StoryBoardHandler:
    def __init__(self, scenario_file: str):
        self.scenario_file = scenario_file
        self.xosc = xosc.ParseOpenScenario(scenario_file)
        self.follow_trajectory_actions_actor_map = (
            self.collect_follow_trajectory_actions_to_actors_map()
        )

    def collect_follow_trajectory_actions_to_actors_map(self):
        follow_trajectory_actions = {}
        for story in self.xosc.storyboard.stories:
            # Check if the element is a maneuver
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
        return follow_trajectory_actions

    def get_follow_trajectory_actions_to_actors_map(self):
        return self.follow_trajectory_actions_actor_map
