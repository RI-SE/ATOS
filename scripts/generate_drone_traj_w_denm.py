from scenariogeneration import esmini
from scenariogeneration import xosc

class SendDenmAction(xosc.CustomCommandAction):
    def __init__(self, name, type):
        self.name = name    
        self.type = type

# Define drone trajectory
drone_at_pos_times = [2, 4, 6, 8, 10, 12, 13, 14, 15,
                      16, 17, 19, 21, 23, 25, 27, 29, 31]
drone_speed = 2.5  # m/s
drone_acceleration = 0.8
drone_positions = [
    xosc.WorldPosition(0, 0, 5, 0, 0, 0),
    xosc.WorldPosition(1, -3, 5, 0, 0, 0),
    xosc.WorldPosition(2, -6, 5, 0, 0, 0),
    xosc.WorldPosition(3, -9, 5, 0, 0, 0),
    xosc.WorldPosition(4, -12, 5, 0, 0, 0),
    xosc.WorldPosition(4, -15, 5, 0, 0, 0),
    xosc.WorldPosition(5, -15, 5, 0, 0, 0),
    xosc.WorldPosition(6, -15, 5, 0, 0, 0),
    xosc.WorldPosition(7, -15, 5, 0, 0, 0),
    xosc.WorldPosition(8, -15, 5, 0, 0, 0),
    xosc.WorldPosition(9, -15, 5, 0, 0, 0),
    xosc.WorldPosition(10, -18, 5, 0, 0, 0),
    xosc.WorldPosition(11, -21, 5, 0, 0, 0),
    xosc.WorldPosition(12, -24, 5, 0, 0, 0),
    xosc.WorldPosition(13, -27, 5, 0, 0, 0),
    xosc.WorldPosition(14, -30, 5, 0, 0, 0),
    xosc.WorldPosition(15, -33, 5, 0, 0, 0),
    xosc.WorldPosition(16, -36, 5, 0, 0, 0)
]


def get_drone_maneuver():
    osc_drone_maneuver_group = xosc.ManeuverGroup(
        str(DRONE_ID) + "_maneuver_group")

    osc_teleport_action = xosc.TeleportAction(drone_positions[0])  # xyzhpr
    osc_absolute_speed_action = xosc.AbsoluteSpeedAction(drone_speed, xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear, xosc.DynamicsDimension.time, drone_acceleration),)
    init.add_init_action(str(DRONE_ID), osc_teleport_action)
    init.add_init_action(str(DRONE_ID), osc_absolute_speed_action)

    polyline = xosc.Polyline(drone_at_pos_times, drone_positions)
    trajectory = xosc.Trajectory(str(DRONE_ID) + "_trajectory", closed=False)
    trajectory.add_shape(polyline)

    osc_drone_maneuver = xosc.Maneuver(str(DRONE_ID) + "_maneuver")

    ### Start event ###
    osc_follow_trajectory = xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowMode.position,
        xosc.ReferenceContext.relative,
        1,
        0
    )
    start_osc_trigger = xosc.ValueTrigger(
        str(DRONE_ID) + "_start_trigger",
        0,
        xosc.ConditionEdge.none,
        xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan)
    )
    start_osc_event = xosc.Event(
        str(DRONE_ID) + "_start_event",
        xosc.Priority.overwrite,
    )

    start_osc_event.add_trigger(start_osc_trigger)
    start_osc_event.add_action(str(DRONE_ID) + ",follow_trajectory",
                               osc_follow_trajectory)
    osc_drone_maneuver.add_event(start_osc_event)
    ########

    ###  Stop event ###
    osc_absolute_speed_action = xosc.AbsoluteSpeedAction(
        0,
        xosc.TransitionDynamics(
            xosc.DynamicsShapes.linear,
            xosc.DynamicsDimension.time,
            -drone_acceleration
        ),
    )
    stop_osc_trigger = xosc.EntityTrigger(
        name=str(DRONE_ID) + "_stop_trigger",
        delay=0,
        conditionedge=xosc.ConditionEdge.rising,
        entitycondition=xosc.ReachPositionCondition(
            drone_positions[-1], 1.0),  # tolerance
        triggerentity=str(DRONE_ID)
    )
    stop_osc_event = xosc.Event(
        str(DRONE_ID) + "_stop_event",
        xosc.Priority.overwrite,
    )

    stop_osc_event.add_trigger(stop_osc_trigger)
    stop_osc_event.add_action(str(DRONE_ID) + "_stop",
                              osc_absolute_speed_action)
    osc_drone_maneuver.add_event(stop_osc_event)

    ### DENM Action ###
    denm_osc_trigger = xosc.EntityTrigger(
        name=str(DRONE_ID) + "position_trigger",
        delay=0,
        conditionedge=xosc.ConditionEdge.none,
        entitycondition=xosc.ReachPositionCondition(
            drone_positions[int(len(drone_positions)/2)], 1.0),
        triggerentity=str(DRONE_ID))

    osc_dummy_action = xosc.VisibilityAction(graphics=True, traffic=True, sensors=True)

    denm_osc_event = xosc.Event(
        str(DRONE_ID) + "_denm_event",
        xosc.Priority.parallel,
    )

    denm_osc_event.add_trigger(denm_osc_trigger)
    denm_osc_event.add_action(str(DRONE_ID) + ",send_denm",
                              osc_dummy_action)
    osc_drone_maneuver.add_event(denm_osc_event)

    osc_drone_maneuver_group.add_actor(str(DRONE_ID))
    osc_drone_maneuver_group.add_maneuver(osc_drone_maneuver)
    return osc_drone_maneuver_group


if __name__ == "__main__":
    odr_path = "../odr/GaragePlanSouth.xodr"
    vehicle_catalog_path = "../Catalogs/Vehicles"
    controllers_catalog_path = "../Catalogs/Controllers"
    output_filename = "DroneScenario.xosc"
    DRONE_ID = 5
    DENM_ID = 0

    init = xosc.Init()
    osc_params = xosc.ParameterDeclarations()
    osc_catalogs = xosc.Catalog()
    osc_catalogs.add_catalog('ControllerCatalog', controllers_catalog_path)
    osc_catalogs.add_catalog('VehicleCatalog', vehicle_catalog_path)
    osc_road_network = xosc.RoadNetwork(roadfile=odr_path)
    osc_entities = xosc.Entities()
    osc_storyboard = xosc.StoryBoard(init)
    osc_act = xosc.Act("start")

    # Add drone to scenario
    osc_entities = osc_entities.add_scenario_object(
        str(DRONE_ID), xosc.CatalogReference('VehicleCatalog', 'camera_drone'))

    osc_act.add_maneuver_group(get_drone_maneuver())
    osc_storyboard.add_act(osc_act)

    osc_file = xosc.Scenario(
        name="OpenScenario",
        author="AstaZero 1.0",
        parameters=osc_params,
        entities=osc_entities,
        storyboard=osc_storyboard,
        roadnetwork=osc_road_network,
        catalog=osc_catalogs
    )

    osc_file.write_xml(output_filename)
