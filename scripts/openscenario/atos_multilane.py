from scenariogeneration import xosc

def create_scenario(opendrive_path):
    init = xosc.Init()
    osc_params = xosc.ParameterDeclarations()
    osc_catalogs = xosc.Catalog()
    osc_catalogs.add_catalog("VehicleCatalog", "../Catalogs/Vehicles")
    osc_road_network = xosc.RoadNetwork(roadfile=opendrive_path)
    osc_entities = xosc.Entities()
    osc_storyboard = xosc.StoryBoard(init)
    osc_act = xosc.Act("start")

    UFO_ID = "O2"
    CARRIER_ID = "O3"
    DRONE_ID = "O5"

    # -------------- CARRIER ----------------
    osc_entities.add_scenario_object(CARRIER_ID,
                                     xosc.CatalogReference("VehicleCatalog", "car_white"))

    carrier_start = xosc.TeleportAction(xosc.LanePosition(30, 0, -1, 0))
    carrier_speed = xosc.AbsoluteSpeedAction(
            40/3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                1.51
            ),
        )

    init.add_init_action(CARRIER_ID, carrier_start)
    init.add_init_action(CARRIER_ID, carrier_speed)

    # Create Maneuver Group for the Carrier.
    carrier_maneuver_group = xosc.ManeuverGroup(CARRIER_ID + "_maneuver_group")
    # Create the Maneuver for the Carrier.
    carrier_maneuver = xosc.Maneuver(CARRIER_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(30, 0, -1, 0),
        xosc.LanePosition(50, 0, -1, 0),
        xosc.LanePosition(70, 0, -1, 0),
        xosc.LanePosition(90, 0, -1, 0),
        xosc.LanePosition(110, 0, -1, 0),
        xosc.LanePosition(130, 0, -1, 0),
        xosc.LanePosition(150, 0, -1, 0),
        xosc.LanePosition(170, 0, -1, 0),
    ]
    polyline = xosc.Polyline(times, positions)
    trajectory = xosc.Trajectory(CARRIER_ID + "_trajectory", closed=False)
    trajectory.add_shape(polyline)
    carrier_follow_trajectory = xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowMode.position,
        xosc.ReferenceContext.relative,
        1,
        0
    )

    carrier_move_trigger = xosc.ValueTrigger(
            "start_" + CARRIER_ID + "_" + "time_trigger",
            0,
            xosc.ConditionEdge.none,
            xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan)
        )

    carrier_follow_trajectory_event = xosc.Event(
            CARRIER_ID + "first_event",
            xosc.Priority.parallel
        )

    carrier_follow_trajectory_event.add_trigger(carrier_move_trigger)
    carrier_follow_trajectory_event.add_action("start", carrier_follow_trajectory)
    carrier_maneuver.add_event(carrier_follow_trajectory_event)

    carrier_first_trigger_position = xosc.LanePosition(130, 0, -1, 0)
    carrier_first_trigger_action = xosc.AbsoluteSpeedAction(
            0,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                -2
            ),
        )
    carrier_first_trigger = xosc.EntityTrigger(
            name="start_" + CARRIER_ID + "_" + "first_trigger",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(carrier_first_trigger_position, 1),
            triggerentity=str(CARRIER_ID)
        )

    carrier_first_event = xosc.Event(
            CARRIER_ID + "_first_event",
            xosc.Priority.parallel
        )

    carrier_first_event.add_trigger(carrier_first_trigger)
    carrier_first_event.add_action("start", carrier_first_trigger_action)
    carrier_maneuver.add_event(carrier_first_event)

    carrier_maneuver_group.add_actor(CARRIER_ID)
    carrier_maneuver_group.add_maneuver(carrier_maneuver)
    osc_act.add_maneuver_group(carrier_maneuver_group)

    # -------------- UFONANO ----------------

    osc_entities.add_scenario_object(UFO_ID,
                                     xosc.CatalogReference("VehicleCatalog", "car_blue"))

    ufo_start = xosc.TeleportAction(xosc.LanePosition(170, 0, 1, 0))

    init.add_init_action(UFO_ID, ufo_start)

    # Create Maneuver Group for the Carrier.
    ufo_maneuver_group = xosc.ManeuverGroup(UFO_ID + "_maneuver_group")
    # Create the Maneuver for the Carrier.
    ufo_maneuver = xosc.Maneuver(UFO_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(170, 0, 1, 0),
        xosc.LanePosition(160, 0, 1, 0),
        xosc.LanePosition(150, 0, 1, 0),
        xosc.LanePosition(140, 0, 1, 0),
        xosc.LanePosition(130, 0, 1, 0),
        xosc.LanePosition(120, 0, 1, 0),
        xosc.LanePosition(100, 0, 1, 0),
        xosc.LanePosition(90, 0, 1, 0),
        xosc.LanePosition(80, 0, 1, 0),
        xosc.LanePosition(70, 0, 1, 0),
        xosc.LanePosition(60, 0, 1, 0),
        xosc.LanePosition(50, 0, 1, 0),
        xosc.LanePosition(40, 0, 1, 0),
        xosc.LanePosition(30, 0, 1, 0),
    ]
    polyline = xosc.Polyline(times, positions)
    trajectory = xosc.Trajectory(UFO_ID + "_trajectory", closed=False)
    trajectory.add_shape(polyline)
    ufo_follow_trajectory = xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowMode.position,
        xosc.ReferenceContext.relative,
        1,
        0
    )

    ufo_speed = xosc.AbsoluteSpeedAction(
            30/3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                1.51
            ),
        )

    ufo_move_trigger = xosc.EntityTrigger(
            name="start_" + UFO_ID + "_" + "move_trigger",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.SpeedCondition(30/3.6, xosc.Rule.greaterThan),
            triggerentity=CARRIER_ID
        )

    ufo_follow_trajectory_event = xosc.Event(
        CARRIER_ID + "first_event",
        xosc.Priority.parallel
    )

    ufo_follow_trajectory_event.add_trigger(ufo_move_trigger)
    ufo_follow_trajectory_event.add_action("start", ufo_follow_trajectory)
    ufo_follow_trajectory_event.add_action("start", ufo_speed)
    ufo_maneuver.add_event(ufo_follow_trajectory_event)

    ufo_first_trigger_position = xosc.LanePosition(50, 0, 1, 0)
    ufo_first_trigger_action = xosc.AbsoluteSpeedAction(
        0,
        xosc.TransitionDynamics(
            xosc.DynamicsShapes.linear,
            xosc.DynamicsDimension.rate,
            -2
        ),
    )
    ufo_first_trigger = xosc.EntityTrigger(
        name="start_" + UFO_ID + "_" + "first_trigger",
        delay=0,
        conditionedge=xosc.ConditionEdge.none,
        entitycondition=xosc.ReachPositionCondition(ufo_first_trigger_position, 1),
        triggerentity=str(UFO_ID)
    )

    ufo_first_event = xosc.Event(
        UFO_ID + "_first_event",
        xosc.Priority.parallel
    )

    ufo_first_event.add_trigger(ufo_first_trigger)
    ufo_first_event.add_action("start", ufo_first_trigger_action)
    ufo_maneuver.add_event(ufo_first_event)

    ufo_maneuver_group.add_actor(UFO_ID)
    ufo_maneuver_group.add_maneuver(ufo_maneuver)
    osc_act.add_maneuver_group(ufo_maneuver_group)

    # -------------- DRONE ----------------
    osc_entities.add_scenario_object(DRONE_ID,
                                     xosc.CatalogReference("VehicleCatalog", "car_yellow"))

    drone_start = xosc.TeleportAction(xosc.LanePosition(170, 2.5, 1, 0))

    init.add_init_action(DRONE_ID, drone_start)

    # Create Maneuver Group for the Carrier.
    drone_maneuver_group = xosc.ManeuverGroup(DRONE_ID + "_maneuver_group")
    # Create the Maneuver for the Carrier.
    drone_maneuver = xosc.Maneuver(DRONE_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(170, 2.5, 1, 0),
        xosc.LanePosition(160, 2.5, 1, 0),
        xosc.LanePosition(150, 2.5, 1, 0),
        xosc.LanePosition(140, 2.5, 1, 0),
        xosc.LanePosition(130, 2.5, 1, 0),
        xosc.LanePosition(120, 2.5, 1, 0),
        xosc.LanePosition(100, 2.5, 1, 0),
        xosc.LanePosition(90, 2.5, 1, 0),
        xosc.LanePosition(80, 2.5, 1, 0),
        xosc.LanePosition(70, 2.5, 1, 0),
        xosc.LanePosition(60, 2.5, 1, 0),
        xosc.LanePosition(50, 2.5, 1, 0),
        xosc.LanePosition(40, 2.5, 1, 0),
        xosc.LanePosition(30, 2.5, 1, 0),
    ]
    polyline = xosc.Polyline(times, positions)
    trajectory = xosc.Trajectory(DRONE_ID + "_trajectory", closed=False)
    trajectory.add_shape(polyline)
    drone_follow_trajectory = xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowMode.position,
        xosc.ReferenceContext.relative,
        1,
        0
    )

    drone_speed = xosc.AbsoluteSpeedAction(
        30 / 3.6,
        xosc.TransitionDynamics(
            xosc.DynamicsShapes.linear,
            xosc.DynamicsDimension.rate,
            1.51
        ),
    )

    drone_move_trigger = xosc.EntityTrigger(
        name="start_" + DRONE_ID + "_" + "move_trigger",
        delay=0,
        conditionedge=xosc.ConditionEdge.none,
        entitycondition=xosc.SpeedCondition(30 / 3.6, xosc.Rule.greaterThan),
        triggerentity=CARRIER_ID
    )

    drone_follow_trajectory_event = xosc.Event(
        DRONE_ID + "first_event",
        xosc.Priority.parallel
    )

    drone_follow_trajectory_event.add_trigger(drone_move_trigger)
    drone_follow_trajectory_event.add_action("start", drone_follow_trajectory)
    drone_follow_trajectory_event.add_action("start", drone_speed)
    drone_maneuver.add_event(drone_follow_trajectory_event)

    drone_first_trigger_position = xosc.LanePosition(50, 2.5, 1, 0)
    drone_first_trigger_action = xosc.AbsoluteSpeedAction(
        0,
        xosc.TransitionDynamics(
            xosc.DynamicsShapes.linear,
            xosc.DynamicsDimension.rate,
            -2
        ),
    )
    drone_first_trigger = xosc.EntityTrigger(
        name="start_" + DRONE_ID + "_" + "first_trigger",
        delay=0,
        conditionedge=xosc.ConditionEdge.none,
        entitycondition=xosc.ReachPositionCondition(drone_first_trigger_position, 1),
        triggerentity=DRONE_ID
    )

    drone_first_event = xosc.Event(
        DRONE_ID + "_first_event",
        xosc.Priority.parallel
    )

    drone_first_event.add_trigger(drone_first_trigger)
    drone_first_event.add_action("start", drone_first_trigger_action)
    drone_maneuver.add_event(drone_first_event)

    drone_maneuver_group.add_actor(DRONE_ID)
    drone_maneuver_group.add_maneuver(drone_maneuver)
    osc_act.add_maneuver_group(drone_maneuver_group)

    # -------------- CREATE SCENARIO ----------------
    osc_storyboard.add_act(osc_act)
    osc_file = xosc.Scenario(
        name="Multilane",
        author="AstaZero 1.0",
        parameters=osc_params,
        entities=osc_entities,
        storyboard=osc_storyboard,
        roadnetwork=osc_road_network,
        catalog=osc_catalogs
    )

    osc_file.write_xml("multilane.xosc")


def main():
    opendrive_path = "/home/seifbourogaa/Documents/AstaZero/FFI-Demo/OpenDrive/Multilane.xodr"
    create_scenario(opendrive_path)

if __name__ == '__main__':
    main()