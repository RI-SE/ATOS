from scenariogeneration import xosc

def create_scenario(opendrive_path):
    MONDEO_ID = "O1"
    UFO_ID = "O2"
    CARRIER_ID = "O3"
    VIRTUAL_ID = "O4"
    DRONE_ID = "O5"

    init = xosc.Init()
    osc_params = xosc.ParameterDeclarations()
    osc_catalogs = xosc.Catalog()
    osc_catalogs.add_catalog('VehicleCatalog', '../Catalogs/Vehicles')
    osc_road_network = xosc.RoadNetwork(roadfile=opendrive_path)
    osc_entities = xosc.Entities()
    osc_storyboard = xosc.StoryBoard(init)
    osc_act = xosc.Act("start")

    # -------------- MONDEO ----------------
    osc_entities.add_scenario_object(MONDEO_ID, xosc.CatalogReference("VehicleCatalog", "car_white"))
    mondeo_start = xosc.TeleportAction(xosc.LanePosition(83.5103988647461, 0, -1, 3))
    mondeo_speed = xosc.AbsoluteSpeedAction(
            15 / 3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                0.51
            ),
        )
    init.add_init_action(MONDEO_ID, mondeo_start)
    init.add_init_action(MONDEO_ID, mondeo_speed)

    mondeo_maneuver_group = xosc.ManeuverGroup(MONDEO_ID + "_maneuver_group")
    mondeo_maneuver = xosc.Maneuver(MONDEO_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(83.5103988647461, 0, -1, 3),
        xosc.LanePosition(100.95606231689453, 0, -1, 3),
        xosc.LanePosition(1.0940648317337036, 0, -1, 7),
        xosc.LanePosition(2.0940648317337036, 0, -1, 7),
        xosc.LanePosition(3.0940648317337036, 0, -1, 7),
        xosc.LanePosition(4.0940648317337036, 0, -1, 7),
        xosc.LanePosition(5.0940648317337036, 0, -1, 7),
        xosc.LanePosition(6.0940648317337036, 0, -1, 7),
        xosc.LanePosition(7.0940648317337036, 0, -1, 7),
        xosc.LanePosition(8.0940648317337036, 0, -1, 7),
        xosc.LanePosition(82, 0, 1, 0),
        xosc.LanePosition(80.36375427246094, 0, 1, 0),
        xosc.LanePosition(68.76012420654297, 0, 1, 0),
        xosc.LanePosition(64.20175170898438, 0, 1, 0),
    ]
    polyline = xosc.Polyline(times, positions)
    trajectory = xosc.Trajectory(MONDEO_ID + "_trajectory", closed=False)
    trajectory.add_shape(polyline)
    mondeo_follow_trajectory = xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowMode.position,
        xosc.ReferenceContext.relative,
        1,
        0
    )
    mondeo_move_trigger =  xosc.ValueTrigger(
            MONDEO_ID + "_move_trigger",
            0,
            xosc.ConditionEdge.none,
            xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan)
        )
    follow_trajectory_event = xosc.Event(
            MONDEO_ID + "mondeo_follow_trajectory_event",
            xosc.Priority.parallel
        )
    follow_trajectory_event.add_trigger(mondeo_move_trigger)
    follow_trajectory_event.add_action("mondeo_follow_trajectory", mondeo_follow_trajectory)
    mondeo_maneuver.add_event(follow_trajectory_event)

    # Create event for the first trigger.
    first_trigger_position = xosc.LanePosition(1.0940648317337036, 0, -1, 7)
    first_trigger_action = xosc.AbsoluteSpeedAction(
            10 / 3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                -2.17
            ),
        )
    first_trigger = xosc.EntityTrigger(
            name=MONDEO_ID + "first_trigger",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(first_trigger_position, 1),
            triggerentity=MONDEO_ID
        )
    first_event = xosc.Event(
            MONDEO_ID + "first_event",
            xosc.Priority.parallel
        )
    first_event.add_trigger(first_trigger)
    first_event.add_action("first_action", first_trigger_action)
    mondeo_maneuver.add_event(first_event)

    # Create event for the second trigger.
    second_trigger_position = xosc.LanePosition(82, 0, 1, 0)
    second_trigger_action = xosc.AbsoluteSpeedAction(
            15 / 3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                0.66
            ),
        )
    second_trigger = xosc.EntityTrigger(
            name=MONDEO_ID + "second",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(second_trigger_position, 1),
            triggerentity=MONDEO_ID
        )
    second_event = xosc.Event(
            MONDEO_ID + "second_event",
            xosc.Priority.parallel
        )
    second_event.add_trigger(second_trigger)
    second_event.add_action("second_action", second_trigger_action)
    mondeo_maneuver.add_event(second_event)

    # Create event for the third trigger.
    third_trigger_position = xosc.LanePosition(68.76012420654297, 0, 1, 0)
    third_trigger_action = xosc.AbsoluteSpeedAction(
            0,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                -1.93
            ),
        )
    third_trigger = xosc.EntityTrigger(
            name=MONDEO_ID + "third",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(third_trigger_position, 1),
            triggerentity=MONDEO_ID
        )
    third_event = xosc.Event(
            MONDEO_ID + "third_event",
            xosc.Priority.parallel
        )
    third_event.add_trigger(third_trigger)
    third_event.add_action("third_action", third_trigger_action)
    mondeo_maneuver.add_event(third_event)

    mondeo_maneuver_group.add_actor(MONDEO_ID)
    mondeo_maneuver_group.add_maneuver(mondeo_maneuver)

    osc_act.add_maneuver_group(mondeo_maneuver_group)

    # -------------- UFONANO ----------------
    osc_entities.add_scenario_object(UFO_ID, xosc.CatalogReference("VehicleCatalog", "car_blue"))

    ufonano_start = xosc.TeleportAction(xosc.LanePosition(19.521028518676758, 0, 1, 1))

    init.add_init_action(UFO_ID, ufonano_start)

    ufonano_maneuver_group = xosc.ManeuverGroup(UFO_ID + "_maneuver_group")
    ufonano_maneuver = xosc.Maneuver(UFO_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(19.521028518676758, 0, 1, 1),
        xosc.LanePosition(13.252511024475098, 0, 1, 1),
        xosc.LanePosition(72.19840240478516, 0, 1, 0),
        xosc.LanePosition(67.49223327636719, 0, 1, 0),
    ]
    polyline = xosc.Polyline(times, positions)
    trajectory = xosc.Trajectory(UFO_ID + "_trajectory", closed=False)
    trajectory.add_shape(polyline)
    ufonano_follow_trajectory = xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowMode.position,
        xosc.ReferenceContext.relative,
        1,
        0
    )

    ufonano_speed = xosc.AbsoluteSpeedAction(
            10 / 3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                1.38
            ),
        )

    ufonano_trigger_position = xosc.LanePosition(1.0940648317337036, 0, -1, 7)
    ufonano_move_trigger = xosc.EntityTrigger(
            name=UFO_ID + "move_trigger",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(ufonano_trigger_position, 1),
            triggerentity=MONDEO_ID
        )
    ufonano_move_event = xosc.Event(
            UFO_ID + "move_event",
            xosc.Priority.parallel
        )
    ufonano_move_event.add_trigger(ufonano_move_trigger)
    ufonano_move_event.add_action("move_action", ufonano_follow_trajectory)
    ufonano_move_event.add_action("speed_action", ufonano_speed)
    ufonano_maneuver.add_event(ufonano_move_event)

    # Create event for the second trigger.
    ufonano_trigger_position = xosc.LanePosition(78.19840240478516, 0, 1, 0)
    ufonano_speed = xosc.AbsoluteSpeedAction(
            0,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                -1.83
            ),
        )
    ufonano_trigger = xosc.EntityTrigger(
            name=UFO_ID + "second",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(ufonano_trigger_position, 1),
            triggerentity=UFO_ID
        )
    ufonano_event = xosc.Event(
            UFO_ID + "second_event",
            xosc.Priority.parallel
        )
    ufonano_event.add_trigger(ufonano_trigger)
    ufonano_event.add_action("second_action", ufonano_speed)
    ufonano_maneuver.add_event(ufonano_event)

    ufonano_maneuver_group.add_actor(UFO_ID)
    ufonano_maneuver_group.add_maneuver(ufonano_maneuver)

    osc_act.add_maneuver_group(ufonano_maneuver_group)

    # -------------- TARGET CARRIER ----------------
    osc_entities.add_scenario_object(CARRIER_ID, xosc.CatalogReference("VehicleCatalog", "car_yellow"))

    carrier_start = xosc.TeleportAction(xosc.LanePosition(44.70673370361328, 0, -1, 0))
    init.add_init_action(CARRIER_ID, carrier_start)

    carrier_maneuver_group = xosc.ManeuverGroup(CARRIER_ID + "_maneuver_group")
    carrier_maneuver = xosc.Maneuver(CARRIER_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(44.70673370361328, 0, -1, 0),
        xosc.LanePosition(52.409610748291016, 0, -1, 0),
        xosc.LanePosition(6.957085132598877, 0, -1, 1),
        xosc.LanePosition(15.372483253479004, 0, -1, 1),
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

    carrier_speed = xosc.AbsoluteSpeedAction(
            15 / 3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                1.12
            ),
        )
    carrier_trigger_position = xosc.LanePosition(1.0940648317337036, 0, -1, 7)
    carrier_move_trigger = xosc.EntityTrigger(
            name=CARRIER_ID + "move_trigger",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(carrier_trigger_position, 1),
            triggerentity=MONDEO_ID
        )
    carrier_move_event = xosc.Event(
            CARRIER_ID + "move_event",
            xosc.Priority.parallel
        )
    carrier_move_event.add_trigger(carrier_move_trigger)
    carrier_move_event.add_action("move_action", carrier_follow_trajectory)
    carrier_move_event.add_action("speed_action", carrier_speed)
    carrier_maneuver.add_event(carrier_move_event)

    carrier_trigger_position = xosc.LanePosition(6.957085132598877, 0, -1, 1)
    carrier_speed = xosc.AbsoluteSpeedAction(
            0,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                -1.03
            ),
        )
    carrier_trigger = xosc.EntityTrigger(
            name=CARRIER_ID + "second",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(carrier_trigger_position, 1),
            triggerentity=CARRIER_ID
        )
    carrier_event = xosc.Event(
            CARRIER_ID + "second_event",
            xosc.Priority.parallel
        )
    carrier_event.add_trigger(carrier_trigger)
    carrier_event.add_action("second_action", carrier_speed)
    carrier_maneuver.add_event(carrier_event)

    carrier_maneuver_group.add_actor(CARRIER_ID)
    carrier_maneuver_group.add_maneuver(carrier_maneuver)
    osc_act.add_maneuver_group(carrier_maneuver_group)

    # -------------- VIRTUAL OBJECT ---------------
    osc_entities.add_scenario_object(VIRTUAL_ID, xosc.CatalogReference("VehicleCatalog", "car_red"))

    virtual_start = xosc.TeleportAction(xosc.LanePosition(18.582534790039062, 0, 1, 0))
    init.add_init_action(VIRTUAL_ID, virtual_start)

    virtual_maneuver_group = xosc.ManeuverGroup(VIRTUAL_ID + "_maneuver_group")
    virtual_maneuver = xosc.Maneuver(VIRTUAL_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(18.582534790039062, 0, 1, 0),
        xosc.LanePosition(31.879226684570312, 0, 1, 0),
        xosc.LanePosition(57.04560852050781, 0, 1, 0),
        xosc.LanePosition(61.62282180786133, 0, 1, 0),
    ]
    polyline = xosc.Polyline(times, positions)
    trajectory = xosc.Trajectory(VIRTUAL_ID + "_trajectory", closed=False)
    trajectory.add_shape(polyline)
    virtual_follow_trajectory = xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowMode.position,
        xosc.ReferenceContext.relative,
        1,
        0
    )
    virtual_speed = xosc.AbsoluteSpeedAction(
            30 / 3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                3.53
            ),
        )
    virtual_trigger_position = xosc.LanePosition(1.0940648317337036, 0, -1, 7)
    virtual_move_trigger = xosc.EntityTrigger(
            name=VIRTUAL_ID + "move_trigger",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(virtual_trigger_position, 1),
            triggerentity=MONDEO_ID
        )
    virtual_move_event = xosc.Event(
            VIRTUAL_ID + "move_event",
            xosc.Priority.parallel
        )
    virtual_move_event.add_trigger(virtual_move_trigger)
    virtual_move_event.add_action("move_action", virtual_follow_trajectory)
    virtual_move_event.add_action("speed_action", virtual_speed)
    virtual_maneuver.add_event(virtual_move_event)

    virtual_trigger_position = xosc.LanePosition(57.04560852050781, 0, 1, 0)
    virtual_speed = xosc.AbsoluteSpeedAction(
        0,
        xosc.TransitionDynamics(
            xosc.DynamicsShapes.linear,
            xosc.DynamicsDimension.rate,
            -10.32
        ),
    )
    virtual_trigger = xosc.EntityTrigger(
            name=VIRTUAL_ID + "second",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(virtual_trigger_position, 1),
            triggerentity=VIRTUAL_ID
        )
    virtual_event = xosc.Event(
            VIRTUAL_ID + "second_event",
            xosc.Priority.parallel
        )
    virtual_event.add_trigger(virtual_trigger)
    virtual_event.add_action("second_action", virtual_speed)
    virtual_maneuver.add_event(virtual_event)

    virtual_maneuver_group.add_actor(VIRTUAL_ID)
    virtual_maneuver_group.add_maneuver(virtual_maneuver)
    osc_act.add_maneuver_group(virtual_maneuver_group)

    # -------------- RECORDING DRONE ----------------
    osc_entities.add_scenario_object(DRONE_ID, xosc.CatalogReference("VehicleCatalog", "car_red"))

    drone_start = xosc.TeleportAction(xosc.LanePosition(13.252511024475098, 2.5, 1, 1))
    init.add_init_action(DRONE_ID, drone_start)

    drone_maneuver_group = xosc.ManeuverGroup(DRONE_ID + "_maneuver_group")
    drone_maneuver = xosc.Maneuver(DRONE_ID + "_maneuver")

    times = []
    positions = [
        xosc.LanePosition(13.252511024475098, 2.5, 1, 1),
        xosc.LanePosition(8.073124885559082, 2.5, 1, 1),
        xosc.LanePosition(66.29085540771484, 2.5, 1, 0),
        xosc.LanePosition(61.50538635253906, 2.5, 1, 0),
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
            15 / 3.6,
            xosc.TransitionDynamics(
                xosc.DynamicsShapes.linear,
                xosc.DynamicsDimension.rate,
                1.76
            ),
        )
    drone_trigger_position = xosc.LanePosition(1.0940648317337036, 0, -1, 7)
    drone_move_trigger = xosc.EntityTrigger(
            name=DRONE_ID + "move_trigger",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(drone_trigger_position, 1),
            triggerentity=MONDEO_ID
        )
    drone_move_event = xosc.Event(
            DRONE_ID + "move_event",
            xosc.Priority.parallel
        )
    drone_move_event.add_trigger(drone_move_trigger)
    drone_move_event.add_action("move_action", drone_follow_trajectory)
    drone_move_event.add_action("speed_action", drone_speed)
    drone_maneuver.add_event(drone_move_event)

    drone_trigger_position = xosc.LanePosition(66.29085540771484, 2.5, 1, 0)
    drone_speed = xosc.AbsoluteSpeedAction(
        0,
        xosc.TransitionDynamics(
            xosc.DynamicsShapes.linear,
            xosc.DynamicsDimension.rate,
            -1.96
        ),
    )
    drone_trigger = xosc.EntityTrigger(
            name=DRONE_ID + "second",
            delay=0,
            conditionedge=xosc.ConditionEdge.none,
            entitycondition=xosc.ReachPositionCondition(drone_trigger_position, 1),
            triggerentity=DRONE_ID
        )
    drone_event = xosc.Event(
            DRONE_ID + "second_event",
            xosc.Priority.parallel
        )
    drone_event.add_trigger(drone_trigger)
    drone_event.add_action("second_action", drone_speed)
    drone_maneuver.add_event(drone_event)
    drone_maneuver_group.add_actor(DRONE_ID)
    drone_maneuver_group.add_maneuver(drone_maneuver)
    osc_act.add_maneuver_group(drone_maneuver_group)

    # -------------- CREATE SCENARIO ----------------
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

    osc_file.write_xml("ffi_scenario.xosc")

def main():
    opendrive_path = "/home/seifbourogaa/Documents/AstaZero/FFI-Demo/OpenDrive/FlexZoneCrossing.xodr"
    create_scenario(opendrive_path)


if __name__ == "__main__":
    main()