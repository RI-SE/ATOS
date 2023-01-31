from scenariogeneration import esmini
from scenariogeneration import xosc

odr_path = "../conf/odr/GaragePlan.xodr"
catalog_path = "../conf/Catalogs/Vehicles"
output_filename = "DroneScenario.xosc"
DRONE_ID = 5

init = xosc.Init()
osc_params = xosc.ParameterDeclarations()
osc_catalogs = xosc.Catalog()
osc_catalogs.add_catalog('VehicleCatalog', catalog_path)
osc_road_network = xosc.RoadNetwork(roadfile=odr_path)
osc_entities = xosc.Entities()
osc_storyboard = xosc.StoryBoard(init)
osc_act = xosc.Act("start")

osc_entities = osc_entities.add_scenario_object(str(DRONE_ID), xosc.CatalogReference('VehicleCatalog', 'car_red'))
osc_teleport_action = xosc.TeleportAction(xosc.WorldPosition(10, 10, 5, 0, 0, 0)) # xyzhpr
osc_absolute_speed_action = xosc.AbsoluteSpeedAction(15/3.6, xosc.TransitionDynamics(xosc.DynamicsShapes.linear, xosc.DynamicsDimension.time, 0.8),)
init.add_init_action(str(DRONE_ID), osc_teleport_action)
init.add_init_action(str(DRONE_ID), osc_absolute_speed_action)

osc_maneuver_group = xosc.ManeuverGroup(str(DRONE_ID) + "_maneuver_group")

times = [0, 4.8, 4.8+8.5]

positions = [
        xosc.WorldPosition(5,50,5,0,0,0),
        xosc.WorldPosition(25,50,5,0,0,0),
        xosc.WorldPosition(30,15,5,0,0,0)
        ]


polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(str(DRONE_ID) + "_trajectory", closed=False)
trajectory.add_shape(polyline)

osc_maneuver = xosc.Maneuver(str(DRONE_ID) + "_maneuver")
#osc_stop_maneuver = xosc.Maneuver(str(DRONE_ID) + "_stop_maneuver")

# Start event
osc_follow_trajectory = xosc.FollowTrajectoryAction(
    trajectory,
    xosc.FollowMode.position,
    xosc.ReferenceContext.relative,
    1,
    0
)
osc_trigger = xosc.ValueTrigger(
    str(DRONE_ID) + "_start_trigger",
    0,
    xosc.ConditionEdge.none,
    xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan)
)
osc_event = xosc.Event(
    str(DRONE_ID) + "_start_event",
    xosc.Priority.overwrite,
)

osc_event.add_trigger(osc_trigger)
osc_event.add_action(str(DRONE_ID) + "_follow_trajectory", osc_follow_trajectory)
osc_maneuver.add_event(osc_event)

# Stop event
osc_absolute_speed_action = xosc.AbsoluteSpeedAction(
    0,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.time,
        -0.8
    ),
)
osc_trigger = xosc.EntityTrigger(
    name=str(DRONE_ID) + "_stop_trigger",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(positions[-1], 1.0), # tolerance
    triggerentity=str(DRONE_ID)
)
osc_event = xosc.Event(
    str(DRONE_ID) + "_stop_event",
    xosc.Priority.overwrite,
)

osc_event.add_trigger(osc_trigger)
osc_event.add_action(str(DRONE_ID) + "_stop", osc_absolute_speed_action)
osc_maneuver.add_event(osc_event)


# Collect everything
osc_maneuver_group.add_actor(str(DRONE_ID))
osc_maneuver_group.add_maneuver(osc_maneuver)
osc_act.add_maneuver_group(osc_maneuver_group)
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
