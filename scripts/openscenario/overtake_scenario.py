#!/usr/bin/env python
# coding: utf-8


from scenariogeneration import xosc

import copy

# Fixed parameters
VUT_ID = "1"
GVT_ID = "2"

CATALOG_PATH = "../Catalogs/Vehicles"
CATALOG_NAME = "VehicleCatalog"

OPENDRIVE_PATH = "../odr/AstaZero_PG_OpenDRIVE_MULTILANE_ROAD.xodr"
SCENARIO_NAME = "OvertakeScenario"
SCENARIO_FILE_NAME = SCENARIO_NAME + ".xosc"

# VUT
vut_speed = 15 / 3.6  # m/s
vut_acceleration = 1.12  # m/s^2
vut_retardation = 1.03  # m/s^2
vut_starting_point = xosc.LanePosition(675, 0, 1, 1055000)
vut_brake_position = xosc.LanePosition(14, 0, -1, 1057000)
vut_limit_position = xosc.LanePosition(14, 0, -1, 1057000)

# GVT

# Preamble
init = xosc.Init()
osc_params = xosc.ParameterDeclarations()
osc_catalogs = xosc.Catalog()
osc_catalogs.add_catalog(CATALOG_NAME, CATALOG_PATH)
osc_road_network = xosc.RoadNetwork(roadfile=OPENDRIVE_PATH)
osc_entities = xosc.Entities()
osc_storyboard = xosc.StoryBoard(init)
osc_act = xosc.Act("start")

# ------------------------ VUT ------------------------
vut_maneuver_group = xosc.ManeuverGroup(VUT_ID + ",maneuver_group")
vut_maneuver = xosc.Maneuver(VUT_ID + ",maneuver")
vut_maneuver_group.add_actor(VUT_ID)

osc_entities.add_scenario_object(VUT_ID, xosc.CatalogReference(CATALOG_NAME, "car_white"))

vut_start_teleport = xosc.TeleportAction(vut_starting_point)

init.add_init_action(VUT_ID, vut_start_teleport)

# Actions and Events

# VUT movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
times = []
positions = [
    vut_starting_point,
    vut_limit_position

]
polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(VUT_ID + ",start_follow_trajectory", closed=False)
trajectory.add_shape(polyline)
vut_follow_trajectory = xosc.FollowTrajectoryAction(
    trajectory,
    xosc.FollowingMode.position,
    xosc.ReferenceContext.relative,
    1,
    0
)
vut_set_speed = xosc.AbsoluteSpeedAction(
    vut_speed,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        vut_acceleration
    ),
)
vut_brake = xosc.AbsoluteSpeedAction(
    0,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        -vut_retardation
    ),
)
vut_reach_stop_position = xosc.EntityTrigger(
    name=VUT_ID + ",reach_stop_position",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(vut_brake_position, 1),
    triggerentity=VUT_ID
)
vut_brake_event = xosc.Event(
    VUT_ID + ",brake_event",
    xosc.Priority.parallel
)
vut_brake_event.add_trigger(vut_reach_stop_position)
vut_brake_event.add_action(VUT_ID + ",brake_to_stop", vut_brake)
vut_maneuver.add_event(vut_brake_event)

vut_move_event = xosc.Event(
    VUT_ID + ",start_move_event",
    xosc.Priority.parallel
)
vut_move_event.add_action(VUT_ID + ",start_follow_trajectory", vut_follow_trajectory)
vut_move_event.add_action(VUT_ID + ",set_speed", vut_set_speed)
vut_maneuver.add_event(vut_move_event)

# ------------------------ GVT ------------------------

# Add GVT start position, trajectory etc here. See above for VUT example

# Collect into a scenario and write to file
vut_maneuver_group.add_maneuver(vut_maneuver)

osc_act.add_maneuver_group(vut_maneuver_group)

osc_storyboard.add_act(osc_act)
osc_file = xosc.Scenario(
    name=SCENARIO_NAME,
    author="AstaZero 1.0",
    parameters=osc_params,
    entities=osc_entities,
    storyboard=osc_storyboard,
    roadnetwork=osc_road_network,
    catalog=osc_catalogs
)

osc_file.write_xml(SCENARIO_FILE_NAME)