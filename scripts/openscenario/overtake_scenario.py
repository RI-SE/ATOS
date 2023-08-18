#!/usr/bin/env python
# coding: utf-8


from scenariogeneration import xosc

import copy

# Fixed parameters
VUT_ID = "1"
GVT_ID = "2"
DRONE_ID = "3"

CATALOG_PATH = "../Catalogs/Vehicles"
CATALOG_NAME = "VehicleCatalog"

OPENDRIVE_PATH = "/home/sebastian/.astazero/ATOS/odr/AstaZero_PG_OpenDRIVE_MULTILANE_ROAD.xodr"
#OPENDRIVE_PATH = "/home/safe/..."
SCENARIO_NAME = "OvertakeScenario"
SCENARIO_FILE_NAME = SCENARIO_NAME + ".xosc"

# VUT
vut_speed = 22 / 3.6  # m/s
vut_acceleration = 1.5  # m/s^2
vut_retardation = 1.75  # m/s^2
vut_starting_point = xosc.LanePosition(675, 0, 1, 1055000)
vut_brake_position = xosc.LanePosition(14, 0, -1, 1057000)
vut_limit_position = xosc.LanePosition(14, 0, -1, 1057000)

# GVT
gvt_speed = 30 / 3.6  # m/s
gvt_acceleration = 1.6  # m/s^2
gvt_retardation = 1.75  # m/s^2
gvt_starting_point = xosc.LanePosition(675, 0, 2, 1055000)
gvt_turn_position_1 = xosc.LanePosition(706, -1.2, 2, 1055000)
gvt_turn_position_2 = xosc.LanePosition(707, -1.4, 2, 1055000)
gvt_turn_position_3 = xosc.LanePosition(708, -1.6, 2, 1055000)
gvt_turn_position_4 = xosc.LanePosition(709, -1.8, 2, 1055000)
gvt_turn_position_5 = xosc.LanePosition(710, -2, 2, 1055000)
gvt_turn_position_6 = xosc.LanePosition(711, -2.2, 2, 1055000)
gvt_turn_position_7 = xosc.LanePosition(712, -2.4, 2, 1055000)
gvt_turn_position_8 = xosc.LanePosition(713, -2.6, 2, 1055000)
gvt_turn_position_9 = xosc.LanePosition(714, -2.8, 2, 1055000)
gvt_turn_position_10 = xosc.LanePosition(715, -3, 2, 1055000)
gvt_turn_position_11 = xosc.LanePosition(716, -3.2, 2, 1055000)
gvt_turn_position_12 = xosc.LanePosition(717, -3.4, 2, 1055000)
gvt_turn_position_13 = xosc.LanePosition(718, -3.7, 2, 1055000)
gvt_turn_position_14 = xosc.LanePosition(719, -3.8, 2, 1055000)
gvt_turn_position_15 = xosc.LanePosition(720, -4, 2, 1055000)

gvt_brake_position = xosc.LanePosition(17, 0, -1, 1057000)
gvt_limit_position = xosc.LanePosition(17, 0, -1, 1057000)

# Drone
drone_speed = 25 / 3.6  # m/s
drone_acceleration = 1.5  # m/s^2
drone_retardation = 1.75  # m/s^2
drone_starting_point = xosc.LanePosition(675, 0, -1, 1055000)
drone_brake_position = xosc.LanePosition(14, 0, -2, 1057000)
drone_limit_position = xosc.LanePosition(14, 0, -2, 1057000)


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

# ----------------------VUT end------------------------------

# ------------------------ GVT ------------------------
# Add GVT start position, trajectory etc here. See above for VUT example

gvt_maneuver_group = xosc.ManeuverGroup(GVT_ID + ",maneuver_group")
gvt_maneuver = xosc.Maneuver(GVT_ID + ",maneuver")
gvt_maneuver_group.add_actor(GVT_ID)

osc_entities.add_scenario_object(GVT_ID, xosc.CatalogReference(CATALOG_NAME, "car_red"))

gvt_start_teleport = xosc.TeleportAction(gvt_starting_point)

init.add_init_action(GVT_ID, gvt_start_teleport)

# Actions and Events

# GVT movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
times = []
positions = [
    gvt_starting_point,
    gvt_turn_position_1,
    gvt_turn_position_2,
    gvt_turn_position_3,
    gvt_turn_position_4,
    gvt_turn_position_5,
    gvt_turn_position_6,
    gvt_turn_position_7,
    gvt_turn_position_8,
    gvt_turn_position_9,
    gvt_turn_position_10,
    gvt_turn_position_11,
    gvt_turn_position_12,
    #gvt_turn_position_13,
    #gvt_turn_position_14,
    #gvt_turn_position_15,
    gvt_limit_position

]
polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(GVT_ID + ",start_follow_trajectory", closed=False)
trajectory.add_shape(polyline)
gvt_follow_trajectory = xosc.FollowTrajectoryAction(
    trajectory,
    xosc.FollowingMode.position,
    xosc.ReferenceContext.relative,
    1,
    0
)
gvt_set_speed = xosc.AbsoluteSpeedAction(
    gvt_speed,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        gvt_acceleration
    ),
)
gvt_brake = xosc.AbsoluteSpeedAction(
    0,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        -gvt_retardation
    ),
)
gvt_reach_stop_position = xosc.EntityTrigger(
    name=GVT_ID + ",reach_stop_position",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(gvt_brake_position, 1),
    triggerentity=GVT_ID
)
gvt_brake_event = xosc.Event(
    GVT_ID + ",brake_event",
    xosc.Priority.parallel
)
gvt_brake_event.add_trigger(gvt_reach_stop_position)
gvt_brake_event.add_action(GVT_ID + ",brake_to_stop", gvt_brake)
gvt_maneuver.add_event(gvt_brake_event)

gvt_move_event = xosc.Event(
    GVT_ID + ",start_move_event",
    xosc.Priority.parallel
)
gvt_move_event.add_action(GVT_ID + ",start_follow_trajectory", gvt_follow_trajectory)
gvt_move_event.add_action(GVT_ID + ",set_speed", gvt_set_speed)
gvt_maneuver.add_event(gvt_move_event)

# ----------------------GVT end------------------------------

# ------------------------ DRONE ------------------------
drone_maneuver_group = xosc.ManeuverGroup(DRONE_ID + ",maneuver_group")
drone_maneuver = xosc.Maneuver(DRONE_ID + ",maneuver")
drone_maneuver_group.add_actor(DRONE_ID)

osc_entities.add_scenario_object(DRONE_ID, xosc.CatalogReference(CATALOG_NAME, "bicycle"))

drone_start_teleport = xosc.TeleportAction(drone_starting_point)

init.add_init_action(DRONE_ID, drone_start_teleport)

# Actions and Events

# DRONE movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
times = []
positions = [
    drone_starting_point,
    drone_limit_position

]
polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(DRONE_ID + ",start_follow_trajectory", closed=False)
trajectory.add_shape(polyline)
drone_follow_trajectory = xosc.FollowTrajectoryAction(
    trajectory,
    xosc.FollowingMode.position,
    xosc.ReferenceContext.relative,
    1,
    0
)
drone_set_speed = xosc.AbsoluteSpeedAction(
    drone_speed,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        vut_acceleration
    ),
)
drone_brake = xosc.AbsoluteSpeedAction(
    0,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        -drone_retardation
    ),
)
drone_reach_stop_position = xosc.EntityTrigger(
    name=DRONE_ID + ",reach_stop_position",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(drone_brake_position, 1),
    triggerentity=DRONE_ID
)
drone_brake_event = xosc.Event(
    DRONE_ID + ",brake_event",
    xosc.Priority.parallel
)
drone_brake_event.add_trigger(drone_reach_stop_position)
drone_brake_event.add_action(DRONE_ID + ",brake_to_stop", drone_brake)
drone_maneuver.add_event(drone_brake_event)

drone_move_event = xosc.Event(
    DRONE_ID + ",start_move_event",
    xosc.Priority.parallel
)
drone_move_event.add_action(DRONE_ID + ",start_follow_trajectory", drone_follow_trajectory)
drone_move_event.add_action(DRONE_ID + ",set_speed", drone_set_speed)
drone_maneuver.add_event(drone_move_event)

# ----------------------DRONE end------------------------------



# Collect into a scenario and write to file
vut_maneuver_group.add_maneuver(vut_maneuver)
gvt_maneuver_group.add_maneuver(gvt_maneuver)
drone_maneuver_group.add_maneuver(drone_maneuver)

osc_act.add_maneuver_group(vut_maneuver_group)
osc_act.add_maneuver_group(gvt_maneuver_group)
osc_act.add_maneuver_group(drone_maneuver_group)

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