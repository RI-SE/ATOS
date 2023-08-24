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
vut_speed = 70 / 3.6  # m/s
vut_acceleration = 1.5  # m/s^2
vut_retardation = 1.75  # m/s^2
vut_starting_point = xosc.LanePosition(320, 0, 1, 1055000)
vut_brake_position = xosc.LanePosition(14, 0, -1, 1057000)
vut_limit_position = xosc.LanePosition(14, 0, -1, 1057000)

# GVT
gvt_speed = 90 / 3.6  # m/s
gvt_acceleration = 2.5  # m/s^2
gvt_retardation = 1.75  # m/s^2
# GVT starting point and trajectory, assuming no overtaking is triggered.
gvt_starting_point = xosc.LanePosition(270, 0, 1, 1055000)
gvt_second_traj_point = xosc.LanePosition(30, 0, -1, 1057000)
# GVT limit point, assuming no overtaking is triggered.
gvt_first_limit_point = xosc.LanePosition(720, 0, 1, 1055000)
# GVT limit point, assuming overtaking is initialized.
gvt_second_limit_point = xosc.LanePosition(14, 0, 1, 1057000)
# GVT overtaking limit point, assuming overtaking is initialized and completed.
gvt_third_limit_point = xosc.LanePosition(0, 0, -1, 1057000)
# Speed the VUT need to reach to trigger the GVT to start moving.
gvt_speed_threshold = 15 / 3.6  # m/s
# Distance between the VUT and GVT to trigger the GVT to start overtaking.
gvt_vut_start_overtake_threshold = 20.0  # m
# Time it takes for the GVT to reach the new lane after the overtaking is triggered.
gvt_vut_start_overtake_change_time = 5.0  # s
# Distance between the VUT and GVT to trigger the GVT to complete the overtaking.
gvt_vut_end_overtake_threshold = 1  # m
# Time it takes for the GVT to perform the lane change after the complete overtaking is triggered.
gvt_vut_end_overtake_change_time = 2.0  # s

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

# VUT initial trajectory and speed.
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
    xosc.Priority.overwrite
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

# GVT initial trajectory and speed.
times = []
positions = [
    gvt_starting_point,
    gvt_second_traj_point,
]
polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(GVT_ID + ",start_follow_trajectory", closed=False)
trajectory.add_shape(polyline)
gvt_follow_trajectory = xosc.FollowTrajectoryAction(
    trajectory,
    xosc.FollowingMode.position,
    xosc.ReferenceContext.absolute,
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

gvt_start_condition = xosc.SpeedCondition(
    gvt_speed_threshold,
    xosc.Rule.greaterThan,
)

gvt_start_trigger = xosc.EntityTrigger(
    GVT_ID + ",start_trigger",
    0,
    xosc.ConditionEdge.rising,
    gvt_start_condition,
    VUT_ID
)

gvt_move_event = xosc.Event(
    GVT_ID + ",start_move_event",
    xosc.Priority.skip
)
gvt_move_event.add_action(GVT_ID + ",start_follow_trajectory", gvt_follow_trajectory)
gvt_move_event.add_action(GVT_ID + ",set_speed", gvt_set_speed)
gvt_move_event.add_trigger(gvt_start_trigger)
gvt_maneuver.add_event(gvt_move_event)

# GVT start overtake maneuver.

# Condition
gvt_overtake_condition = xosc.RelativeDistanceCondition(
    gvt_vut_start_overtake_threshold, # distance
    xosc.Rule.lessThan, # rule
    xosc.RelativeDistanceType.longitudinal, # distance_type
    VUT_ID, # reference_entity
    freespace=False # freespace
)

# Trigger
gvt_start_overtake_trigger = xosc.EntityTrigger(
    GVT_ID + ",start_overtake_trigger",
    0,
    xosc.ConditionEdge.rising,
    gvt_overtake_condition,
    GVT_ID
)

# Overtake action
gvt_start_overtake_action = xosc.RelativeLaneChangeAction(
    1, # Lane ID
    VUT_ID, # Reference entity
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.sinusoidal,
        xosc.DynamicsDimension.time,
        gvt_vut_start_overtake_change_time
    ),
)

# GVT acquire new position.
gvt_start_overtake_acquire_position_action = xosc.AcquirePositionAction(gvt_second_limit_point)

# Event
gvt_start_overtake_event = xosc.Event(
    GVT_ID + ",start_overtake_event",
    xosc.Priority.overwrite
)

gvt_start_overtake_event.add_trigger(gvt_start_overtake_trigger)
gvt_start_overtake_event.add_action(GVT_ID + ",gvt_start_overtake_action", gvt_start_overtake_action)
gvt_start_overtake_event.add_action(GVT_ID + ",acquire_position_action", gvt_start_overtake_acquire_position_action)
gvt_maneuver.add_event(gvt_start_overtake_event)

# GVT end overtake maneuver.

# Condition
gvt_overtake_condition = xosc.RelativeDistanceCondition(
    gvt_vut_end_overtake_threshold, # distance
    xosc.Rule.greaterThan, # rule
    xosc.RelativeDistanceType.longitudinal, # distance_type
    VUT_ID, # reference_entity
    freespace=False # freespace
)

# Trigger
gvt_start_overtake_trigger = xosc.EntityTrigger(
    GVT_ID + ",start_overtake_trigger",
    0,
    xosc.ConditionEdge.rising,
    gvt_overtake_condition,
    GVT_ID
)

# Overtake action
gvt_start_overtake_action = xosc.RelativeLaneChangeAction(
    0, # Lane ID
    VUT_ID, # Reference entity
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.sinusoidal,
        xosc.DynamicsDimension.time,
        gvt_vut_end_overtake_change_time
    ),
)

# GVT acquire new position.
gvt_start_overtake_acquire_position_action = xosc.AcquirePositionAction(gvt_third_limit_point)

# Event
gvt_start_overtake_event = xosc.Event(
    GVT_ID + ",end_overtake_event",
    xosc.Priority.overwrite
)

gvt_start_overtake_event.add_trigger(gvt_start_overtake_trigger)
gvt_start_overtake_event.add_action(GVT_ID + ",gvt_end_overtake_action", gvt_start_overtake_action)
gvt_start_overtake_event.add_action(GVT_ID + ",acquire_position_action", gvt_start_overtake_acquire_position_action)
gvt_maneuver.add_event(gvt_start_overtake_event)

# GVT braking events
gvt_brake = xosc.AbsoluteSpeedAction(
    0,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        -gvt_retardation
    ),
)

gvt_reach_stop_position_1 = xosc.EntityTrigger(
    name=GVT_ID + ",reach_stop_position_1",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(gvt_first_limit_point, 1),
    triggerentity=GVT_ID
)

gvt_brake_event_1 = xosc.Event(
    GVT_ID + ",brake_event_1",
    xosc.Priority.overwrite
)
gvt_brake_event_1.add_trigger(gvt_reach_stop_position_1)
gvt_brake_event_1.add_action(GVT_ID + ",brake_to_stop", gvt_brake)
gvt_maneuver.add_event(gvt_brake_event_1)


# Stop conditions
gvt_reach_stop_position_2 = xosc.EntityTrigger(
    name=GVT_ID + ",reach_stop_position_2",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(gvt_second_limit_point, 1),
    triggerentity=GVT_ID
)

gvt_brake_event_2 = xosc.Event(
    GVT_ID + ",brake_event_2",
    xosc.Priority.overwrite
)
gvt_brake_event_2.add_trigger(gvt_reach_stop_position_2)
gvt_brake_event_2.add_action(GVT_ID + ",brake_to_stop", gvt_brake)
gvt_maneuver.add_event(gvt_brake_event_2)

gvt_reach_stop_position_3 = xosc.EntityTrigger(
    name=GVT_ID + ",reach_stop_position_3",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(gvt_third_limit_point, 1),
    triggerentity=GVT_ID
)

gvt_brake_event_3 = xosc.Event(
    GVT_ID + ",brake_event_3",
    xosc.Priority.overwrite
)
gvt_brake_event_3.add_trigger(gvt_reach_stop_position_3)
gvt_brake_event_3.add_action(GVT_ID + ",brake_to_stop", gvt_brake)
gvt_maneuver.add_event(gvt_brake_event_3)

# ----------------------GVT end------------------------------

# Collect into a scenario and write to file
vut_maneuver_group.add_maneuver(vut_maneuver)
gvt_maneuver_group.add_maneuver(gvt_maneuver)

osc_act.add_maneuver_group(vut_maneuver_group)
osc_act.add_maneuver_group(gvt_maneuver_group)

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