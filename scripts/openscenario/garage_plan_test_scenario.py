#!/usr/bin/env python
# coding: utf-8

# In[337]:


from scenariogeneration import xosc


# In[338]:


# Fixed parameters
MONDEO_ID = "1"
UFO_ID = "2"
CARRIER_ID = "3"
VIRTUAL_ID = "4"
CAMERA_DRONE_ID = "5"
MARKER_DRONE_ID = "6"

MONDEO_CATALOG_NAME = "mondeo"
UFO_CATALOG_NAME = "humanetics"
CARRIER_CATALOG_NAME = "flexible_target_carrier"
VIRTUAL_CATALOG_NAME = "virtual_object"
DRONE_CATALOG_NAME = "camera_drone"
MARKER_DRONE_CATALOG_NAME = "marker_drone"

CATALOG_PATH = "../Catalogs/Vehicles"
CATALOG_NAME = "VehicleCatalog"
CONTROLLER_CATALOG_PATH = "../Catalogs/Controllers"
CONTROLLER_CATALOG_NAME = "ControllerCatalog"

OPENDRIVE_PATH = "../odr/GaragePlan.xodr"
SCENARIO_NAME = "GaragePlanScenario"
SCENARIO_FILE_NAME = SCENARIO_NAME + ".xosc"


# In[339]:


# Dynamic parameters
camera_drone_speed = 30 / 3.6 # m/s
camera_drone_acceleration = 1.76 # m/s^2
camera_drone_retardation = 1.96 # m/s^2
camera_drone_O5_point = xosc.LanePosition(15, -3.5, -1, 1)
camera_drone_brake_position = xosc.LanePosition(10, -3.5, -1, 0)

# Dynamic parameters
# mondeo
mondeo_speed = 15 / 3.6 # m/s
mondeo_slow_speed = 10 / 3.6 # m/s
mondeo_acceleration = 0.51 # m/s^2
mondeo_retardation = 2.17 # m/s^2
mondeo_O1_point = xosc.LanePosition(8, 0, 1, 3)
mondeo_trigger_positions = {
	CAMERA_DRONE_ID: xosc.LanePosition(0, 0, 1, 3),
	UFO_ID: xosc.LanePosition(0, 0, 1, 3),
	VIRTUAL_ID: xosc.LanePosition(0, 0, 1, 3),
    CARRIER_ID: xosc.LanePosition(0, 0, 1, 3),
    MARKER_DRONE_ID: xosc.LanePosition(0, 0, 1, 3)
}
mondeo_slow_down_position = xosc.LanePosition(2, 0, 1, 3)
mondeo_accelerate_position = xosc.LanePosition(1.0940648317337036, 0, 1, 12)
mondeo_brake_position = xosc.LanePosition(20, 0, 1, 0)

# ufo
ufo_speed = 10 / 3.6 # m/s
ufo_acceleration = 1.38 # m/s^2
ufo_retardation = 1.83 # m/s^2
ufo_O2_point = xosc.LanePosition(7.5, 0, 1, 1)
ufo_brake_position = xosc.LanePosition(25, 0, 1, 0)

# virtual object
virtual_speed = 30 / 3.6 # m/s
virtual_acceleration = 3.53 # m/s^2
virtual_retardation = 10.32 # m/s^2
virtual_O4_point = xosc.LanePosition(5, 0, 1, 0)
virtual_brake_position = xosc.LanePosition(8, 0, 1, 0)
virtual_trigger_delay = 3

# Flexible target carrier
carrier_speed = 15 / 3.6 # m/s
carrier_acceleration = 1.12 # m/s^2
carrier_retardation = 1.03 # m/s^2
carrier_O3_point = xosc.LanePosition(20, 0, -1, 0)
carrier_brake_position = xosc.LanePosition(6.957085132598877, 0, -1, 1)

# marker drone
marker_drone_speed = virtual_speed # m/s
marker_drone_acceleration = virtual_acceleration # m/s^2
marker_drone_retardation = virtual_retardation # m/s^2
marker_drone_O6_point = virtual_O4_point
marker_drone_brake_position = virtual_brake_position


denm_trigger_speed = 10 / 3.6 # m/s


# In[340]:


# Preamble
init = xosc.Init()
osc_params = xosc.ParameterDeclarations()
osc_catalogs = xosc.Catalog()
osc_catalogs.add_catalog(CATALOG_NAME, CATALOG_PATH)
osc_catalogs.add_catalog(CONTROLLER_CATALOG_NAME, CONTROLLER_CATALOG_PATH)
osc_road_network = xosc.RoadNetwork(roadfile=OPENDRIVE_PATH)
osc_entities = xosc.Entities()
osc_storyboard = xosc.StoryBoard(init)
osc_act = xosc.Act("start")

camera_drone_maneuver_group = xosc.ManeuverGroup(CAMERA_DRONE_ID + ",maneuver_group")
camera_drone_maneuver = xosc.Maneuver(CAMERA_DRONE_ID + ",maneuver")
camera_drone_maneuver_group.add_actor(CAMERA_DRONE_ID)

mondeo_maneuver_group = xosc.ManeuverGroup(MONDEO_ID + ",maneuver_group")
mondeo_maneuver = xosc.Maneuver(MONDEO_ID + ",maneuver")
mondeo_maneuver_group.add_actor(MONDEO_ID)

ufo_maneuver_group = xosc.ManeuverGroup(UFO_ID + ",maneuver_group")
ufo_maneuver = xosc.Maneuver(UFO_ID + ",maneuver")
ufo_maneuver_group.add_actor(UFO_ID)

virtual_maneuver_group = xosc.ManeuverGroup(VIRTUAL_ID + ",maneuver_group")
virtual_maneuver = xosc.Maneuver(VIRTUAL_ID + ",maneuver")
virtual_maneuver_group.add_actor(VIRTUAL_ID)

carrier_maneuver_group = xosc.ManeuverGroup(CARRIER_ID + ",maneuver_group")
carrier_maneuver = xosc.Maneuver(CARRIER_ID + ",maneuver")
carrier_maneuver_group.add_actor(CARRIER_ID)

marker_drone_maneuver_group = xosc.ManeuverGroup(MARKER_DRONE_ID + ",maneuver_group")
marker_drone_maneuver = xosc.Maneuver(MARKER_DRONE_ID + ",maneuver")
marker_drone_maneuver_group.add_actor(MARKER_DRONE_ID)


# In[341]:


# Starting position
# Starting positions
osc_entities.add_scenario_object(MONDEO_ID,
	xosc.CatalogReference(CATALOG_NAME, MONDEO_CATALOG_NAME),
	xosc.CatalogReference(CONTROLLER_CATALOG_NAME, "externalController"))
osc_entities.add_scenario_object(UFO_ID,
	xosc.CatalogReference(CATALOG_NAME, UFO_CATALOG_NAME),
	xosc.CatalogReference(CONTROLLER_CATALOG_NAME, "externalController"))
osc_entities.add_scenario_object(VIRTUAL_ID,
	xosc.CatalogReference(CATALOG_NAME, VIRTUAL_CATALOG_NAME),
	xosc.CatalogReference(CONTROLLER_CATALOG_NAME, "externalController"))
osc_entities.add_scenario_object(CARRIER_ID,
	xosc.CatalogReference(CATALOG_NAME, CARRIER_CATALOG_NAME),
	xosc.CatalogReference(CONTROLLER_CATALOG_NAME, "externalController"))
osc_entities.add_scenario_object(MARKER_DRONE_ID,
	xosc.CatalogReference(CATALOG_NAME, MARKER_DRONE_CATALOG_NAME),
	xosc.CatalogReference(CONTROLLER_CATALOG_NAME, "externalController"))
osc_entities = osc_entities.add_scenario_object(
	CAMERA_DRONE_ID,
	xosc.CatalogReference(CATALOG_NAME, DRONE_CATALOG_NAME),
	xosc.CatalogReference(CONTROLLER_CATALOG_NAME, "externalController"))

drone_start_teleport = xosc.TeleportAction(camera_drone_O5_point)
virtual_start_teleport = xosc.TeleportAction(virtual_O4_point)
ufo_start_teleport = xosc.TeleportAction(ufo_O2_point)
carrier_start_teleport = xosc.TeleportAction(carrier_O3_point)
mondeo_start_teleport = xosc.TeleportAction(mondeo_O1_point)
marker_drone_start_teleport = xosc.TeleportAction(marker_drone_O6_point)

mondeo_set_speed = xosc.AbsoluteSpeedAction(
	mondeo_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		mondeo_acceleration
	),
)

init.add_init_action(CAMERA_DRONE_ID, drone_start_teleport)
init.add_init_action(MONDEO_ID, mondeo_start_teleport)
init.add_init_action(MONDEO_ID, mondeo_set_speed)
init.add_init_action(UFO_ID, ufo_start_teleport)
init.add_init_action(VIRTUAL_ID, virtual_start_teleport)
init.add_init_action(CARRIER_ID, carrier_start_teleport)
init.add_init_action(MARKER_DRONE_ID, marker_drone_start_teleport)



# In[342]:


# Camera drone movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
times = []
positions = [
	camera_drone_O5_point,
	camera_drone_brake_position,
]
polyline = xosc.Polyline(times, positions)
camera_drone_trajectory = xosc.Trajectory(CAMERA_DRONE_ID + ",start_follow_trajectory", closed=False)
camera_drone_trajectory.add_shape(polyline)
camera_drone_follow_trajectory = xosc.FollowTrajectoryAction(
	camera_drone_trajectory,
	xosc.FollowingMode.position,
	xosc.ReferenceContext.relative,
	1,
	0
)
camera_drone_set_speed = xosc.AbsoluteSpeedAction(
	camera_drone_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		camera_drone_acceleration
	),
)
camera_drone_brake = xosc.AbsoluteSpeedAction(
	0,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		-camera_drone_retardation
	),
)

# Stop
drone_brake = xosc.AbsoluteSpeedAction(
	0,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.time,
		-camera_drone_acceleration
	)
)
drone_reach_brake_position = xosc.EntityTrigger(
    name=CAMERA_DRONE_ID + ",reach_brake_position",
    delay=0,
    conditionedge=xosc.ConditionEdge.rising,
    entitycondition=xosc.ReachPositionCondition(camera_drone_brake_position, 1.0), # tolerance
    triggerentity=CAMERA_DRONE_ID
)
drone_brake_event = xosc.Event(
	CAMERA_DRONE_ID + ",stop_event",
    xosc.Priority.overwrite,
)

drone_brake_event.add_trigger(drone_reach_brake_position)
drone_brake_event.add_action(CAMERA_DRONE_ID + ",brake_to_stop", drone_brake)
camera_drone_maneuver.add_event(drone_brake_event)


# In[343]:


# Virtual movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
times = []
positions = [
  virtual_O4_point,
	virtual_brake_position
]
polyline = xosc.Polyline(times, positions)
virtual_trajectory = xosc.Trajectory(VIRTUAL_ID + ",start_follow_trajectory", closed=False)
virtual_trajectory.add_shape(polyline)
virtual_follow_trajectory = xosc.FollowTrajectoryAction(
	virtual_trajectory,
	xosc.FollowingMode.position,
	xosc.ReferenceContext.relative,
	1,
	0
)
virtual_set_speed = xosc.AbsoluteSpeedAction(
	virtual_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		virtual_acceleration
	),
)
virtual_brake = xosc.AbsoluteSpeedAction(
	0,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		-virtual_retardation
	),
)
virtual_reach_stop_position = xosc.EntityTrigger(
	name=VIRTUAL_ID + ",reach_stop_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(virtual_brake_position, 1),
	triggerentity=VIRTUAL_ID
)
virtual_brake_event = xosc.Event(
	VIRTUAL_ID + ",brake_event",
	xosc.Priority.parallel
)
virtual_brake_event.add_trigger(virtual_reach_stop_position)
virtual_brake_event.add_action(VIRTUAL_ID + ",brake_to_stop", virtual_brake)
virtual_maneuver.add_event(virtual_brake_event)


# In[344]:


# UFO movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
times = []
positions = [
  ufo_O2_point,
	ufo_brake_position
]
polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(UFO_ID + ",start_follow_trajectory", closed=False)
trajectory.add_shape(polyline)
ufo_follow_trajectory = xosc.FollowTrajectoryAction(
	trajectory,
	xosc.FollowingMode.position,
	xosc.ReferenceContext.relative,
	1,
	0
)
ufo_set_speed = xosc.AbsoluteSpeedAction(
	ufo_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		ufo_acceleration
	),
)

ufo_brake = xosc.AbsoluteSpeedAction(
	0,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		-ufo_retardation
	),
)
ufo_reach_stop_position = xosc.EntityTrigger(
	name=UFO_ID + ",reach_stop_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(ufo_brake_position, 1),
	triggerentity=UFO_ID
)
ufo_brake_event = xosc.Event(
	UFO_ID + ",brake_event",
	xosc.Priority.parallel
)
ufo_brake_event.add_trigger(ufo_reach_stop_position)
ufo_brake_event.add_action(UFO_ID + ",brake_to_stop", ufo_brake)
ufo_maneuver.add_event(ufo_brake_event)


# In[345]:


# Carrier movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
times = []
positions = [
	carrier_O3_point,
    carrier_brake_position
]
polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(CARRIER_ID + ",start_follow_trajectory", closed=False)
trajectory.add_shape(polyline)
carrier_follow_trajectory = xosc.FollowTrajectoryAction(
	trajectory,
	xosc.FollowingMode.position,
	xosc.ReferenceContext.relative,
	1,
	0
)
carrier_set_speed = xosc.AbsoluteSpeedAction(
	carrier_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		carrier_acceleration
	),
)
carrier_brake = xosc.AbsoluteSpeedAction(
	0,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		-carrier_retardation
	),
)
carrier_reach_stop_position = xosc.EntityTrigger(
	name=CARRIER_ID + ",reach_stop_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(carrier_brake_position, 1),
	triggerentity=CARRIER_ID
)
carrier_brake_event = xosc.Event(
	CARRIER_ID + ",brake_event",
	xosc.Priority.parallel
)
carrier_brake_event.add_trigger(carrier_reach_stop_position)
carrier_brake_event.add_action(CARRIER_ID + ",brake_to_stop", carrier_brake)
carrier_maneuver.add_event(carrier_brake_event)


# In[346]:


# Marker drone movement profile:
# accelerate to top speed,
# follow a trajectory,
# then decelerate to a stop
marker_drone_follow_trajectory = virtual_follow_trajectory
marker_drone_follow_trajectory.trajectory.name = MARKER_DRONE_ID + ",start_follow_trajectory"
marker_drone_set_speed = xosc.AbsoluteSpeedAction(
	marker_drone_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		marker_drone_acceleration
	),
)
marker_drone_brake = xosc.AbsoluteSpeedAction(
	0,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		-marker_drone_retardation
	),
)
marker_drone_reach_stop_position = xosc.EntityTrigger(
	name=MARKER_DRONE_ID + ",reach_stop_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(marker_drone_brake_position, 1),
	triggerentity=MARKER_DRONE_ID
)
marker_drone_brake_event = xosc.Event(
	MARKER_DRONE_ID + ",brake_event",
	xosc.Priority.parallel
)
marker_drone_brake_event.add_trigger(marker_drone_reach_stop_position)
marker_drone_brake_event.add_action(MARKER_DRONE_ID + ",brake_to_stop", marker_drone_brake)
marker_drone_maneuver.add_event(marker_drone_brake_event)


# In[347]:


# Mondeo movement profile:
# accelerate to top speed,
# slow down before turn,
# turn,
# accelerate to top speed,

# Start moving
times = []
positions = [
    mondeo_O1_point,
    xosc.LanePosition(0, 0, 1, 3),
    xosc.LanePosition(8.0940648317337036, 0, 1, 12),
    xosc.LanePosition(7.0940648317337036, 0, 1, 12),
    xosc.LanePosition(6.0940648317337036, 0, 1, 12),
    xosc.LanePosition(5.0940648317337036, 0, 1, 12),
    xosc.LanePosition(4.0940648317337036, 0, 1, 12),
    xosc.LanePosition(3.0940648317337036, 0, 1, 12),
    xosc.LanePosition(2.0940648317337036, 0, 1, 12),
    xosc.LanePosition(1.0940648317337036, 0, 1, 12),
    xosc.LanePosition(33, 0, 1, 0),
    xosc.LanePosition(20, 0, 1, 0),
    mondeo_brake_position
]
polyline = xosc.Polyline(times, positions)
trajectory = xosc.Trajectory(
    MONDEO_ID + ",init_follow_trajectory", closed=False)
trajectory.add_shape(polyline)
mondeo_follow_trajectory = xosc.FollowTrajectoryAction(
    trajectory,
    xosc.FollowingMode.position,
    xosc.ReferenceContext.relative,
    1,
    0
)
mondeo_scenario_started = xosc.ValueTrigger(
    MONDEO_ID + ",scenario_started",
    0,
    xosc.ConditionEdge.none,
    xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan)
)
mondeo_follow_trajectory_event = xosc.Event(
    MONDEO_ID + ",mondeo_follow_trajectory_event",
    xosc.Priority.parallel
)
mondeo_follow_trajectory_event.add_trigger(mondeo_scenario_started)
mondeo_follow_trajectory_event.add_action(
    MONDEO_ID + ",init_follow_trajectory", mondeo_follow_trajectory)
mondeo_maneuver.add_event(mondeo_follow_trajectory_event)

# Slow down
mondeo_slow_down = xosc.AbsoluteSpeedAction(
	mondeo_slow_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		-mondeo_retardation
	),
)
mondeo_reach_slow_down_position = xosc.EntityTrigger(
	MONDEO_ID + ",reach_slow_down_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(mondeo_slow_down_position, 1),
	triggerentity=MONDEO_ID
)
mondeo_slow_down_event = xosc.Event(
	MONDEO_ID + ",mondeo_slow_down_event",
	xosc.Priority.parallel
)
mondeo_slow_down_event.add_trigger(mondeo_reach_slow_down_position)
mondeo_slow_down_event.add_action(MONDEO_ID + ",slow_down", mondeo_slow_down)
mondeo_maneuver.add_event(mondeo_slow_down_event)

# Accelerate
mondeo_accelerate = xosc.AbsoluteSpeedAction(
	mondeo_speed,
	xosc.TransitionDynamics(
		xosc.DynamicsShapes.linear,
		xosc.DynamicsDimension.rate,
		mondeo_acceleration
	),
)
mondeo_reach_accelerate_position = xosc.EntityTrigger(
	MONDEO_ID + ",reach_accelerate_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(mondeo_accelerate_position, 1),
	triggerentity=MONDEO_ID
)
mondeo_accelerate_event = xosc.Event(
	MONDEO_ID + ",mondeo_accelerate_event",
	xosc.Priority.parallel
)
mondeo_accelerate_event.add_trigger(mondeo_reach_accelerate_position)
mondeo_accelerate_event.add_action(MONDEO_ID + ",mondeo_accelerate", mondeo_accelerate)
mondeo_maneuver.add_event(mondeo_accelerate_event)

# Brake
mondeo_brake = xosc.AbsoluteSpeedAction(
    0,
    xosc.TransitionDynamics(
        xosc.DynamicsShapes.linear,
        xosc.DynamicsDimension.rate,
        -mondeo_retardation
    ),
)
mondeo_reach_brake_position = xosc.EntityTrigger(
    MONDEO_ID + ",reach_brake_position",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(mondeo_brake_position, 1),
    triggerentity=MONDEO_ID
)
mondeo_brake_event = xosc.Event(
    MONDEO_ID + ",mondeo_brake_event",
    xosc.Priority.parallel
)
mondeo_brake_event.add_trigger(mondeo_reach_brake_position)
mondeo_brake_event.add_action(MONDEO_ID + ",brake_to_stop", mondeo_brake)
mondeo_maneuver.add_event(mondeo_brake_event)


# In[348]:


# Mondeo triggers camera drone
mondeo_reached_camera_drone_trigger_position = xosc.EntityTrigger(
	name=MONDEO_ID + ",reached_camera_drone_trigger_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(mondeo_trigger_positions[CAMERA_DRONE_ID], 2),
	triggerentity=MONDEO_ID
)

camera_drone_move_event = xosc.Event(
	CAMERA_DRONE_ID + ",start_move_event",
	xosc.Priority.parallel
)
camera_drone_move_event.add_trigger(mondeo_reached_camera_drone_trigger_position)
camera_drone_move_event.add_action(CAMERA_DRONE_ID + ",start_follow_trajectory", camera_drone_follow_trajectory)
camera_drone_move_event.add_action(CAMERA_DRONE_ID + ",set_speed", camera_drone_set_speed)
camera_drone_maneuver.add_event(camera_drone_move_event)


# In[349]:


# Mondeo object triggers DENM warning

### DENM Action ###
mondeo_high_speed_detected = xosc.EntityTrigger(
	name=MONDEO_ID + ",high_speed_detected",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.SpeedCondition(denm_trigger_speed, xosc.Rule.greaterThan),
	triggerentity=MONDEO_ID)

mondeo_denm_action = xosc.VisibilityAction(graphics=True, traffic=True, sensors=True)

denm_osc_event = xosc.Event(
	MONDEO_ID + ",high_speed_event",
	xosc.Priority.parallel,
)

denm_osc_event.add_trigger(mondeo_high_speed_detected)
denm_osc_event.add_action(MONDEO_ID + ",send_denm",
							mondeo_denm_action)
mondeo_maneuver.add_event(denm_osc_event)


# In[350]:


# Mondeo triggers UFO
mondeo_reached_ufo_trigger_position = xosc.EntityTrigger(
	name=MONDEO_ID + ",reached_ufo_trigger_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(mondeo_trigger_positions[UFO_ID], 1),
	triggerentity=MONDEO_ID
)

ufo_move_event = xosc.Event(
	UFO_ID + ",start_move_event",
	xosc.Priority.parallel
)
ufo_move_event.add_trigger(mondeo_reached_ufo_trigger_position)
ufo_move_event.add_action(UFO_ID + ",start_follow_trajectory", ufo_follow_trajectory)
ufo_move_event.add_action(UFO_ID + ",set_speed", ufo_set_speed)
ufo_maneuver.add_event(ufo_move_event)


# In[351]:


# Mondeo triggers virtual
mondeo_reached_virtual_trigger_position = xosc.EntityTrigger(
	name=MONDEO_ID + ",reached_virtual_trigger_position",
	delay=virtual_trigger_delay,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(mondeo_trigger_positions[VIRTUAL_ID], 1),
	triggerentity=MONDEO_ID
)

virtual_move_event = xosc.Event(
	VIRTUAL_ID + ",start_move_event",
	xosc.Priority.parallel
)
virtual_move_event.add_trigger(mondeo_reached_virtual_trigger_position)
virtual_move_event.add_action(VIRTUAL_ID + ",start_follow_trajectory", virtual_follow_trajectory)
virtual_move_event.add_action(VIRTUAL_ID + ",set_speed", virtual_set_speed)
virtual_maneuver.add_event(virtual_move_event)


# In[352]:


# Mondeo triggers carrier
mondeo_reached_carrier_trigger_position = xosc.EntityTrigger(
	name=MONDEO_ID + ",reached_carrier_trigger_position",
	delay=0,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(mondeo_trigger_positions[CARRIER_ID], 1),
	triggerentity=MONDEO_ID
)

carrier_move_event = xosc.Event(
	CARRIER_ID + ",start_move_event",
	xosc.Priority.parallel
)
carrier_move_event.add_trigger(mondeo_reached_carrier_trigger_position)
carrier_move_event.add_action(CARRIER_ID + ",start_follow_trajectory", carrier_follow_trajectory)
carrier_move_event.add_action(CARRIER_ID + ",set_speed", carrier_set_speed)
carrier_maneuver.add_event(carrier_move_event)


# In[353]:


# Mondeo triggers marker drone
mondeo_reached_marker_drone_trigger_position = xosc.EntityTrigger(
	name=MONDEO_ID + ",reached_marker_drone_trigger_position",
	delay=virtual_trigger_delay,
	conditionedge=xosc.ConditionEdge.none,
	entitycondition=xosc.ReachPositionCondition(mondeo_trigger_positions[MARKER_DRONE_ID], 1),
	triggerentity=MONDEO_ID
)

marker_drone_move_event = xosc.Event(
	MARKER_DRONE_ID + ",start_move_event",
	xosc.Priority.parallel
)
marker_drone_move_event.add_trigger(mondeo_reached_marker_drone_trigger_position)
marker_drone_move_event.add_action(MARKER_DRONE_ID + ",start_follow_trajectory", marker_drone_follow_trajectory)
marker_drone_move_event.add_action(MARKER_DRONE_ID + ",set_speed", marker_drone_set_speed)
marker_drone_maneuver.add_event(marker_drone_move_event)


# In[354]:


# Collect into a scenario and write to file
mondeo_maneuver_group.add_maneuver(mondeo_maneuver)
camera_drone_maneuver_group.add_maneuver(camera_drone_maneuver)
ufo_maneuver_group.add_maneuver(ufo_maneuver)
virtual_maneuver_group.add_maneuver(virtual_maneuver)
carrier_maneuver_group.add_maneuver(carrier_maneuver)
marker_drone_maneuver_group.add_maneuver(marker_drone_maneuver)

osc_act.add_maneuver_group(mondeo_maneuver_group)
osc_act.add_maneuver_group(camera_drone_maneuver_group)
osc_act.add_maneuver_group(ufo_maneuver_group)
osc_act.add_maneuver_group(virtual_maneuver_group)
osc_act.add_maneuver_group(carrier_maneuver_group)
osc_act.add_maneuver_group(marker_drone_maneuver_group)

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

