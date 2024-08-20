# OpenScenarioGateway

## About the module
This is module load and holds the chosen scenario in memory and contains various utility functions to interact with the scenario. 

## Features

*IP settings* 
ScenarioModule reads the IP address for each object from the ASAM Vehicle Catalog file (located in the ~/.astazero/ATOS/Catalogs/Vehicles/VehicleCatalog directory).
The IP address must be defined as a property in the Vehicle Catalog file for each object. The property must be named `ip` and have the IP address as its value.

´´´xml
<Properties>
    <Property name="ip" value="127.0.0.1"/>
</Properties>
´´´

## Usage

The following ROS parameters can be set for `ScenarioModule`:
```yaml
atos:
  object_control:
    ros__parameters:
      open_scenario_file: "scenario_name.xosc"     # The name of the scenario to open. Located in the ~/.astazero/ATOS/osc directory.
      active_object_names: ["object1", "object2"]  # List of object names to be active in the scenario. An empty list means all objects are active.
```

## Using CustomCommandActions

The ASAM OpenX standard supports adding user defied actions to the scenario. These actions are called `CustomCommandActions`. The `OpenScenarioGateway` module provides a ROS publisher that will publish a message to the `/custom_command_action` topic when a `CustomCommandAction` is encountered in the scenario. The message will contain the type of the action and the content (details: https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/content/CustomCommandAction.html)

In the code below is an example of how you could add a `CustomCommandAction` to an Event. This code uses the scenariogeneration python library and will trigger a POST REST request with the RESTGateway module:

```python
car_reached_trigger_position = xosc.EntityTrigger(
    name="car_reached_trigger_position",
    delay=0,
    conditionedge=xosc.ConditionEdge.none,
    entitycondition=xosc.ReachPositionCondition(
        xosc.LanePosition(0, 0, 1, 3), 2
    ),
    triggerentity=CAR_ID,
)

custom_command_content = json.dumps(
    {
        "endpoint": "http://localhost:8080",
        "data": {
            "my_custom_command": "my_custom_command_data"
        },
    }
)

my_action = xosc.UserDefinedAction(
    custom_command_action=xosc.CustomCommandAction(type="POST_JSON", content=custom_command_content)
)

my_event = xosc.Event(
    CAR_ID + ",high_speed_event",
    xosc.Priority.parallel,
)

my_event.add_trigger(car_reached_trigger_position)
my_event.add_action("MyCustomAction", my_action)
car_maneuver.add_event(my_event)
```