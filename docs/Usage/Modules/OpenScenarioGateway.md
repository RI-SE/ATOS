# SampleModule

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