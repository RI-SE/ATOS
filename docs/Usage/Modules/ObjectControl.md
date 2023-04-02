# ObjectControl
A module for controlling test objects according to the ISO 22133 standard.
## About the module
This module has many responsibilities:
- Establishes and tracks connections with all test objects
- Keeps track of test object states
- Collects position data from objects, and record that data
- Transmit safety heartbeats to the objects
- Configures objects with trajectories and other settings
- Convert input from other modules into or from the ISO 22133 protocol
- Hold the ATOS system state (ISO 22133 control center status)
- Convert positional data into VUT-relative coordinates, if desired

## ROS parameters
The following ROS parameters can be set for `ObjectControl`:

```yaml
atos:
  object_control:
    ros__parameters:
      max_missing_heartbeats: 1     # The number of position update (MONR) message periods that are allowed to pass since the last received message before an abort signal is sent to all objects. 
      transmitter_id: 110           # The ISO 22133 transmitted id to be used for ATOS.
```

## Examples
### Example 1
At most 3 position updates missing, and transmitter ID set to 175:
```yaml
atos:
  object_control:
    ros__parameters:
      max_missing_heartbeats: 3
      transmitter_id: 175
```
