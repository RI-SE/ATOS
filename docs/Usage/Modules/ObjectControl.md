# ObjectControl

## About the module
Short description what the module does and what to use it for.

## ROS parameters
The following ROS parameters can be set for `ObjectControl`:

- `max_missing_heartbeats` - The number of position update (MONR) message periods that are allowed to pass since the last received message before an abort signal is sent to all objects. 
- `transmitter_id` - The ISO 22133 transmitted id to be used for ATOS.


## Examples
### Example 1
At most 3 position updates missing, and transmitter ID set to 175:
- `max_missing_heartbeats: 3`
- `transmitter_id: 175`
