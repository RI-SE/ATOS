# MQTTBridge

## About the module
MQTTBridge is a module that allows you to publish V2X data from the ATOS system to a MQTT broker. The module subscribes to the atos/v2x_message and parses the content of this msg
to JSON which is then published over MQTT to a specified topic.  

Note! The module will shut if no broker ip is specified in the params.yaml
## Integration with EsminiAdapter
The module can be used togheter with the EsminiAdapter module to trigger V2X while running a OpenScenario file in ATOS. You can find more information how to set this up at [EsminiAdapter](./EsminiAdapter.md).

## ROS parameters
The following ROS parameters should be set in the params.yaml file:

```yaml
atos:
  mqtt_bridge:
    ros__parameters:
      broker_ip: ""     # Required. IP address of the MQTT broker.
      pub_client_id: "" # Name of the MQTT client used for publishing. A random number is appended to the name to avoid name collisions.
      username: ""      # Username if required by the broker.
      password: ""      # Password if required by the broker.
      topic: ""         #  Topic to publish to.
      quality_of_service: "" # QoS level to use for publishing. Can be 0, 1 or 2.
```

## atos/v2x_message

```bash
string message_type # DENM or CAM
string event_id 
uint8 cause_code
uint64 detection_time # millis since epoch
int32 altitude # in centimeters
int32 latitude # in microdegrees
int32 longitude # in microdegrees
```

## Testing the module

You can test the module by running the following command.

```bash
ros2 topic pub -r 1 /atos/v2x_message atos_interfaces/msg/V2x \
"{message_type: 'DENM', event_id: 'ATOSEvent', cause_code: 12, \
 altitude: 200, detection_time: 1674131259, latitude: 577063000, longitude: 119416860}"
```

This should result in a DENM message being published to the broker and topic specified in params.yaml.