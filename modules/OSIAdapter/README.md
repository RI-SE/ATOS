# About the module
`OSIAdapter` is used to translate `MONR`-data into OSI-data, and then send it over either `tcp` or `udp` for all objects used in a test. The following data is currently sent for each object:
- `Object ID`
- `Position: x, y, z`
- `Velocity: x, y, z`
- `Acceleration, x, y, z`
- `Orientation: yaw`

# ROS parameters
The following ROS parameters can be set for `OSIAdapter`:
- `address` - IP address for client to connect to.
- `port` - Port for client to connect to.
- `protocol` - Which protocol to use, use `"tcp"` or `"udp"`.
- `frequency` - Frequency for sending data, measured in Hz.

## Examples
### Example 1
Using any address on port 55555, using udp and a frequency of 50 Hz.
- `address: "0.0.0.0"`
- `port: 55555`
- `protocol: "udp"`
- `frequency: 50`


### Example 2
Using localhost on port 12345, using tcp and a frequency of 100 Hz.
- `address: "127.0.0.1"`
- `port: 12345`
- `protocol: "tcp"`
- `frequency: 100`
