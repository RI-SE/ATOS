# About the module
Short description what the module does and what to use it for.

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
