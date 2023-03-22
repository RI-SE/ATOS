# ROS parameters
The follwing ROS parameters can be set for `ATOSBase`:
- `test_origin_latitude` - The test origin's latitude in decimal degrees.
- `test_origin_longitude` - The test origin's longitude in decimal degrees.
- `test_origin_altitude` - The test origin's altitude in meters.
- `test_origin_rot` - The test origin's rotation in degrees.

# Examples
## Example 1
Setting the test origin at AstaZero's office in Gothenburg, with no rotation.
- `test_origin_latitude: 57.7072583357822`
- `test_origin_longitude: 11.940293773902779`
- `test_origin_altitude: 1.0`
- `test_origin_rot: 0.0`


## Example 2
Setting the test origin at AstaZero's test track outside of Gothenburg, with 90 degrees rotation.
- `test_origin_latitude: 57.77907816522942`
- `test_origin_longitude: 12.779735711698871`
- `test_origin_altitude: 193.0`
- `test_origin_rot: 90.0`