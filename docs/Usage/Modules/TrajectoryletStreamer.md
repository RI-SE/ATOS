# TrajectoryletStreamer
An experimental module for generating piecewise trajectory "chunks" based on a known trajectory. It is indended for testing dynamic trajectory functionality in other modules with a trajectory that is known before hand.

# ROS parameters
The following ROS parameters can be set for `TrajectoryletStreamer`:

- `chunk_duration` - Length of the chunks to be transmitted, in seconds. The current time is used to find a chunk start point in the trajectory and the chunk length marks the end of that chunk.
