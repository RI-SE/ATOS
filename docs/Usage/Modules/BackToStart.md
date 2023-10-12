# BackToStart
A module for generating an object's return trajectory.

## About the module
This module provides a service to reverse the test object's trajectory and generate a Williamson turn at the start and end to let the object return to its starting point. Execute the new trajectories similar to a normal test with `Arm` and `Start`. It is up to the operator to determine if the new trajectories are possible to execute or not. The service is only allowed in `Disarmed` state and calling `Reset test` in the GUI does not result in a state change. 