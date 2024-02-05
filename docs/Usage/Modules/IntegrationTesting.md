# IntegrationTesting

## About the module
`IntegrationTesting` is used for integration tests. The module launches an integration testing handler, which reads from `params.yaml` which integration tests we want to execute. A node for each integration
test is then launched, and the node performs the specified integration tests. An integration tests can for instance be if an object reports all states correctly during a run, or if it follows a trajectory
correctly. This module can be run both offline and through GitHub Actions.

!!! note

    You can read more about integration testing in the section [Testing](../../Contributing/Testing/testing.md).

## ROS parameters
The integration tests are specified in `params.yaml`, and can be set to true of false depending on if we want to run them or not. The following ROS parameters can be set for `IntegrationTesting`:

```yaml
integration_testing_handler:
    ros__parameters:
      scenario_execution: true # set to true or false depending if you want to run this integration test
```
