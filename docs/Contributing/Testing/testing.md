# Testing

## Unit Tests
Unit tests for c++ code in ATOS are written in gtest. New modules are expected to have unit tests covering atlest the most important functionality. The unit tests are run automatically on every pull request. 

To run run individual unit tests, run the following command from the your atos build folder in your workspace:
```bash
cd ~/atos_ws/build/atos
ctest -R <test_name>
```

To run all unit tests you can just run "colcon test" from the root of your workspace.

