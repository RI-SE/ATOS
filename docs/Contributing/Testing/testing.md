# Testing

## Unit Tests
Unit tests for c++ code in ATOS are written in gtest. New modules are expected to have unit tests covering at least the most important functionality. All unit tests will run automatically on every pull request. 

To run individual unit tests, run the following command from the your atos build folder in your workspace:
```bash
cd ~/atos_ws/build/atos
ctest -R <test_name>
```

To run all unit tests you can just run 
```bash
colcon test --event-handlers console_cohesion+
```
from the root folder of your workspace.

