#EsminiAdapter
Module for interfacing with the [esmini](https://github.com/esmini/esmini) ScenarioEngine.
## About the module
This module has the following responsibilities:
    * Load an OpenSCENARIO-file and extract static trajectories from it.
    * Dynamically, while the test is running, execute actions based on triggers specified in the OpenSCENARIO-file.
Note: some refactoring is needed to divide the responsibilities of this module into two separate modules.

## ROS parameters
The following ROS parameters can be set for `EsminiAdapter`:

- `open_scenario_file` - Name of the OpenSCENARIO-file. The file must end in `.xosc` and be located in the `osc`-directory.


## Example
Load an OpenSCENARIO-file called `MyScenario.xosc`.

- `open_scenario_file: "MyScenario.xosc"`


