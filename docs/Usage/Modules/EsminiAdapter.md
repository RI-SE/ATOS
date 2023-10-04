#EsminiAdapter
Module for interfacing with the [esmini](https://github.com/esmini/esmini) ScenarioEngine.
## About the module
This module has the following responsibilities:
    * Load an OpenSCENARIO-file and extract static trajectories from it.
    * From the associated OpenDrive file, extract the test origin and make available to other modules.
    * Dynamically, while the test is running, execute actions based on triggers specified in the OpenSCENARIO-file.
Note: some refactoring is needed to divide the responsibilities of this module into two separate modules.

## ROS parameters
The following ROS parameters can be set for `EsminiAdapter`:

- `open_scenario_file` - Name of the OpenSCENARIO-file. The file must end in `.xosc` and be located in the `osc`-directory.


## Example
Load an OpenSCENARIO-file called `MyScenario.xosc`.

- `open_scenario_file: "MyScenario.xosc"`

## Test origin

The test origin is extracted from the OpenDrive file of the scenario. To change the test origin to a different location, change the "geoReference" tag in the OpenDrive file header. 

*Example*
```xml
<OpenDRIVE>
  <header ... >
    <geoReference><![CDATA[+proj=tmerc +lat_0=57.77752905043481 +lon_0=12.7814573051027 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs ]]></geoReference>
  </header>
```