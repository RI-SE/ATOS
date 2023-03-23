# Configurations

## The test directory
After starting ATOS the first time, you will find the test directory in `~/.astazero/ATOS`. This directory will contain all test settings and will contain the following directories:

- **Catalogs**
    - Explanation here: 
    - Supported file types:
- **certs**
    - Explanation here: 
    - Supported file types:
- **conf**
    - Explanation here: 
    - Supported file types:
- **geofence**
    - Explanation here: 
    - Supported file types:
- **journal**
    - Explanation here: 
    - Supported file types:
- **objects**
    - Explanation here: Directory containing all objects that should be used in a test.
    - Supported file types: `.opro`
- **odr**
    - Explanation here: Directory containing OpenDRIVE-files.
    - Supported file types: `.xodr`
- **osc**
    - Explanation here: Directory containing the OpenSCENARIO-files.
    - Supported file types: `.xosc`
- **pointclouds**
    - Explanation here: Directory containing site scans as pointclouds.
    - Supported file types: `.pcd`
- **traj**
    - Explanation here: 
    - Supported file types:


## Changing ROS parameters
In the `conf`-directory, `params.yaml` is located and sets ROS-parameters for the ATOS modules, which are read when launching ATOS. The parameters are described in their respective module:

- [ATOSBase](../Modules/ATOSBase.md)
- [EsminiAdapter](../Modules/EsminiAdapter.md)
- [MQTTBridge](../Modules/MQTTBridge.md)
- [ObjectControl](../Modules/ObjectControl.md)
- [OSIAdapter](../Modules/OSIAdapter.md)
- [PointcloudPublisher](../Modules/PointcloudPublisher.md)
- [SystemControl](../Modules/SystemControl.md)
- [TrajectoryletStreamer](../Modules/TrajectoryletStreamer.md)