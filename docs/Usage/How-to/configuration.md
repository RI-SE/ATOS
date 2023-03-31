# Configurations

## The test directory
After starting ATOS the first time, you will find the test directory in `~/.astazero/ATOS`. This directory will contain all test settings and will contain the following directories:

- **Catalogs**
    - Explanation here: Catalog directory containing various OpenSCENARIO-files with settings and parameters used by esmini. 
    - Supported file types: `.xosc`
- **certs**
    - Explanation here: Directory containing the certificates used by e.g. the web-gui.
    - Supported file types: `.pem`
- **conf**
    - Explanation here:  Directory containing the configuration files used to configure ATOS, i.e. the ROS parameters located in the file params.yaml.
    - Supported file types: `.yaml`
- **journal**
    - Explanation here: Directory containing the journal files, i.e. recorded configuration/state/position of each object for the duration of a test.
    - Supported file types: `.jnl`
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
    - Explanation here: Legacy directory containing trajectory files.
    - Supported file types: `.traj`


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