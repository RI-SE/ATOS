# Configuration

## The test directory
After starting ATOS the first time, you will find the test directory `.astazero/ATOS` in your home folder on the host machine (or where you have specified it to be). 
This directory contains all configuration settings and journals which are located in the following directories:

- **Catalogs**
    - Explanation: Catalog directory containing various OpenSCENARIO-files with settings and parameters used by scenario engine esmini. 
        - Vehicle/VehicleCatalog.xosc - Contains object details about vehicles that can be used in a test.
        Mandatory properties are the "ip" and "model_id". The "ip" is the IP-address of the vehicle and the "model_id" is the ID which must be the same as the ID in the object .opro file in the objects folder.
- **certs**
    - Explanation: Directory containing the certificates used by e.g. the web-gui.
- **conf**
    - Explanation: Directory containing the configuration files used to configure ATOS, i.e. the ROS parameters located in the file params.yaml. Find more information below.
- **journal**
    - Explanation: Directory containing the journal files, i.e. recorded configuration/state/position of each object for the duration of a test.
- **objects**
    - Explanation: Directory containing all objects that should be used in a test.
        - Each object present in the test must have a corresponding .opro file in this directory. Each .opro file contains a row with the format 
`ID={objetcID}` where `{objectID}` must be the same as the "model_id" in the VehicleCatalog.xosc file.
- **odr**
    - Explanation: Directory containing OpenDRIVE-files.
- **osc**
    - Explanation: Directory containing the OpenSCENARIO-files.
- **pointclouds**
    - Explanation: Directory containing site scans as point clouds.


## Changing ROS parameters
In the `conf`-directory, `params.yaml` is located and sets ROS-parameters for the ATOS modules, which are read when launching ATOS. The parameters are described in their respective module:

- [ATOSBase](../Modules/ATOSBase.md)
- [EsminiAdapter](../Modules/EsminiAdapter.md)
- [MQTTBridge](../Modules/MQTTBridge.md)
- [ObjectControl](../Modules/ObjectControl.md)
- [OSIAdapter](../Modules/OSIAdapter.md)
- [PointcloudPublisher](../Modules/PointcloudPublisher.md)
- [TrajectoryletStreamer](../Modules/TrajectoryletStreamer.md)
