# ATOS 
<img align="left" width="100" height="100" src="./doc/ATOS_icon.svg">

ATOS, the communication hub for all test objects, controls and monitors iso22133-compliant vehicles and equipment. To build ATOS follow the guide below.

<br />
<br />


# Table of contents
- [ATOS](#atos)
- [Table of contents](#table-of-contents)
- [ Building ATOS with colcon](#-building-atos-with-colcon)
  - [ Dependencies \& external libraries](#-dependencies--external-libraries)
    - [ Installing OpenSimulationInterface v3.4.0](#-installing-opensimulationinterface-v340)
    - [ Installing atos-interfaces](#-installing-atos-interfaces)
    - [ Installing esmini](#-installing-esmini)
  - [ Installing ROS2 and building for the first time with colcon](#-installing-ros2-and-building-for-the-first-time-with-colcon)
    - [ Ubuntu 20.04](#-ubuntu-2004)
- [ Optional builds \& installations](#-optional-builds--installations)
  - [ How to build with RelativeKinematics instead of ObjectControl](#-how-to-build-with-relativekinematics-instead-of-objectcontrol)
- [User manual](#user-manual)
  - [The test directory](#the-test-directory)
  - [Changing ROS parameters](#changing-ros-parameters)
  - [Using ATOS with a Graphical User Interface (GUI)](#using-atos-with-a-graphical-user-interface-gui)
    - [Simple Control](#simple-control)
    - [Foxglove Studio](#foxglove-studio)
      - [Connect to ATOS](#connect-to-atos)
      - [Extensions](#extensions)
      - [Changing layout](#changing-layout)
      - [Panels in Foxglove Studio](#panels-in-foxglove-studio)

# User manual

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
- [ATOSBase](./modules/ATOSBase/README.md)
- [EsminiAdapter](./modules/EsminiAdapter/README.md)
- [MQTTBridge](./modules/MQTTBridge/README.md)
- [ObjectControl](./modules/ObjectControl/README.md)
- [OSIAdapter](./modules/OSIAdapter/README.md)
- [PointcloudPublisher](./modules/PointcloudPublisher/README.md)
- [SystemControl](./modules/SystemControl/README.md)
- [TrajectoryletStreamer](./modules/TrajectoryletStreamer/README.md)