# Instructions
There are two ways of starting ATOS: using the docker image or building from source. The docker image is the easiest way to get started, but if you intend to make changes to ATOS, we recommend building from source with the setup_atos.sh script.

## <a name="docker"></a> Running with docker
To run ATOS with docker, first [install docker](https://docs.docker.com/engine/install/) on your computer and download the [ATOS repo](https://github.com/RI-SE/ATOS). Then follow the [quick start guide](./quickstart.md), which starts ATOS with docker compose, connects it to a virtual test object and runs a scenario.

## <a name="Installation script"></a> Running natively
ATOS comes with an installation script that automates the installation process. It is intended for use on Ubuntu 20.04, 22.04 or 24.04. The script will install ROS 2, ATOS dependencies, and ATOS itself. It will also create a workspace and build ATOS. The script can be executed using the following command:
```bash
./setup_atos.sh
```

!!! note
    The ATOS GUI ([control panel](../Usage/GUI/controlpanel.md)) requires Python 3.10 or newer, so it is **not available on Ubuntu 20.04 / ROS 2 Foxy**. ATOS is installed and runs there without the GUI.

## <a name="running"></a> Starting ATOS
Launch ATOS
```bash
ros2 launch atos launch_basic.py insecure:=True
```

When running ATOS for the first time and the no pre-existing .astazero/ATOS folder is found, ATOS will create some barebone configuration files which you can read more about in the configuration section [here](../Usage/How-to/configuration.md).
