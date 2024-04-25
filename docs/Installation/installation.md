# Instructions
There are two ways of starting ATOS: using the docker image or building from source. The docker image is the easiest way to get started, but if you intend to make changes to ATOS, we recommend building from source with the setup_atos.sh script.

## <a name="docker"></a> Running with docker
To run ATOS using docker compose, first [install docker](https://docs.docker.com/engine/install/) on your computer. Download the [ATOS repo](https://github.com/RI-SE/ATOS). Then, run the following command from the root repo directory:
```bash
docker compose up
```
or if you are not in the docker group:
```bash
sudo -E docker compose up
```

Note: If no preexisting .astazero/ATOS folder is found, the docker image will create one. This folder will have root user permissions due to the way the docker image is built. For ease of use, change owner to your own user, run the following command on the host machine:
```bash 
sudo chown -R $USER:$USER ~/.astazero/ATOS/
```

## <a name="Installation script"></a> Running natively
ATOS comes with an installation script that automates the installation process. It is intended for use on Ubuntu 22.04. The script will install ROS 2, ATOS dependencies, and ATOS itself. It will also create a workspace and build ATOS. The script can be executed using the following command:
```bash
./setup_atos.sh
```

## <a name="running"></a> Starting ATOS
Launch ATOS
```bash
ros2 launch atos launch_basic.py insecure:=True
```
See the [GUI](../Usage/GUI/foxglove.md) documentation on how to enable secure connections. 

When running ATOS for the first time and the no pre-existing .astazero/ATOS folder is found, ATOS will create some barebone configuration files which you can read more about in the configuration section [here](../Usage/How-to/configuration.md).
