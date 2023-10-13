# Instructions
There are two ways of starting ATOS: using the docker image or building from source. The docker image is the easiest way to get started, but if you intend to make changes to ATOS, we recommend building from source.

## <a name="docker"></a> Using the docker image
To run ATOS using docker compose, first [install docker](https://docs.docker.com/engine/install/) on your computer. Download the [ATOS repo](https://github.com/RI-SE/ATOS). Then, run the following command from the root repo directory:
```bash
docker compose up
```
or if you are not in the docker group:
```bash
sudo -E docker compose up
```

If you run Docker Engine you can start ATOS using the computers own network stack with
```bash
docker run --network="host"  --ipc=host --privileged -it -v ~/.astazero/ATOS/:/root/.astazero/ATOS/ astazero/atos_docker_env:latest bash -c "source /root/atos_ws/install/setup.sh ; ros2 launch atos launch_basic.py"
```
If you run Docker Desktop, special care must be taken to allow the container running in the Docker Desktop VM to communicate with clients running on the host network. If uncertain about this we suggest installing Docker Engine instead. 

Note: If no preexisting .astazero/ATOS folder is found, the docker image will create one. This folder will have root user permissions due to the way the docker image is built. For ease of use, change owner to your own user, run the following command on the host machine:
```bash 
sudo chown -R $USER:$USER ~/.astazero/ATOS/
```

## <a name="Installation script"></a> Using the installation script
ATOS comes with an installation script that automates the installation process. It is intended for use on Ubuntu 20.04 or 22.04, and has been tested on a fresh install of Ubuntu 20.04. The script will install ROS 2, ATOS dependencies, and ATOS itself. It will also create a workspace and build ATOS. The script can be executed using the following command:
```bash
./install_atos.sh
```

## <a name="Native build"></a> Building from source manually
The following instructions are for installing ATOS manually on Ubuntu 20.04.

## <a name="ros2"></a> Installing ROS 2

ATOS is based on ROS 2, and requires ROS 2 to be installed on the host computer. The following instructions are for installing ROS 2 on Ubuntu 20.04.

Download ROS 2 prerequisites:
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
```

Authorize the ROS 2 gpg key with apt:
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  \
-o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repo to sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
### Installing ROS 2 Foxy
ATOS Supports both ROS 2 Foxy and ROS 2 Humble. Chose one of the versions, we recommend Foxy as the default choice.
 Instructions for Humble follow in the next subsection.

 To install ROS 2 Foxy, do:
```bash
sudo apt update
sudo apt install ros-foxy-desktop python3-colcon-common-extensions ros-foxy-nav-msgs \
ros-foxy-geographic-msgs ros-foxy-foxglove-msgs ros-foxy-sensor-msgs \
ros-foxy-rosbridge-suite ros-foxy-pcl-conversions
```

source the setup script:
```bash
source /opt/ros/foxy/setup.bash
```
Add the above line to ~/.bashrc or similar startup script to automate this process.

### Installing ROS 2 Humble
If you instead want to install ROS 2 Humble, do:
```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions ros-humble-nav-msgs \
ros-humble-geographic-msgs ros-humble-foxglove-msgs ros-humble-sensor-msgs \
ros-humble-rosbridge-suite ros-humble-pcl-conversions \
ros-humble-foxglove-bridge
```

and source the setup script:
```bash
source /opt/ros/humble/setup.bash
```
Add the above line to ~/.bashrc or similar startup script to automate this process.

## <a name="dependencies"></a> Dependencies & external libraries
In order to build ATOS, dependencies and exernal libraries need to be installed. First install the necessary development packages:

clone ATOS in your git folder, and make sure that all submodules are present and up to date:
```bash
git clone git@github.com:RI-SE/ATOS.git
cd ATOS
git submodule update --init --recursive
```

Install the dependencies (Stand in the repo directory where the dependencies.txt file is located):
```bash
```bash
apt_deps=$(cat dependencies.txt | tr '\n' ' ') | sudo apt update && sudo apt install -y ${apt_deps}
```


Lastly, the following external libraries need to be installed:

- [OpenSimulationInterface v3.4.0](https://github.com/OpenSimulationInterface/open-simulation-interface)
- [esmini](https://github.com/esmini/esmini)

### <a name="osi"></a> Installing OpenSimulationInterface v3.4.0

```bash
git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0
cd open-simulation-interface
mkdir build && cd build
cmake .. && make
sudo make install
```

Make sure that the linker knows where OpenSimulationInterface is located:
```bash
sudo sh -c "echo '/usr/local/lib/osi3' > /etc/ld.so.conf.d/osi3.conf"
sudo ldconfig
```

### <a name="esmini"></a> Installing esmini
1. Begin by installing esmini dependencies listed under section 2.3 on the page [https://esmini.github.io/](https://esmini.github.io/)

2. Then install esmini:
```bash
git clone https://github.com/esmini/esmini
cd esmini
mkdir build && cd build
cmake .. && make
sudo make install
cp ../bin/libesminiLib.so /usr/local/lib
cp ../bin/libesminiRMLib.so /usr/local/lib
sudo mkdir -p /usr/local/include/esmini/
cp ../EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp /usr/local/include/esmini/
cp ../EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp /usr/local/include/esmini/
sudo ldconfig
```

## <a name="installing"></a> Installing ATOS

Create a workspace:
```bash
mkdir -p ~/atos_ws/src
```

Create symlinks to atos and atos_interfaces
```bash
ln -s path/to/ATOS ~/atos_ws/src/atos
ln -s path/to/ATOS/atos_interfaces ~/atos_ws/src/atos_interfaces
```

Change directory into the workspace and build
```bash
cd ~/atos_ws
colcon build
```

Source the project setup file:
```bash
source ~/atos_ws/install/setup.bash
```
Also add the above line to ~/.bashrc or similar.

## <a name="running"></a> Running ATOS
Launch ATOS
```bash
ros2 launch atos launch_basic.py
```
When running ATOS for the first time and the no pre-existing .astazero/ATOS folder is found, ATOS will create some barebone configuration files which you can read more about in the configuration section [here](../Usage/How-to/configuration.md).
