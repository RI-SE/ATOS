# Instructions
There are two ways of starting ATOS: using the docker image or building from source. The docker image is the easiest way to get started, but if you intend to make changes to ATOS, we recommend building from source.

## <a name="docker"></a> Using the docker image
To run ATOS using the docker image, first install docker on your computer. Then, run the following command from the root repo directory:
```bash
docker compose up
```

If you run Docker Engine you can start ATOS using the computers own network stack with
```bash
docker run --network="host"  --ipc=host --privileged -it -v ~/.astazero/ATOS/:/root/.astazero/ATOS/ astazero/atos_docker_env:latest bash -c "source /root/atos_ws/install/setup.sh ; ros2 launch atos launch_basic.py"
```
If you run Docker Desktop you will need to specify the ports to expose to the host computer.
```bash
docker run --ipc=host --privileged -it -v ~/.astazero/ATOS/:/root/.astazero/ATOS/ -p 80:80 -p 8080:8080 -p 8081:8081 -p 8082:8082 -p 3000:3000 -p 3443:3443 -p 55555:55555 -p 443:443 -p 9090:9090 astazero/atos_docker_env:latest bash -c "source /root/atos_ws/install/setup.sh ; ros2 launch atos launch_basic.py"
```

You might wish to mount the config directory at ~/.astazero/ATOS/ to a different location on your host computer. This can be done by changing the path after the -v flag in the above command. You might also wish to inspect the image with instead of running the launch_basic.py script. This can be done by removing the last "bash -c ..." part of the command.

## <a name="Installation script"></a> Using the installation script
ATOS comes with an installation script that automates the installation process. It is intended for use on Ubuntu 20.04 or 22.04, and has been tested on a fresh install of Ubuntu 20.04. The script will install ROS2, ATOS dependencies, and ATOS itself. It will also create a workspace and build ATOS. The script can be run using the following command:
```bash
./install_atos.sh
```

## <a name="Native build"></a> Building from source manually
The following instructions are for installing ATOS manually on Ubuntu 20.04.

## <a name="ros2"></a> Installing ROS2

ATOS is based on ROS2, and requires ROS2 to be installed on the host computer. The following instructions are for installing ROS2 on Ubuntu 20.04.

Download ROS2 prerequisites:
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
```

Authorize the ROS2 gpg key with apt:
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  \
-o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS2 repo to sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ros foxy for desktop and colcon
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

## <a name="dependencies"></a> Dependencies & external libraries
In order to build ATOS, dependencies and exernal libraries need to be installed. First install the necessary development packages:

clone ATOS in your git folder, and make sure that all submodules are present and up to date:
```bash
git clone git@github.com:RI-SE/ATOS.git
cd ATOS
git submodule update --init --recursive
```

Install the dependencies:
```bash
sudo apt install libsystemd-dev libprotobuf-dev protobuf-compiler \
libeigen3-dev ros-foxy-paho-mqtt-c nlohmann-json3-dev npm nodejs libpcl-dev
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
