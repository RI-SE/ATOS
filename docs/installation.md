# Installation
Below are the steps for building ATOS for the first time with colcon.

Prerequisites: C/C++ compiler, CMake (minimum version 3.10.2)


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
git clone https://github.com/RI-SE/ATOS.git
cd ATOS
git submodule update --init --recursive
```

```bash
sudo apt install libsystemd-dev libprotobuf-dev protobuf-compiler \
libeigen3-dev ros-foxy-paho-mqtt-c nlohmann-json3-dev npm nodejs libpcl-dev
```

Then fetch the submodules [iso22133](https://github.com/RI-SE/iso22133) and [atos-interfaces](https://github.com/RI-SE/atos_interfaces):
```bash
git submodule update --init --recursive
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

## <a name="atos"></a> Installing ATOS

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

Launch ATOS
```bash
ros2 launch atos launch_basic.py
```

### <a name="optional-builds--installations"></a> Optional builds & installations
ATOS can be installed in alternative ways, and built with support for various optional modules, described here.

### <a name="relativekinematics"></a> How to build with RelativeKinematics instead of ObjectControl

The server will build the ObjectControl with AbsolutKinematics by default. It's possible to build with RelativeKinematics support by rebuilding with the argument -DWITH_RELATIVE_KINEMATICS=ON, see following command
```sh
colcon build --cmake-args -DWITH_RELATIVE_KINEMATICS=ON
```
To include ObjectControl in the build again run the same command with OFF, as follows
```sh
colcon build --cmake-args -DWITH_RELATIVE_KINEMATICS=OFF
```
