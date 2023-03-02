# ATOS 
<img align="left" width="100" height="100" src="./doc/ATOS_icon.svg">

The ATOS server is a communication hub for all test objects. The server monitors and controls the test objects and is also responsible for creating logfiles. To build ATOS follow the guide below.

<br />
<br />


# Table of contents
- [ATOS](#atos)
- [Table of contents](#table-of-contents)
- [ Building ATOS with colcon](#-building-atos-with-colcon)
  - [ Dependencies \& external libraries](#-dependencies--external-libraries)
    - [ Installing OpenSimulationInterface v3.4.0](#-installing-opensimulationinterface-v340)
    - [ Installing atos-interfaces](#-installing-atos-interfaces)
    - [ Installing ad-xolib](#-installing-ad-xolib)
    - [ Installing esmini](#-installing-esmini)
  - [ Installing ROS2 and building for the first time with colcon](#-installing-ros2-and-building-for-the-first-time-with-colcon)
    - [ Ubuntu 20.04](#-ubuntu-2004)
- [ Optional builds \& installations](#-optional-builds--installations)
  - [ How to build with RelativeKinematics instead of ObjectControl](#-how-to-build-with-relativekinematics-instead-of-objectcontrol)

# <a name="ATOS"></a> Building ATOS with colcon
Below are the steps for building ATOS for the first time with colcon.

Prerequisites: C/C++ compiler, CMake (minimum version 3.10.2)

## <a name="dependencies"></a> Dependencies & external libraries
In order to build ATOS, dependencies and exernal libraries need to be installed. First install the necessary development packages:
```
sudo apt install libsystemd-dev libprotobuf-dev protobuf-compiler libeigen3-dev ros-foxy-paho-mqtt-c nlohmann-json3-dev
```

Then, the following external libraries need to be installed:
- [OpenSimulationInterface v3.4.0](https://github.com/OpenSimulationInterface/open-simulation-interface)
- [atos-interfaces](https://github.com/RI-SE/atos-interfaces)
- [ad-xolib](https://github.com/javedulu/ad-xolib)
- [esmini](https://github.com/esmini/esmini)

### <a name="osi"></a> Installing OpenSimulationInterface v3.4.0
```
git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0
cd open-simulation-interface
mkdir build && cd build
cmake .. && make
sudo make install
```
Make sure that the linker knows where OpenSimulationInterface is located:
```
sudo sh -c "echo '/usr/local/lib/osi3' > /etc/ld.so.conf.d/osi3.conf"
sudo ldconfig
```

### <a name="atos-interfaces"></a> Installing atos-interfaces
```
git clone https://github.com/RI-SE/atos_interfaces
```


### <a name="ad-xolib"></a> Installing ad-xolib
```
git clone https://github.com/javedulu/ad-xolib.git
cd ad-xolib
git submodule update --init --recursive
mkdir build && cd build
cmake .. -DBUILD_EMBED_TARGETS=OFF && make
sudo make install
sudo ldconfig
```

### <a name="esmini"></a> Installing esmini
Begin by installing esmini dependencies listed under section 2.3 on the page https://esmini.github.io/
```
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


## <a name="ros2"></a> Installing ROS2 and building for the first time with colcon
### <a name="ubuntu-20.04"></a> Ubuntu 20.04
clone ATOS in your git folder, and make sure that all submodules are present and up to date:
```
git clone https://github.com/RI-SE/ATOS.git
cd ATOS
git submodule update --init --recursive
```

Download prerequisites:
```
sudo apt update && sudo apt install curl gnupg2 lsb-release
```
Authorize the ros2 gpg key with apt:
```sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Add the ROS2 repo to sources list:
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Install ros foxy for desktop and colcon
```
sudo apt update
sudo apt install ros-foxy-desktop python3-colcon-common-extensions ros-foxy-nav-msgs ros-foxy-geographic-msgs ros-foxy-foxglove-msgs
```

source the setup script:
```
source /opt/ros/foxy/setup.bash
```
Add the above line to ~/.bashrc or similar startup script to automate this process.

Create a workspace:
```
mkdir -p ~/atos_ws/src
```

Create symlinks to atos and atos_interfaces
```
ln -s path/to/ATOS ~/atos_ws/src/atos
ln -s path/to/atos-interfaces ~/atos_ws/src/atos_interfaces
```

Change directory into the workspace and build
```
cd ~/atos_ws
colcon build
```

Source the project setup file:
```
source ~/atos_ws/install/setup.bash
```
Also add the above line to ~/.bashrc or similar.

Launch ATOS
```
ros2 launch atos launch_basic.py
```

# <a name="optional-builds--installations"></a> Optional builds & installations
ATOS can be installed in alternative ways, and built with support for various optional modules, described here.

## <a name="relativekinematics"></a> How to build with RelativeKinematics instead of ObjectControl

The server will build the ObjectControl with AbsolutKinematics by default. It's possible to build with RelativeKinematics support by rebuilding with the argument -DWITH_RELATIVE_KINEMATICS=ON, see following command
```sh
colcon build --cmake-args -DWITH_RELATIVE_KINEMATICS=ON
```
To include ObjectControl in the build again run the same command with OFF, as follows
```sh
colcon build --cmake-args -DWITH_RELATIVE_KINEMATICS=OFF
```

