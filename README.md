# Maestro 
<img align="left" width="100" height="100" src="/doc/MaestroServer.svg">

The Maestro server is a communication hub for all test objects. The server monitors and controls the test objects and is also responsible for creating logfiles. To build Maestro follow the guide below.

<br />
<br />


# Table of contents
- [Maestro](#maestro)
- [Table of contents](#table-of-contents)
- [ Building Maestro with colcon](#-building-maestro-with-colcon)
  - [ Dependencies \& external libraries](#-dependencies--external-libraries)
    - [ Installing OpenSimulationInterface v3.4.0](#-installing-opensimulationinterface-v340)
    - [ Installing maestro-interfaces](#-installing-maestro-interfaces)
    - [ Installing ad-xolib](#-installing-ad-xolib)
    - [ Installing esmini](#-installing-esmini)
  - [ Installing ROS2 and building for the first time with colcon](#-installing-ros2-and-building-for-the-first-time-with-colcon)
    - [ Ubuntu 20.04](#-ubuntu-2004)
- [ Optional builds \& installations](#-optional-builds--installations)
    - [ Installation via dpkg](#-installation-via-dpkg)
  - [ Building the server with CITS module and mqtt](#-building-the-server-with-cits-module-and-mqtt)
  - [ How to build with RelativeKinematics instead of ObjectControl](#-how-to-build-with-relativekinematics-instead-of-objectcontrol)

# <a name="maestro"></a> Building Maestro with colcon
Below are the steps for building Maestro for the first time with colcon.

Prerequisites: C/C++ compiler, CMake (minimum version 3.10.2)

## <a name="dependencies"></a> Dependencies & external libraries
In order to build Maestro, dependencies and exernal libraries need to be installed. First install the necessary development packages:
```
sudo apt install libsystemd-dev libprotobuf-dev protobuf-compiler libeigen3-dev
```

Then, the following external libraries need to be installed:
- [OpenSimulationInterface v3.4.0](https://github.com/OpenSimulationInterface/open-simulation-interface)
- [maestro-interfaces](https://github.com/RI-SE/maestro-interfaces)
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
echo /usr/local/lib/osi3 > /etc/ld.so.conf.d/osi3.conf
sudo ldconfig
```

### <a name="maestro-interfaces"></a> Installing maestro-interfaces
```
git clone https://github.com/RI-SE/maestro-interfaces
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
clone Maestro in your git folder, and make sure that all submodules are present and up to date:
```
git clone https://github.com/RI-SE/Maestro.git
cd Maestro
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
sudo apt install ros-foxy-desktop python3-colcon-common-extensions ros-foxy-nav-msgs
```

source the setup script:
```
source /opt/ros/foxy/setup.bash
```
Add the above line to ~/.bashrc or similar startup script to automate this process.

Create a workspace:
```
mkdir -p ~/maestro_ws/src
```

Create symlinks to maestro and maestro_interfaces
```
ln -s path/to/Maestro ~/maestro_ws/src/maestro
ln -s path/to/maestro-interfaces ~/maestro_ws/src/maestro_interfaces
```

Change directory into the workspace and build
```
cd ~/maestro_ws
colcon build
```

Source the project setup file:
```
source ~/maestro_ws/install/setup.bash
```
Also add the above line to ~/.bashrc or similar.

Launch Maestro
```
ros2 launch maestro maestro_launch.py
```

# <a name="optional-builds--installations"></a> Optional builds & installations
Maestro can be installed in alternative ways, and built with support for various optional modules, described here.

### <a name="installation-dpkg"></a> Installation via dpkg
Navigate to the .deb file and install it
```sh
sudo dpkg -i Maestro-x.x.x-Linux.deb
```
on first install, it is necessary to reboot to reload groups

## <a name="build-cits-mqtt"></a> Building the server with CITS module and mqtt

The CITS module uses PAHO MQTT, which can be found through the following link:
https://www.eclipse.org/paho/

To be able to run the server with the CITS module you must first build and install paho mqtt. 

Paho mqtt requires OpenSSL to be able to run. To install OpenSSL do
```sh
apt-get install libssl-dev
```
In order to get and build the documentation for paho mqtt, do the following
```sh
apt-get install doxygen graphviz
```

Now get the latest source code for paho mqtt
```sh
git clone https://github.com/eclipse/paho.mqtt.c.git
```

Go to the root of the cloned git repo and build the documentation by doing
```sh
cd paho.mqtt.c.git
make html
```
This will build the documentation for all the code. Then proceede to build and install paho
```sh
make
sudo make install
```

The server will not build the CITS module by default. This is to prevent the use of the CITS module when it is not necessary. To enable building of the module issue the following command
```sh
colcon build --cmake-args -DUSE_CITS:BOOL=TRUE ..
```
then you can build and run the server as normal

To disable the CITS module, rebuild as follows
```sh
colcon build --cmake-args -DUSE_CITS:BOOL=FALSE ..
```

## <a name="relativekinematics"></a> How to build with RelativeKinematics instead of ObjectControl

The server will build the ObjectControl with AbsolutKinematics by default. It's possible to build with RelativeKinematics support by rebuilding with the argument -DWITH_RELATIVE_KINEMATICS=ON, see following command
```sh
colcon build --cmake-args -DWITH_RELATIVE_KINEMATICS=ON
```
To include ObjectControl in the build again run the same command with OFF, as follows
```sh
colcon build --cmake-args -DWITH_RELATIVE_KINEMATICS=OFF
```

