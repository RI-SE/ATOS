# Maestro 
<img align="left" width="100" height="100" src="/doc/MaestroServer.svg">

The Maestro server is a communication hub for all test objects. The server monitors and controls the test objects and is also responsible for creating logfiles. To build Maestro follow the guide below.

<br />
<br />


# Table of contents
- [Bulding Maestro with colcon](#maestro)
  - [Dependencies & external libraries](#dependencies)
    - [Installing OpenSimulationInterface v3.4.0](#osi)
    - [Installing maestro-interfaces](#maestro-interfaces)
    - [Installing ad-xolib](#ad-xolib)
  - [Installing ROS2 and building for the first time with colcon](#ros2)
    - [Ubuntu 20.04](#ubuntu-20.04)
- [Other builds & installations](#other-installations)
   - [How to build and run the server](#build-run-server)
     - [Installation](#server-installation)
     - [Installation via dpkg](#installation-dpkg)
   - [Troubleshooting](#troubleshooting)
   - [Building the server with CITS module and mqtt](#build-cits-mqtt)
   - [How to build with RelativeKinematics instead of ObjectControl](#relativekinematics)

# <a name="maestro"></a> Building Maestro with colcon
Below are the steps for building Maestro for the first time with colcon.

## <a name="dependencies"></a> Dependencies & external libraries
In order to build Maestro, dependencies and exernal libraries need to be installed. First install the necessary development packages:
```
sudo apt install libsystemd-dev libprotobuf-dev protobuf-compiler libeigen3-dev
```

Then, the following external libraries need to be installed:
- [OpenSimulationInterface v3.4.0](https://github.com/OpenSimulationInterface/open-simulation-interface)
- [maestro-interfaces](https://github.com/RI-SE/maestro-interfaces)
- [ad-xolib](https://github.com/javedulu/ad-xolib)

### <a name="osi"></a> Installing OpenSimulationInterface v3.4.0
```
git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0
cd open-simulation-interface
mkdir build
cd build
cmake ..
make
sudo make install
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
mkdir build
cd build
cmake .. -DBUILD_EMBED_TARGETS=OFF
make
sudo make install
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

Make sure that the linker knows where OpenSimulationInterface is located:
```
echo /usr/local/lib/osi3 > /etc/ld.so.conf.d/osi3.conf
sudo ldconfig
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

# <a name="other-installations"></a> Other builds & installations
Listed below are other types of builds and installations.

## <a name="build-run-server"></a> How to build and run the server

Prerequisites: C/C++ compiler, CMake (minimum version 3.10.2)

##### Ubuntu

Make sure to have an updated package index (apt update).

##### Dependencies (required)

Install necessary development packages:

```sh
sudo apt install libsystemd-dev libprotobuf-dev protobuf-compiler libeigen3-dev
```

Install OpenSimulationInterface v3.4.0 (see [https://github.com/OpenSimulationInterface](https://github.com/OpenSimulationInterface/open-simulation-interface#installation)):

```sh
git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git -b v3.4.0
cd open-simulation-interface
mkdir build
cd build
cmake ..
make
sudo make install
```

##### Dependencies (optional)
Install SWIG:

See https://github.com/RI-SE/iso22133#readme


##### Build and run the server

Clone the repo and make sure you run the following command to update all submodules:

```sh
git submodule update --init --recursive
```

Navigate to the the repo and enter the build directory 

```sh
mkdir build && cd build
```
create project
```sh
cmake ..
```

make the project and install (requires superuser privileges). This will create required directories for logs, configuration files etc.:
```sh
make && sudo make install
```

Start the server
```sh
bin/Core
```

To get debug printouts, add the verbose option when running:
```
bin/Core -v
```

To run one or several of the modules along with Core, either run them in a separate terminal after starting Core (with the required number of additional message queue slots with `-m`):
```
# Core binary
bin/Core -m 2
# Module binaries in new terminals
bin/RelativeKinematics
bin/Visualization
```

or, modify the runServer.sh script by adding the modules you wish to execute in the variable near the top. Then run the script from the top level directory:
```sh
./runServer.sh
```
To see which modules are available, check the build output inside the ```build/bin``` directory

### <a name="server-installation"></a> Installation
To install the server (recommended) navigate to the build directory and configure the project:
```sh
cd build
cmake ..
```
then build and install the server (be aware that this requires superuser privileges)
```sh
sudo make install
```

### <a name="installation-dpkg"></a> Installation via dpkg
First install dependencies
```sh
sudo apt install libsystemd-dev libprotobuf-dev libeigen3-dev
```
then navigate to the .deb file and install it
```sh
sudo dpkg -i Maestro-x.x.x-Linux.deb
```
on first install, it is necessary to reboot to reload groups

## <a name="troubleshooting"></a> Troubleshooting
The command
```sh
groups
```
should show the current user belonging to the maestro group, and
```sh
mount -l | grep -E "(shm|mqueue)"
```
should show two mount points on /dev/shm and /dev/mqueue. The directory
```sh
ls -lad /var/log/maestro
```
should be owned by the maestro group.

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

The server will not build the CITS module by default. This is to prevent the use of the CITS module when it is not necessary. To enable building of the module, run `cmake` from the `build/` directory
```sh
cmake "Unix Makefiles" -DUSE_CITS:BOOL=TRUE ..
```
then you can build and run the server as normal
```sh
make && cd bin
./Core
```

To disable the CITS module, remake the `cmake` procedure
```sh
cmake "Unix Makefiles" -DUSE_CITS:BOOL=FALSE ..
```

## <a name="relativekinematics"></a> How to build with RelativeKinematics instead of ObjectControl

The server will build ObjectControl thread in Core by default. It's possible to replace ObjectControl with the RelativeKinematics module remaking the `cmake` procedure with the argument -DWITH_RELATIVE_KINEMATICS=ON, see following command
```sh
cmake .. -DWITH_RELATIVE_KINEMATICS=ON
```
To include ObjectControl in the build again run the same command with OFF, see following command
```sh
cmake .. -DWITH_RELATIVE_KINEMATICS=OFF
```

