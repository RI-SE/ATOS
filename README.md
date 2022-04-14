# Maestro 
<img align="left" width="100" height="100" src="/doc/MaestroServer.svg">

The Maestro server is a communication hub for all test objects. The server monitors and controls the test objects and is also responsible for creating logfiles.

<br />
<br />


To build Maestro follow the guide below.


## How to build and run the server

Prerequisites: C/C++ compiler, CMake (minimum version 3.10.2)

**Ubuntu**

Make sure to have an updated package index (apt update).

##### Dependencies (required)

Install necessary development packages:

```sh
sudo apt install libsystemd-dev libprotobuf-dev protobuf-compiler libeigen3-dev
```

Install OpenSimulationInterface (see [https://github.com/OpenSimulationInterface](https://github.com/OpenSimulationInterface/open-simulation-interface#installation)):

```sh
git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git
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

### Installation
To install the server (recommended) navigate to the build directory and configure the project:
```sh
cd build
cmake ..
```
then build and install the server (be aware that this requires superuser privileges)
```sh
sudo make install
```

### Installation via dpkg
First install dependencies
```sh
sudo apt install libsystemd-dev libprotobuf-dev libeigen3-dev
```
then navigate to the .deb file and install it
```sh
sudo dpkg -i Maestro-x.x.x-Linux.deb
```
on first install, it is necessary to reboot to reload groups

## Troubleshooting
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

## Building the server with CITS module and mqtt

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

## How to build with RelativeKinematics instead of ObjectControl

The server will build ObjectControl thread in Core by default. It's possible to replace ObjectControl with the RelativeKinematics module remaking the `cmake` procedure with the argument -DWITH_RELATIVE_KINEMATICS=ON, see following command
```sh
cmake .. -DWITH_RELATIVE_KINEMATICS=ON
```
To include ObjectControl in the build again run the same command with OFF, see following command
```sh
cmake .. -DWITH_RELATIVE_KINEMATICS=OFF
```

## Installing ROS2 and building for the first time with colcon
### Ubuntu
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
sudo apt install ros-foxy-desktop
sudo apt install python3-colcon-common-extensions
```

clone maestro-interfaces from github to your git folder:
```
git clone https://github.com/RI-SE/maestro-interfaces
```

Install opensimulation interface as above, and then make sure that the linker knows where it is located:
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
mkdir -p ~/dev_ws/src
```
Create symlinks to maestro and maestro_interfaces

```
ln -s path/to/Maestro ~/dev_ws/src/maestro
ln -s path/to/maestro-interfaces ~/dev_ws/src/maestro_interfaces
```

Change directory into the workspace and build
```
cd ~/dev_ws
colcon build
```

Source the project setup file:
```
source ~/dev_ws/install/setup.bash
```
Also add this line to ~/.bashrc or similar.

Launch Maestro
```
ros2 launch maestro maestro_launch.py
```
