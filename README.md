# Maestro 
<img align="left" width="100" height="100" src="/doc/MaestroServer.svg">

The Maestro server is a communication hub for all test objects. The server monitors and controls the test objects and is also responsible for creating logfiles.

<br />
<br />

To build Maestro follow the guide below.


## How to build and run the server

Install necessary development packages.

**Ubuntu**
```sh
sudo apt-get install libsystemd-dev
``` 

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
