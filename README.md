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
cmake -G "Unix Makefiles" ..
```
For debug data add the following: 

```sh
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
```

make the project
```sh
make
```

Start the server
```sh
cd bin
./Core
```

To run one or several of the modules along with Core, modify the runServer.sh script by adding the modules you wish to execute in the variable near the top. Then run the script from the top level directory:
```sh
./runServer.sh
```
To see which modules are available, check the build output inside the ```build/bin``` directory

### Installation
To install the server (recommended) navigate to the build directory and configure the project:
```sh
cd build
cmake -G "Unix Makefiles" ..
```
then build and install the server (be aware that this requires superuser privileges)
```sh
make install
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

The server will build ObjectControl module by default. It's possible to disable the ObjectControl module by remake the `cmake` procedure with the argument -DDISABLE_OBJECT_CONTROL=YES, see following command
```sh
cmake .. -DDISABLE_OBJECT_CONTROL=YES
```
If the argument -DDISABLE_OBJECT_CONTROL=YES has been used it's necessary enable the ObjectControl module again, see following command
```sh
cmake .. -DDISABLE_OBJECT_CONTROL=NO
```