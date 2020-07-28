# Maestro 
<img align="left" width="100" height="100" src="https://github.com/RI-SE/Maestro/blob/dev/server/src/icon/MaestroICON.svg">

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

# poti
Make sure that the system has the correct locale, such that float numbers are defined with '.'
If not the right locale, do
```sh 
sudo update-locale LC_NUMERIC="en_GB.UTF-8"
```
Proceed to build the application
```sh
cd poti
mkdir build && cd build
gcc -I../inc/ ../src/nmea2etsi.c ../src/poti_byte.c -o poti
```
Now you are able to run the program through
```sh
./poti
```

# Compile and run RTKLIB
```sh
cd rtklib/app
chmod 755 makeall.sh
./makeall.sh
cp chronos/rtklib_conf/original/* rtklib/app/rtkrcv/gcc/
```
Attach a GNSS receiver to one of the USB ports of the computer
Check which serial port it gets (e.g. ttyUSB0 or ttyACM3) using dmesg command
Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr1-path
If SP's base station is used:   inpstr2-path       =hener:rcb1l@www.igs-ip.net:80/SPT00:
                                ant2-postype       =xyz        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
				  ant2-pos1          =3328984.527 # (deg|m)
				  ant2-pos2          =761910.265  # (deg|m)
				  ant2-pos3          =5369033.689 # (m|m)
 If Asta's base station is used, connect modem via serial cable to usb port, check assigned serial port using dmesg command
 Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr2-path
				  inpstr2-path       =ttyUSB0:9600:8:n:1  
                                 ant2-postype       =rtcm        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
				  #ant2-pos1         =3328984.527 # (deg|m)
				  #ant2-pos2         =761910.265  # (deg|m)
				  #ant2-pos3         =5369033.689 # (m|m)
```sh
cd rtklib/app/rtkrcv/gcc/
./rtkrcv -o rover_ubx_m8t.conf -s
```
Useful commands in rtkrcv (0.1 is the update frequency) all commands are terminated by ctrl-c:
status 0.1       to check solution status
satellite 0.1    to check satellite data
stream 0.1       to check data streams
shutdown         to exit rtkrcv

# Compile and run RTKLIB Explorer
Make sure user is in dialout group (need to logout/login to make effect): sudo adduser [user] dialout
```sh
cd rtklibexplorer/app
make
cp chronos/rtklib_conf/explorer/* rtklibexplorer/app/rtkrcv/gcc/
```
Attach a GNSS receiver to one of the USB ports of the computer
Check which serial port it gets (e.g. ttyUSB0 or ttyACM3) using dmesg command
Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr1-path
If SP's base station is used:   inpstr2-path       =hener:rcb1l@www.igs-ip.net:80/SPT00:
                                 ant2-postype       =xyz        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
				  ant2-pos1          =3328984.527 # (deg|m)
				  ant2-pos2          =761910.265  # (deg|m)
				  ant2-pos3          =5369033.689 # (m|m)
If Asta's base station is used, connect modem via serial cable to usb port, check assigned serial port using dmesg command
Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr2-path
				  inpstr2-path       =ttyUSB0:9600:8:n:1  
                                 ant2-postype       =rtcm        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
				  #ant2-pos1         =3328984.527 # (deg|m)
				  #ant2-pos2         =761910.265  # (deg|m)
				  #ant2-pos3         =5369033.689 # (m|m)
```sh
cd rtklibexplorer/app/rtkrcv/gcc/
./rtkrcv -o rover_ubx_m8t.conf -s
```
Useful commands in rtkrcv (0.1 is the update frequency) all commands are terminated by ctrl-c:
 status 0.1       to check solution status
 satellite 0.1    to check satellite data
 stream 0.1       to check data streams
 shutdown         to exit rtkrcv
