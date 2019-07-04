# Chronos test server

## How to build and run the server

Navigate to the the repo and enter the build folder 

```sh
cd  server
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
]
```sh
make
cp -R ../conf/ .
```

Create a folder for Trajectory files in /build and move one of the existing trajectory files to this folder. 
```sh
mkdir traj
cp ../traj/0.traj ./traj/192.168.0.1
```

Start the server
```sh
./TEServer
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
sudo make html
```
This will build the documentation for all the code. Then proceede to build and install paho
```sh
make
make install
```

The server will not bu default build the CITS module. This is to prevent the use of the CITS module when it is not necessary. To enable building of the module, run `cmake` from the `build/` directory
```sh
cmake "Unix Makefiles" -DUSE_CITS:BOOL=TRUE ..
```
then you can build and run the server as normal
```sh
make
./TEServer
```

To disable the CITS module, remake the `cmake` procedure

```sh
cmake "Unix Makefiles" -DUSE_CITS:BOOL=FALSE ..
```

# To communicate with server start program.
./UserControl [IP] [port]

## How to build and run gui server
cd server_gui
mkdir build & cd build
"/home/kj/Qt/5.7/gcc_64/bin/qmake" ../server_gui.pro -spec linux-g++ (if debug add CONFIG+=debug CONFIG+=qml_debug; if windows -spec win32-g++)
make
./server_gui

## Visualization adaption
mkdir build_websocket_visualization && cd build_websocket_visualization
"/home/kj/Qt/5.7/gcc_64/bin/qmake" ../websocket_visualization/websocket_visualization.pro -spec linux-g++ 
("/opt/Qt5.7.1/5.7/gcc_64/bin/qmake"  ../websocket_visualization.pro -spec linux-g++)
make
./websocket_visualization 53251 1 ./../data/ 100

## How to start objects
./object 57.12345 12.54321 123.43 99999 99998

# CentOS installation for server
yum install cmake
yum groupinstall 'Development Tools'
wget http://download.qt.io/official_releases/qt/5.7/5.7.1/qt-opensource-linux-x64-5.7.1.run
./http://download.qt.io/official_releases/qt/5.7/5.7.1/qt-opensource-linux-x64-5.7.1.run

# Install on raspberry
sudo apt-get install qt-sdk
sudo apt install libqt5websockets5_dev
qmake -qt=5 ../websocket_visualization.pro -spec linux-g++

# poti
# Make sure that the system has the correct locale, such that float numbers are defined with '.'
# If not the right locale, do 
sudo update-locale LC_NUMERIC="en_GB.UTF-8"
# Make sure to log out, then perform
cd poti
mkdir build && cd build
gcc -I../inc/ ../src/nmea2etsi.c ../src/poti_byte.c -o poti
# Run the program
./poti

# Eclipse project
cd /home/kj/Repos/chronos
mkdir buildEclipse
cd buildEclipse
cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../server/
Import project using Menu File->Import
Select General->Existing projects into workspace:
Browse root directory /home/kj/Repos/chronos/build_server_eclipse

# KDevelop ubuntu
wget -O KDevelop.AppImage https://download.kde.org/stable/kdevelop/5.1.0/bin/linux/KDevelop-5.1.0-x86_64.AppImage
chmod +x KDevelop.AppImage
./KDevelop.AppImage
Project -> Open/import cmake
Build main, usercontrol & VisualizationAdapter
Run -> Configure Launches

# Compile and run RTKLIB
cd rtklib/app
chmod 755 makeall.sh
./makeall.sh
cp chronos/rtklib_conf/original/* rtklib/app/rtkrcv/gcc/
# Attach a GNSS receiver to one of the USB ports of the computer
# Check which serial port it gets (e.g. ttyUSB0 or ttyACM3) using dmesg command
# Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr1-path
# If SP's base station is used:   inpstr2-path       =hener:rcb1l@www.igs-ip.net:80/SPT00:
#                                 ant2-postype       =xyz        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
#				  ant2-pos1          =3328984.527 # (deg|m)
#				  ant2-pos2          =761910.265  # (deg|m)
#				  ant2-pos3          =5369033.689 # (m|m)
# If Asta's base station is used, connect modem via serial cable to usb port, check assigned serial port using dmesg command
# Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr2-path
#				  inpstr2-path       =ttyUSB0:9600:8:n:1  
#                                 ant2-postype       =rtcm        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
#				  #ant2-pos1         =3328984.527 # (deg|m)
#				  #ant2-pos2         =761910.265  # (deg|m)
#				  #ant2-pos3         =5369033.689 # (m|m)
cd rtklib/app/rtkrcv/gcc/
./rtkrcv -o rover_ubx_m8t.conf -s
# Useful commands in rtkrcv (0.1 is the update frequency) all commands are terminated by ctrl-c:
# status 0.1       to check solution status
# satellite 0.1    to check satellite data
# stream 0.1       to check data streams
# shutdown         to exit rtkrcv

# Compile and run RTKLIB Explorer
# Make sure user is in dialout group (need to logout/login to make effect): sudo adduser [user] dialout
cd rtklibexplorer/app
make
cp chronos/rtklib_conf/explorer/* rtklibexplorer/app/rtkrcv/gcc/
# Attach a GNSS receiver to one of the USB ports of the computer
# Check which serial port it gets (e.g. ttyUSB0 or ttyACM3) using dmesg command
# Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr1-path
# If SP's base station is used:   inpstr2-path       =hener:rcb1l@www.igs-ip.net:80/SPT00:
#                                 ant2-postype       =xyz        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
#				  ant2-pos1          =3328984.527 # (deg|m)
#				  ant2-pos2          =761910.265  # (deg|m)
#				  ant2-pos3          =5369033.689 # (m|m)
# If Asta's base station is used, connect modem via serial cable to usb port, check assigned serial port using dmesg command
# Modify the corresponding config file (skytraq or ubx) to reflect the assigned port in inpstr2-path
#				  inpstr2-path       =ttyUSB0:9600:8:n:1  
#                                 ant2-postype       =rtcm        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
#				  #ant2-pos1         =3328984.527 # (deg|m)
#				  #ant2-pos2         =761910.265  # (deg|m)
#				  #ant2-pos3         =5369033.689 # (m|m)
cd rtklibexplorer/app/rtkrcv/gcc/
./rtkrcv -o rover_ubx_m8t.conf -s
# Useful commands in rtkrcv (0.1 is the update frequency) all commands are terminated by ctrl-c:
# status 0.1       to check solution status
# satellite 0.1    to check satellite data
# stream 0.1       to check data streams
# shutdown         to exit rtkrcv
