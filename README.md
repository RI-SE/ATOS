# cproto

## How to build and run server

cd  server

mkdir build && cd build

## Create project
cmake -G "Unix Makefiles" ..

## Create project with debug data
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ..


make
./TEServer

## How to build and run gui server
cd server_gui
mkdir build & cd build
"/home/kj/Qt/5.7/gcc_64/bin/qmake" ../server_gui.pro -spec linux-g++ (if debug add CONFIG+=debug CONFIG+=qml_debug; if windows -spec win32-g++)
make
./server_gui

## Visualization adaption
cd websocket_visualization
mkdir build && cd build
"/home/kj/Qt/5.7/gcc_64/bin/qmake" ../websocket_visualization.pro -spec linux-g++ 
("/opt/Qt5.7.1/5.7/gcc_64/bin/qmake"  ../websocket_visualization.pro -spec linux-g++)
make
./websocket_visualization 53251 1 ./../data/ 100

## How to start objects
./object 57.12345 12.54321 123.43 99999 99998

# CentOS installation for server
yum install cmake
yum groupinstall 'Development Tools'


# CentOS installation qt
wget http://download.qt.io/official_releases/qt/5.7/5.7.1/qt-opensource-linux-x64-5.7.1.run
./http://download.qt.io/official_releases/qt/5.7/5.7.1/qt-opensource-linux-x64-5.7.1.run

# POTI
cd poti
mkdir build && cd build
gcc -I../inc/ ../src/nmea2etsi.c ../src/object_rpi.c -o object_rpi
./object_rpi 
