#!/bin/sh
MAESTRODIR=$(pwd)

apt-get install -y build-essential gcc make cmake cmake-gui cmake-curses-gui
apt-get install -y fakeroot fakeroot devscripts dh-make lsb-release libssl-dev

git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
make
make install
cd $MAESTRODIR
git submodule update --init --recursive
cd $MAESTRODIR/util/C
cmake -G "Unix Makefiles" .
make
cd $MAESTRODIR/server
mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
cp -R ../conf/ .
mkdir traj && mkdir log
cp ../traj/0.traj ./traj/127.0.0.1
make
cd $MAESTRODIR
