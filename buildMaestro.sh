#!/bin/sh
sudo git clone https://github.com/RI-SE/Maestro.git
cd Maestro
sudo git checkout $1
cd server
mkdir ./build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
make
cp -R ../conf/ .
mkdir traj
cp ../traj/0.traj ./traj/127.0.0.1
