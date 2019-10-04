#!/bin/sh
MAESTRODIR=$(pwd)
git submodule update --init --recursive
cd util/C
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

# Build separate modules
mkdir $MAESTRODIR/modules/ScenarioControl/build
cd $MAESTRODIR/modules/ScenarioControl/build
cmake ..
make
