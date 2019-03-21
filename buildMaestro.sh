#!/bin/sh
MAESTRODIR=$(pwd)
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
cd build
make
cd $MAESTRODIR
