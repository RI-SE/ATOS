#!/bin/sh
mkdir ./build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
make
cp -R ../conf/ .
mkdir traj
cp ../traj/0.traj ./traj/127.0.0.1
