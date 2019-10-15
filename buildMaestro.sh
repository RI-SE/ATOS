#!/bin/sh
MAESTRODIR=$(pwd)
git submodule update --init --recursive || exit 1

# Build util
cd util/C
echo "Building util"
cmake -G "Unix Makefiles" . && make || exit 1

# Build core modules
cd $MAESTRODIR/server
mkdir build
cd build
echo "Building core modules"
cmake -G "Unix Makefiles" -DUSE_CITS:BOOL=FALSE -DCMAKE_BUILD_TYPE=Debug .. && make || exit 1

# Build ScenarioControl module
mkdir $MAESTRODIR/modules/ScenarioControl/build
cd $MAESTRODIR/modules/ScenarioControl/build
echo "Building ScenarioControl"
cmake .. && make || exit 1


# Set up running directory in home
cd
echo "Setting up running directory"
if [ ! -d ".maestro" ]; then
	mkdir .maestro
	cd .maestro
	mkdir journal
	mkdir traj
	mkdir conf
	mkdir geofence
	cp -R $MAESTRODIR/server/conf/ .
else
	echo "Running directory already exists, nothing to do"
fi

