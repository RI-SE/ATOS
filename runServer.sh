#!/bin/bash

#### User settings
# Modify this array by adding more modules to include them in the execution
MODULES=(ScenarioControl)






####
# Save top directory
MAESTRODIR=$(pwd)

# Build string for executing server alongside modules
SERVER_EXEC_STRING="(cd $MAESTRODIR/server/build && ./Maestro)"
for i in "${MODULES[@]}"
do
	SERVER_EXEC_STRING="$SERVER_EXEC_STRING & (cd $MAESTRODIR/modules/$i/build && ./$i)"
done

# Run the generated string
# Each module in MODULES will be run in parallel with TEServer
eval $SERVER_EXEC_STRING

