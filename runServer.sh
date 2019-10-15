#!/bin/bash

#### User settings
# Modify this array by adding more modules to include them in the execution
MODULES=()

####
# Save top directory
MAESTRODIR=$(pwd)

# Number of modules
MODULE_LENGTH=${#MODULES[@]}

echo "Starting server with $MODULE_LENGTH extra message queues."

# Build string for executing server alongside modules
SERVER_EXEC_STRING="(cd $MAESTRODIR/server/build && ./TEServer -m $MODULE_LENGTH)"
for i in "${MODULES[@]}"
do
	SERVER_EXEC_STRING="$SERVER_EXEC_STRING & (cd $MAESTRODIR/modules/$i/build && ./$i)"
done

# Run the generated string
# Each module in MODULES will be run in parallel with TEServer
eval $SERVER_EXEC_STRING

