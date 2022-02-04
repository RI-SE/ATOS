#!/bin/sh

# Find out who is the intended normal user
MAESTRO_GRP="maestro"

SYSTEM_SHARED_MEMORY_PATH="/dev/shm"
SYSTEM_MQUEUE_PATH="/dev/mqueue"
SYSTEM_LOG_PATH="/var/log/maestro"

# Remove message queue
echo "Removing message queue at $SYSTEM_MQUEUE_PATH"
rm -r ${SYSTEM_MQUEUE_PATH}

# Remove shared memory
echo "Removing shared memory $SYSTEM_SHARED_MEMORY_PATH"
rm -r ${SYSTEM_SHARED_MEMORY_PATH}


