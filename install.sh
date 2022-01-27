#!/bin/sh

# Find out who is the intended normal user
MAESTRO_GRP="maestro"
NORMAL_USER=$(whoami)
if [ ! -z "$SUDO_USER" ]
then
	NORMAL_USER=$SUDO_USER
fi

USER_HOME=$(getent passwd $NORMAL_USER | cut -d: -f6)
TEST_DIR="$USER_HOME/.maestro"
JOURNAL_DIR="$TEST_DIR/journal"
TRAJ_DIR="$TEST_DIR/traj"
GEOFENCE_DIR="$TEST_DIR/geofence"
OBJECTS_DIR="$TEST_DIR/objects"
SYSTEM_SHARED_MEMORY_PATH="/dev/shm/maestro"
SYSTEM_MQUEUE_PATH="/dev/mqueue/maestro"
SYSTEM_LOG_PATH="/var/log/maestro"

# Create maestro group and add user to it
echo "Creating group $MAESTRO_GRP and adding user $NORMAL_USER to it"
groupadd $MAESTRO_GRP
usermod -a -G $MAESTRO_GRP $NORMAL_USER

# Create settings directories
echo "Creating home directory environment under $TEST_DIR"
mkdir -p $JOURNAL_DIR
mkdir -p $TRAJ_DIR
mkdir -p $GEOFENCE_DIR
mkdir -p $OBJECTS_DIR

cp -r conf $TEST_DIR

chown -R $NORMAL_USER:$MAESTRO_GRP $TEST_DIR
chmod -R g+w $TEST_DIR

# Create shared memory location
echo "Creating shared memory location $SYSTEM_SHARED_MEMORY_PATH"
mkdir -p $SYSTEM_SHARED_MEMORY_PATH
chown -R :$MAESTRO_GRP $SYSTEM_SHARED_MEMORY_PATH
chmod g+w $SYSTEM_SHARED_MEMORY_PATH

# Create message queue location
echo "Creating message queue location $SYSTEM_MQUEUE_PATH"
mkdir -p $SYSTEM_MQUEUE_PATH
chown :$MAESTRO_GRP $SYSTEM_MQUEUE_PATH
chmod g+w $SYSTEM_MQUEUE_PATH
mount -t mqueue none $SYSTEM_MQUEUE_PATH

# Create log directory
echo "Creating log directory location $SYSTEM_LOG_PATH"
mkdir -p $SYSTEM_LOG_PATH
chown :$MAESTRO_GRP $SYSTEM_LOG_PATH
chmod g+w $SYSTEM_LOG_PATH

# TODO should below be in deb file?
# TODO install dependencies?
# TODO install binaries?
# TODO install libraries?



# Reload linkage
echo "Configuring dynamic linker run-time bindings"
ldconfig
systemctl daemon-reload
