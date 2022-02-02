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

# Remove log directory
echo "Removing log directory $SYSTEM_LOG_PATH"
rm ${SYSTEM_LOG_PATH}/*
rmdir ${SYSTEM_LOG_PATH}

# Remove message queue
echo "Removing message queue at $SYSTEM_MQUEUE_PATH"
umount -t mqueue $SYSTEM_MQUEUE_PATH
rm ${SYSTEM_MQUEUE_PATH}/*
rmdir ${SYSTEM_MQUEUE_PATH}

# Remove shared memory
echo "Removing shared memory $SYSTEM_SHARED_MEMORY_PATH"
rm ${SYSTEM_SHARED_MEMORY_PATH}/*
rmdir ${SYSTEM_SHARED_MEMORY_PATH}

# Remove maestro group
echo "Removing group $MAESTRO_GRP"
groupdel $MAESTRO_GRP

# Remove settings directories
echo "Please remove any configuration directories in /home"
exit 0

# TODO figure out a less risky way of removing previous configurations
while true; do
	read -p "Do you wish to remove any user settings in /home? [Y/n]" yn
	case $yn in
		[Yy]*)
			cd /home
			find . -maxdepth 1 -type d \( ! -name . \) -exec bash -c "cd '{}' && echo 'Clearing user settings under $(pwd)' && pwd" \;
			break 2
			;;
		[Nn]*) 
			break 2
			;;
		*) 
			echo "Please answer yes or no.";;
	esac
done

