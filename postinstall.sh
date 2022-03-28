#!/bin/sh

# Find out who is the intended normal user
MAESTRO_GRP="maestro"
MAESTRO_USER="maestro"
NORMAL_USER=$(whoami)
TIMECONTROL_PROGRAM=$(dpkg-divert --truename /usr/bin/Core)

if [ ! -z "$SUDO_USER" ]
then
	NORMAL_USER=$SUDO_USER
fi

SYSTEM_SHARED_MEMORY_PATH="/dev/shm"
SYSTEM_MQUEUE_PATH="/dev/mqueue"
SYSTEM_LOG_PATH="/var/log/maestro"

# Create maestro group and add user to it
echo "Creating group $MAESTRO_GRP and adding user $NORMAL_USER to it"
addgroup --quiet --system $MAESTRO_GRP
adduser --quiet $NORMAL_USER $MAESTRO_GRP

# Create shared memory location
echo "Creating shared memory location $SYSTEM_SHARED_MEMORY_PATH"
mkdir -p $SYSTEM_SHARED_MEMORY_PATH
chown -R :$MAESTRO_GRP $SYSTEM_SHARED_MEMORY_PATH
chmod g+w $SYSTEM_SHARED_MEMORY_PATH

# Create message queue location
if grep -qs "$SYSTEM_MQUEUE_PATH" /proc/mounts
then
	echo "Found existing message queue location $SYSTEM_MQUEUE_PATH"
	if [ $(stat -c "%a" "$SYSTEM_MQUEUE_PATH") != "1777" ]
	then
		chmod 1777 $SYSTEM_MQUEUE_PATH	
	fi
else
	echo "Creating message queue location $SYSTEM_MQUEUE_PATH"
	mkdir -p $SYSTEM_MQUEUE_PATH
	chmod 1777 $SYSTEM_MQUEUE_PATH
	mount -t mqueue none $SYSTEM_MQUEUE_PATH
fi

# Create log directory
echo "Creating log directory location $SYSTEM_LOG_PATH"
mkdir -p $SYSTEM_LOG_PATH
chown :$MAESTRO_GRP $SYSTEM_LOG_PATH
chmod 2775 $SYSTEM_LOG_PATH

# Give time control program access to set time
if [ $(command -v setcap) ]
then
	echo "Giving $TIMECONTROL_PROGRAM access to set system time"
	setcap cap_sys_time=eip $TIMECONTROL_PROGRAM
else
	echo "Unable to find command setcap - maestro time sync will not function properly"
fi

# Reload linkage
echo "Configuring dynamic linker run-time bindings"
ldconfig
if [ $(command -v systemctl) ]
then
	systemctl daemon-reload
else
	echo "Unable to find command systemctl - manual reload will be necessary to run daemonized maestro"
fi
