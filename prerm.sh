#!/bin/sh
SYSTEM_MQUEUE_PATH="/dev/mqueue"

# Remove message queue
echo "Unmounting message queue at $SYSTEM_MQUEUE_PATH"
umount -t mqueue $SYSTEM_MQUEUE_PATH


