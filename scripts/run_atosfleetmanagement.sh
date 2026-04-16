#!/usr/bin/env bash

set -euo pipefail

INSECURE="${INSECURE:-True}"
WITH_TRUCK_SIMULATOR="${WITH_TRUCK_SIMULATOR:-False}"
FOXBRIDGE="${FOXBRIDGE:-True}"

source /root/atos_ws/install/setup.sh

echo "Starting ATOSFleetManagement with:"
echo "  INSECURE=${INSECURE}"
echo "  WITH_TRUCK_SIMULATOR=${WITH_TRUCK_SIMULATOR}"
echo "  FOXBRIDGE=${FOXBRIDGE}"

exec ros2 launch atos launch_atosfleetmanagement.py \
  insecure:="${INSECURE}" \
  with_truck_simulator:="${WITH_TRUCK_SIMULATOR}" \
  foxbridge:="${FOXBRIDGE}"
