#!/usr/bin/env bash
set -euo pipefail

# Canonical build profile for ATOSFleetManagement.
# Default workspace: ~/atos_ws
# Usage:
#   ./scripts/build_atosfleetmanagement.sh
#   ./scripts/build_atosfleetmanagement.sh /path/to/ws
#   ATOS_WS=/path/to/ws ./scripts/build_atosfleetmanagement.sh --event-handlers console_direct+

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'HELP'
ATOSFleetManagement build script

Usage:
  scripts/build_atosfleetmanagement.sh [workspace] [extra colcon args...]

Examples:
  scripts/build_atosfleetmanagement.sh
  scripts/build_atosfleetmanagement.sh /home/user/atos_ws
  scripts/build_atosfleetmanagement.sh --event-handlers console_direct+
  scripts/build_atosfleetmanagement.sh /home/user/atos_ws --executor sequential

Behavior:
  - Builds packages: atos, atos_gui
  - Enables TruckObjectControl module
  - Disables legacy ATOS modules not used in ATOSFleetManagement
HELP
  exit 0
fi

DEFAULT_WS="${ATOS_WS:-$HOME/atos_ws}"
WS="$DEFAULT_WS"

# If first arg is not an option, treat it as workspace path.
if [[ $# -gt 0 && "${1:0:1}" != "-" ]]; then
  WS="$1"
  shift
fi

if [[ ! -d "$WS" ]]; then
  echo "Workspace does not exist: $WS"
  echo "Create it first or pass a valid workspace path."
  exit 1
fi

if [[ ! -d "$WS/src" ]]; then
  echo "Workspace is missing src directory: $WS/src"
  exit 1
fi

echo "Building ATOSFleetManagement in workspace: $WS"

cd "$WS"
colcon build --packages-select atos atos_gui --symlink-install --cmake-args \
  -DWITH_TRUCK_OBJECT_CONTROL=ON \
  -DWITH_OBJECT_CONTROL=OFF \
  -DWITH_OPEN_SCENARIO_GATEWAY=OFF \
  -DWITH_JOURNAL_CONTROL=OFF \
  -DWITH_ESMINI_ADAPTER=OFF \
  -DWITH_DIRECT_CONTROL=OFF \
  -DWITH_TRAJECTORYLET_STREAMER=OFF \
  -DWITH_OSI_ADAPTER=OFF \
  -DWITH_MQTT_BRIDGE=OFF \
  -DWITH_POINTCLOUD_PUBLISHER=OFF \
  -DWITH_INTEGRATION_TESTING=OFF \
  -DWITH_BACK_TO_START=OFF \
  -DWITH_REST_BRIDGE=OFF \
  -DWITH_MONR_RELAY=OFF \
  "$@"
