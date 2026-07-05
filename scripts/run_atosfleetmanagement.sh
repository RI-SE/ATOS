#!/usr/bin/env bash

set -euo pipefail

INSECURE="${INSECURE:-True}"
WITH_TRUCK_SIMULATOR="${WITH_TRUCK_SIMULATOR:-False}"
FOXBRIDGE="${FOXBRIDGE:-True}"
COT_TLS_REQUIRE_CLIENT_CERT="${COT_TLS_REQUIRE_CLIENT_CERT:-False}"
COT_TLS_CERT_PATH="${COT_TLS_CERT_PATH:-}"
COT_TLS_KEY_PATH="${COT_TLS_KEY_PATH:-}"
COT_TLS_CA_PATH="${COT_TLS_CA_PATH:-}"
SIMULATOR_1_LATERAL_OFFSET_M="${SIMULATOR_1_LATERAL_OFFSET_M:-0.0}"
SIMULATOR_2_LATERAL_OFFSET_M="${SIMULATOR_2_LATERAL_OFFSET_M:-0.0}"
SIMULATOR_3_LATERAL_OFFSET_M="${SIMULATOR_3_LATERAL_OFFSET_M:-0.0}"

# setup.sh from colcon references vars that may be unset; avoid nounset during sourcing.
set +u
source /root/.local/share/atos/venv/bin/activate
source /root/atos_ws/install/setup.sh
set -u

echo "Starting ATOSFleetManagement with:"
echo "  INSECURE=${INSECURE}"
echo "  WITH_TRUCK_SIMULATOR=${WITH_TRUCK_SIMULATOR}"
echo "  FOXBRIDGE=${FOXBRIDGE}"
echo "  COT_TLS_REQUIRE_CLIENT_CERT=${COT_TLS_REQUIRE_CLIENT_CERT}"
echo "  COT_TLS_CERT_PATH=${COT_TLS_CERT_PATH}"
echo "  COT_TLS_KEY_PATH=${COT_TLS_KEY_PATH}"
echo "  COT_TLS_CA_PATH=${COT_TLS_CA_PATH}"
echo "  SIMULATOR_1_LATERAL_OFFSET_M=${SIMULATOR_1_LATERAL_OFFSET_M}"
echo "  SIMULATOR_2_LATERAL_OFFSET_M=${SIMULATOR_2_LATERAL_OFFSET_M}"
echo "  SIMULATOR_3_LATERAL_OFFSET_M=${SIMULATOR_3_LATERAL_OFFSET_M}"

launch_args=(
  "insecure:=${INSECURE}"
  "with_truck_simulator:=${WITH_TRUCK_SIMULATOR}"
  "foxbridge:=${FOXBRIDGE}"
  "cot_tls_require_client_cert:=${COT_TLS_REQUIRE_CLIENT_CERT}"
  "cot_tls_cert_path:=${COT_TLS_CERT_PATH}"
  "cot_tls_key_path:=${COT_TLS_KEY_PATH}"
  "simulator_1_lateral_offset_m:=${SIMULATOR_1_LATERAL_OFFSET_M}"
  "simulator_2_lateral_offset_m:=${SIMULATOR_2_LATERAL_OFFSET_M}"
  "simulator_3_lateral_offset_m:=${SIMULATOR_3_LATERAL_OFFSET_M}"
)

if [[ -n "${COT_TLS_CA_PATH}" ]]; then
  launch_args+=("cot_tls_ca_path:=${COT_TLS_CA_PATH}")
fi

exec ros2 launch atos launch_atosfleetmanagement.py "${launch_args[@]}"
