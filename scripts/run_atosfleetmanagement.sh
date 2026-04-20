#!/usr/bin/env bash

set -euo pipefail

INSECURE="${INSECURE:-True}"
WITH_TRUCK_SIMULATOR="${WITH_TRUCK_SIMULATOR:-False}"
FOXBRIDGE="${FOXBRIDGE:-True}"
COT_TLS_REQUIRE_CLIENT_CERT="${COT_TLS_REQUIRE_CLIENT_CERT:-False}"
COT_TLS_CERT_PATH="${COT_TLS_CERT_PATH:-}"
COT_TLS_KEY_PATH="${COT_TLS_KEY_PATH:-}"
COT_TLS_CA_PATH="${COT_TLS_CA_PATH:-}"

# setup.sh from colcon references vars that may be unset; avoid nounset during sourcing.
set +u
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

launch_args=(
  "insecure:=${INSECURE}"
  "with_truck_simulator:=${WITH_TRUCK_SIMULATOR}"
  "foxbridge:=${FOXBRIDGE}"
  "cot_tls_require_client_cert:=${COT_TLS_REQUIRE_CLIENT_CERT}"
  "cot_tls_cert_path:=${COT_TLS_CERT_PATH}"
  "cot_tls_key_path:=${COT_TLS_KEY_PATH}"
)

if [[ -n "${COT_TLS_CA_PATH}" ]]; then
  launch_args+=("cot_tls_ca_path:=${COT_TLS_CA_PATH}")
fi

exec ros2 launch atos launch_atosfleetmanagement.py "${launch_args[@]}"
