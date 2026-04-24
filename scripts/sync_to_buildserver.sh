#!/usr/bin/env bash
set -euo pipefail

# Sync current ATOS repository to build server.
# Default target:
#   az-buildserver@srv-l039-p:~/ATOS
#
# Usage:
#   scripts/sync_to_buildserver.sh
#   scripts/sync_to_buildserver.sh --dry-run
#   scripts/sync_to_buildserver.sh --delete
#   scripts/sync_to_buildserver.sh --delete --remote-dir /home/az-buildserver/ATOS

REMOTE_USER="${REMOTE_USER:-az-buildserver}"
REMOTE_HOST="${REMOTE_HOST:-srv-l039-p}"
REMOTE_DIR="${REMOTE_DIR:-/home/az-buildserver/ATOS}"

DELETE_FLAG=""
DRY_RUN_FLAG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --delete)
      DELETE_FLAG="--delete"
      shift
      ;;
    --dry-run|-n)
      DRY_RUN_FLAG="--dry-run"
      shift
      ;;
    --remote-user)
      REMOTE_USER="$2"
      shift 2
      ;;
    --remote-host)
      REMOTE_HOST="$2"
      shift 2
      ;;
    --remote-dir)
      REMOTE_DIR="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Sync ATOS repo to remote build server.

Options:
  --delete              Remove files on remote that no longer exist locally.
  --dry-run, -n         Show what would change, without changing files.
  --remote-user USER    Override SSH user (default: az-buildserver).
  --remote-host HOST    Override SSH host (default: srv-l039-p).
  --remote-dir DIR      Override remote ATOS path (default: /home/az-buildserver/ATOS).
EOF
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use --help for usage."
      exit 1
      ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${REMOTE_USER}@${REMOTE_HOST}"

echo "Sync source: ${REPO_ROOT}/"
echo "Sync target: ${REMOTE}:${REMOTE_DIR}/"
if [[ -n "${DELETE_FLAG}" ]]; then
  echo "Mode: mirror update (with delete)"
else
  echo "Mode: incremental update (no delete)"
fi
if [[ -n "${DRY_RUN_FLAG}" ]]; then
  echo "Dry-run: enabled"
fi

ssh "${REMOTE}" "mkdir -p '${REMOTE_DIR}'"

rsync -azvi --human-readable \
  ${DRY_RUN_FLAG} \
  ${DELETE_FLAG} \
  --exclude ".git" \
  --exclude ".codex" \
  --exclude "build" \
  --exclude "install" \
  --exclude "log" \
  --exclude "__pycache__" \
  "${REPO_ROOT}/" "${REMOTE}:${REMOTE_DIR}/"

echo
echo "Sync complete."
echo "Run these on server:"
echo "  cd ${REMOTE_DIR}"
echo "  docker compose -f docker-compose-fleetmanagement.yml down"
echo "  COT_TLS_REQUIRE_CLIENT_CERT=False \\"
echo "  COT_TLS_CERT_PATH=/root/.astazero/ATOS/certs/server.crt \\"
echo "  COT_TLS_KEY_PATH=/root/.astazero/ATOS/certs/server.key \\"
echo "  COT_TLS_CA_PATH=/root/.astazero/ATOS/certs/ca-trusted.pem \\"
echo "  docker compose -f docker-compose-fleetmanagement.yml up -d --build"
echo "  docker compose -f docker-compose-fleetmanagement.yml logs -f atos-fleetmanagement"
