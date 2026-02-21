#!/usr/bin/env bash
set -euo pipefail

# Sync BeagleBone payload from repo root -> BeagleBone over SSH (rsync).
# Default: sync ONLY the beaglebone/ directory (the stuff the BB needs).
# Optional: --all to sync the whole repo directly into --dest (still excludes artifacts).

HOST="192.168.7.2"
USER="debian"
DEST="~/robot-sink"
MODE="beaglebone"   # or "all"
DO_DELETE=1
DRYRUN=0

usage() {
  cat <<EOF
Usage:
  ./sync_bb.sh [--host IP] [--user USER] [--dest PATH] [--all] [--no-delete] [--dry-run]

Defaults:
  --host 192.168.7.2
  --user debian
  --dest ~/robot-sink
  (syncs only ./beaglebone/ by default)

Examples:
  ./sync_bb.sh
  ./sync_bb.sh --dest ~/toybot
  ./sync_bb.sh --host 192.168.2.23 --user robot --dest ~/repo
  ./sync_bb.sh --dry-run
  ./sync_bb.sh --all        # sync whole repo (still excludes junk)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host) HOST="${2:?}"; shift 2 ;;
    --user) USER="${2:?}"; shift 2 ;;
    --dest) DEST="${2:?}"; shift 2 ;;
    --all) MODE="all"; shift ;;
    --no-delete) DO_DELETE=0; shift ;;
    --dry-run) DRYRUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

# Determine repo root (works when called from anywhere inside the repo)
if git rev-parse --show-toplevel >/dev/null 2>&1; then
  ROOT="$(git rev-parse --show-toplevel)"
else
  ROOT="$(pwd)"
fi

# Common excludes (safe for both modes)
EXCLUDES=(
  "--exclude=.git/"
  "--exclude=.DS_Store"
  "--exclude=**/.DS_Store"
  "--exclude=**/__pycache__/"
  "--exclude=**/*.pyc"
  "--exclude=**/*.pyo"
  "--exclude=**/.pytest_cache/"
  "--exclude=**/.mypy_cache/"
  "--exclude=**/.ruff_cache/"
  "--exclude=**/.cache/"
  "--exclude=**/.venv/"
  "--exclude=**/venv/"
  "--exclude=**/node_modules/"
  "--exclude=ros_ws/build/"
  "--exclude=ros_ws/install/"
  "--exclude=ros_ws/log/"
)

RSYNC_OPTS=(
  "-avh"
  "--info=stats2,progress2"
  "--partial"
  "--compress"
)

if [[ "$DO_DELETE" -eq 1 ]]; then
  RSYNC_OPTS+=("--delete")
fi

if [[ "$DRYRUN" -eq 1 ]]; then
  RSYNC_OPTS+=("--dry-run")
fi

REMOTE="${USER}@${HOST}:${DEST}"

echo "[sync_bb] repo root: $ROOT"
echo "[sync_bb] mode: $MODE"
echo "[sync_bb] remote: $REMOTE"
echo "[sync_bb] delete: $DO_DELETE  dry-run: $DRYRUN"
echo

# Ensure destination exists
ssh "${USER}@${HOST}" "mkdir -p ${DEST}" >/dev/null

if [[ "$MODE" == "all" ]]; then
  # Sync entire repo directly into DEST/
  rsync "${RSYNC_OPTS[@]}" "${EXCLUDES[@]}" \
    "${ROOT}/" \
    "${USER}@${HOST}:${DEST}/"
else
  # Default: sync only the beaglebone/ subtree into DEST/beaglebone/
  ssh "${USER}@${HOST}" "mkdir -p ${DEST}/beaglebone" >/dev/null
  rsync "${RSYNC_OPTS[@]}" "${EXCLUDES[@]}" \
    "${ROOT}/beaglebone/" \
    "${USER}@${HOST}:${DEST}/beaglebone/"
fi

echo
echo "[sync_bb] done."
