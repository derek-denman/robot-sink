#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

log() {
  echo "[${SCRIPT_NAME}] $*"
}

usage() {
  cat <<USAGE
Usage:
  ./jetson/scripts/sync_to_jetson.sh [--dry-run] [--no-delete] user@JETSON_IP:/home/user/robot-sink

Examples:
  ./jetson/scripts/sync_to_jetson.sh ubuntu@192.168.1.42:/home/ubuntu/robot-sink
  ./jetson/scripts/sync_to_jetson.sh --dry-run ubuntu@192.168.1.42:/home/ubuntu/robot-sink
USAGE
}

DRY_RUN=0
DO_DELETE=1
TARGET=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --no-delete)
      DO_DELETE=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      TARGET="$1"
      shift
      ;;
  esac
done

if [[ -z "${TARGET}" || "${TARGET}" != *:* ]]; then
  usage
  exit 1
fi

if ! command -v rsync >/dev/null 2>&1; then
  log "rsync is required but not installed."
  exit 1
fi

if ! command -v ssh >/dev/null 2>&1; then
  log "ssh is required but not installed."
  exit 1
fi

if git rev-parse --show-toplevel >/dev/null 2>&1; then
  ROOT="$(git rev-parse --show-toplevel)"
else
  ROOT="$(pwd)"
fi

RSYNC_VERSION_LINE="$(rsync --version | head -n 1 || true)"
log "local rsync: ${RSYNC_VERSION_LINE}"

EXCLUDES_FILE="${ROOT}/jetson/scripts/sync_to_jetson.exclude"
if [[ ! -f "${EXCLUDES_FILE}" ]]; then
  log "Missing excludes file: ${EXCLUDES_FILE}"
  exit 1
fi

REMOTE_HOST="${TARGET%%:*}"
REMOTE_PATH="${TARGET#*:}"

log "repo root: ${ROOT}"
log "target host: ${REMOTE_HOST}"
log "target path: ${REMOTE_PATH}"
log "delete mode: ${DO_DELETE}"
log "dry run: ${DRY_RUN}"

ssh "${REMOTE_HOST}" "mkdir -p '${REMOTE_PATH}'"

RSYNC_OPTS=(
  -avh
  --compress
  --partial
  --exclude-from="${EXCLUDES_FILE}"
)

if rsync --help 2>&1 | grep -q -- "--info"; then
  RSYNC_OPTS+=(--info=stats2,progress2)
elif rsync --help 2>&1 | grep -q -- "--progress"; then
  RSYNC_OPTS+=(--progress)
  log "Using --progress (older rsync detected)."
else
  log "No rsync progress flag support detected; continuing without progress output."
fi

if [[ ${DRY_RUN} -eq 1 ]]; then
  RSYNC_OPTS+=(--dry-run)
fi

REQUIRED_PATHS=(
  jetson
  ros_ws
  docs
  README.md
)

OPTIONAL_PATHS=(
  beaglebone/host_daemon/config.yaml
)

for rel_path in "${REQUIRED_PATHS[@]}"; do
  src="${ROOT}/${rel_path}"
  if [[ ! -e "${src}" ]]; then
    log "Required path missing: ${rel_path}"
    exit 1
  fi
  log "Syncing ${rel_path}"
  if [[ -d "${src}" && ${DO_DELETE} -eq 1 ]]; then
    rsync "${RSYNC_OPTS[@]}" --delete "${src}" "${TARGET}/"
  else
    rsync "${RSYNC_OPTS[@]}" "${src}" "${TARGET}/"
  fi
done

for rel_path in "${OPTIONAL_PATHS[@]}"; do
  src="${ROOT}/${rel_path}"
  if [[ -e "${src}" ]]; then
    ssh "${REMOTE_HOST}" "mkdir -p '${REMOTE_PATH}/$(dirname "${rel_path}")'"
    log "Syncing optional ${rel_path}"
    rsync "${RSYNC_OPTS[@]}" "${src}" "${REMOTE_HOST}:${REMOTE_PATH}/${rel_path}"
  fi
done

log "Sync complete."
