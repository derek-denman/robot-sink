#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

log() {
  echo "[${SCRIPT_NAME}] $*"
}

usage() {
  cat <<USAGE
Usage:
  ./sync_nodes.sh [options]

Syncs this repo from the current machine to Jetson and/or BeagleBone so both nodes
stay on the same code snapshot.

Options:
  --target TARGET            one of: both | jetson | beaglebone (default: both)

  --jetson TARGET            Jetson destination in user@host:/path form
                             (default: jetson@10.0.0.178:/home/jetson/robot-sink)

  --bb-host HOST             BeagleBone host/IP (default: 192.168.7.2)
  --bb-user USER             BeagleBone SSH user (default: debian)
  --bb-dest PATH             BeagleBone destination root (default: ~/robot-sink)
  --bb-all                   Sync whole repo to BeagleBone (default: only beaglebone/)

  --dry-run                  Show what would be synced
  --no-delete                Disable rsync --delete
  -h, --help                 Show this help

Examples:
  ./sync_nodes.sh
  ./sync_nodes.sh --dry-run
  ./sync_nodes.sh --target jetson --jetson ubuntu@10.0.0.55:/home/ubuntu/robot-sink
  ./sync_nodes.sh --target beaglebone --bb-host 192.168.7.2 --bb-user debian
  ./sync_nodes.sh --bb-all
USAGE
}

TARGET="both"
JETSON_TARGET="jetson@10.0.0.178:/home/jetson/robot-sink"
BB_HOST="192.168.7.2"
BB_USER="debian"
BB_DEST="~/robot-sink"
BB_MODE="beaglebone"
DRY_RUN=0
DO_DELETE=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target)
      TARGET="${2:?}"
      shift 2
      ;;
    --jetson)
      JETSON_TARGET="${2:?}"
      shift 2
      ;;
    --bb-host)
      BB_HOST="${2:?}"
      shift 2
      ;;
    --bb-user)
      BB_USER="${2:?}"
      shift 2
      ;;
    --bb-dest)
      BB_DEST="${2:?}"
      shift 2
      ;;
    --bb-all)
      BB_MODE="all"
      shift
      ;;
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
      log "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ "${TARGET}" != "both" && "${TARGET}" != "jetson" && "${TARGET}" != "beaglebone" ]]; then
  log "Invalid --target value: ${TARGET}"
  usage
  exit 1
fi

if [[ "${TARGET}" == "both" || "${TARGET}" == "jetson" ]]; then
  if [[ "${JETSON_TARGET}" != *:* ]]; then
    log "--jetson must be in user@host:/path form"
    exit 1
  fi
fi

for cmd in rsync ssh; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    log "Required command not found: ${cmd}"
    exit 1
  fi
done

if git rev-parse --show-toplevel >/dev/null 2>&1; then
  ROOT="$(git rev-parse --show-toplevel)"
else
  ROOT="$(pwd)"
fi

JETSON_EXCLUDES_FILE="${ROOT}/jetson/scripts/sync_to_jetson.exclude"
if [[ ! -f "${JETSON_EXCLUDES_FILE}" ]]; then
  log "Missing exclude file: ${JETSON_EXCLUDES_FILE}"
  exit 1
fi

RSYNC_BASE_OPTS=(
  -avh
  --compress
  --partial
)

if rsync --help 2>&1 | grep -q -- "--info"; then
  RSYNC_BASE_OPTS+=(--info=stats2,progress2)
elif rsync --help 2>&1 | grep -q -- "--progress"; then
  RSYNC_BASE_OPTS+=(--progress)
fi

if [[ ${DRY_RUN} -eq 1 ]]; then
  RSYNC_BASE_OPTS+=(--dry-run)
fi

if [[ ${DO_DELETE} -eq 1 ]]; then
  RSYNC_DELETE_OPT=(--delete)
else
  RSYNC_DELETE_OPT=()
fi

sync_jetson() {
  local remote_host remote_path

  remote_host="${JETSON_TARGET%%:*}"
  remote_path="${JETSON_TARGET#*:}"

  log "syncing Jetson -> ${JETSON_TARGET}"
  ssh "${remote_host}" "mkdir -p '${remote_path}'"

  local -a required_paths=(
    jetson
    ros_ws
    docs
    README.md
  )
  local -a optional_paths=(
    beaglebone/host_daemon/config.yaml
  )

  local src rel_path
  for rel_path in "${required_paths[@]}"; do
    src="${ROOT}/${rel_path}"
    if [[ ! -e "${src}" ]]; then
      log "required path missing for jetson sync: ${rel_path}"
      exit 1
    fi

    if [[ -d "${src}" ]]; then
      log "jetson sync: ${rel_path}/"
      rsync "${RSYNC_BASE_OPTS[@]}" "${RSYNC_DELETE_OPT[@]}" --exclude-from="${JETSON_EXCLUDES_FILE}" "${src}" "${JETSON_TARGET}/"
    else
      log "jetson sync: ${rel_path}"
      rsync "${RSYNC_BASE_OPTS[@]}" --exclude-from="${JETSON_EXCLUDES_FILE}" "${src}" "${JETSON_TARGET}/"
    fi
  done

  for rel_path in "${optional_paths[@]}"; do
    src="${ROOT}/${rel_path}"
    if [[ -e "${src}" ]]; then
      ssh "${remote_host}" "mkdir -p '${remote_path}/$(dirname "${rel_path}")'"
      log "jetson sync: optional ${rel_path}"
      rsync "${RSYNC_BASE_OPTS[@]}" "${src}" "${remote_host}:${remote_path}/${rel_path}"
    fi
  done
}

sync_beaglebone() {
  local remote

  remote="${BB_USER}@${BB_HOST}"
  log "syncing BeagleBone -> ${remote}:${BB_DEST} (mode=${BB_MODE})"

  local -a bb_excludes=(
    --exclude=.git/
    --exclude=.DS_Store
    --exclude=**/.DS_Store
    --exclude=**/__pycache__/
    --exclude=**/*.pyc
    --exclude=**/*.pyo
    --exclude=**/.pytest_cache/
    --exclude=**/.mypy_cache/
    --exclude=**/.ruff_cache/
    --exclude=**/.cache/
    --exclude=**/.venv/
    --exclude=**/venv/
    --exclude=**/node_modules/
    --exclude=ros_ws/build/
    --exclude=ros_ws/install/
    --exclude=ros_ws/log/
  )

  local -a bb_rsync_opts=("${RSYNC_BASE_OPTS[@]}")
  if [[ ${DO_DELETE} -eq 1 ]]; then
    bb_rsync_opts+=(--delete)
  fi

  ssh "${remote}" "mkdir -p '${BB_DEST}'"

  if [[ "${BB_MODE}" == "all" ]]; then
    rsync "${bb_rsync_opts[@]}" "${bb_excludes[@]}" "${ROOT}/" "${remote}:${BB_DEST}/"
  else
    ssh "${remote}" "mkdir -p '${BB_DEST}/beaglebone'"
    rsync "${bb_rsync_opts[@]}" "${bb_excludes[@]}" "${ROOT}/beaglebone/" "${remote}:${BB_DEST}/beaglebone/"
  fi
}

log "repo root: ${ROOT}"
log "target=${TARGET} delete=${DO_DELETE} dry-run=${DRY_RUN}"

case "${TARGET}" in
  both)
    sync_jetson
    sync_beaglebone
    ;;
  jetson)
    sync_jetson
    ;;
  beaglebone)
    sync_beaglebone
    ;;
esac

log "sync complete"
