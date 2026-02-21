#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

log() {
  echo "[${SCRIPT_NAME}] $*"
}

run_sudo() {
  if [[ ${EUID} -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

ROBOT_USER="${ROBOT_USER:-${SUDO_USER:-${USER:-ubuntu}}}"
START_NOW=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --user)
      ROBOT_USER="${2:?missing user}"
      shift 2
      ;;
    --now)
      START_NOW=1
      shift
      ;;
    -h|--help)
      cat <<USAGE
Usage:
  ./jetson/scripts/enable_services.sh [--user ubuntu] [--now]
USAGE
      exit 0
      ;;
    *)
      echo "Unknown arg: $1"
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SERVICE_SRC="${REPO_ROOT}/jetson/systemd/robot-jetson-stack.service"
SERVICE_DST="/etc/systemd/system/robot-jetson-stack.service"

if [[ ! -f "${SERVICE_SRC}" ]]; then
  log "Missing service template: ${SERVICE_SRC}"
  exit 1
fi

TMP_FILE="$(mktemp)"
trap 'rm -f "${TMP_FILE}"' EXIT

sed \
  -e "s|__ROBOT_USER__|${ROBOT_USER}|g" \
  -e "s|__ROBOT_ROOT__|${REPO_ROOT}|g" \
  "${SERVICE_SRC}" >"${TMP_FILE}"

log "Installing service to ${SERVICE_DST}"
run_sudo install -m 0644 "${TMP_FILE}" "${SERVICE_DST}"

run_sudo systemctl daemon-reload
run_sudo systemctl enable robot-jetson-stack.service

if [[ ${START_NOW} -eq 1 ]]; then
  run_sudo systemctl restart robot-jetson-stack.service
else
  log "Service enabled but not started. Use --now to start immediately."
fi

log "Check status: sudo systemctl status robot-jetson-stack.service"
log "Follow logs:  sudo journalctl -u robot-jetson-stack.service -f"
