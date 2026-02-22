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

show_serial_candidates() {
  local dev
  shopt -s nullglob
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    log "candidate: ${dev}"
    udevadm info --query=property --name="${dev}" 2>/dev/null | \
      grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL)=' || true
  done
  shopt -u nullglob
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
RULES_SRC="${REPO_ROOT}/jetson/udev/99-robot-devices.rules"
RULES_DST="/etc/udev/rules.d/99-robot-devices.rules"

if [[ ! -f "${RULES_SRC}" ]]; then
  log "Missing rules file: ${RULES_SRC}"
  exit 1
fi

log "Installing udev rules to ${RULES_DST}"
run_sudo install -m 0644 "${RULES_SRC}" "${RULES_DST}"

if getent group dialout >/dev/null 2>&1; then
  log "Ensuring current user is in dialout group"
  run_sudo usermod -aG dialout "${SUDO_USER:-${USER}}"
fi

if getent group plugdev >/dev/null 2>&1; then
  log "Ensuring current user is in plugdev group"
  run_sudo usermod -aG plugdev "${SUDO_USER:-${USER}}"
fi

log "Reloading udev rules"
run_sudo udevadm control --reload-rules
run_sudo udevadm trigger
run_sudo udevadm settle

if [[ -L /dev/ttyRPLIDAR ]]; then
  log "Found /dev/ttyRPLIDAR -> $(readlink /dev/ttyRPLIDAR)"
else
  log "Missing /dev/ttyRPLIDAR"
fi

if [[ -L /dev/ttyROARM ]]; then
  log "Found /dev/ttyROARM -> $(readlink /dev/ttyROARM)"
else
  log "Missing /dev/ttyROARM"
fi

if [[ ! -L /dev/ttyRPLIDAR || ! -L /dev/ttyROARM ]]; then
  log "Available USB serial candidates:"
  show_serial_candidates
  log "Use the ID_VENDOR_ID/ID_MODEL_ID values above to refine jetson/udev/99-robot-devices.rules."
fi

log "Done."
