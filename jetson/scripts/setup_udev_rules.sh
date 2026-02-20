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

log "Reloading udev rules"
run_sudo udevadm control --reload-rules
run_sudo udevadm trigger

log "Done. Replug USB serial devices and verify:"
log "  ls -l /dev/ttyRPLIDAR /dev/ttyROARM"
