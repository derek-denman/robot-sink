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

TARGET_USER="${ROBOT_USER:-${SUDO_USER:-${USER:-ubuntu}}}"

log "Installing Docker engine (optional)."
run_sudo apt-get update
run_sudo apt-get install -y docker.io docker-compose-plugin
run_sudo systemctl enable --now docker
run_sudo usermod -aG docker "${TARGET_USER}"

log "Docker installation complete."
log "Re-login is required for '${TARGET_USER}' to use docker without sudo."
