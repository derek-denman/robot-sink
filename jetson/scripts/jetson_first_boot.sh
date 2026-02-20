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

has_internet() {
  ping -c 1 -W 1 1.1.1.1 >/dev/null 2>&1
}

ROBOT_USER="${ROBOT_USER:-${SUDO_USER:-${USER:-ubuntu}}}"
ROBOT_HOME="$(getent passwd "${ROBOT_USER}" | cut -d: -f6 || true)"
if [[ -z "${ROBOT_HOME}" ]]; then
  ROBOT_HOME="/home/${ROBOT_USER}"
fi

ROBOT_ROOT="${ROBOT_ROOT:-${ROBOT_HOME}/robot-sink}"

log "Applying first-boot baseline for user '${ROBOT_USER}'"

run_sudo timedatectl set-ntp true || true

if has_internet; then
  log "Internet detected. Installing baseline packages."
  run_sudo apt-get update
  run_sudo apt-get install -y \
    openssh-server \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    net-tools \
    iproute2 \
    usbutils \
    ethtool \
    network-manager
else
  log "No internet detected. Skipping apt installs for now."
fi

if systemctl list-unit-files | grep -q '^ssh\.service'; then
  run_sudo systemctl enable --now ssh
fi

for group in dialout video plugdev; do
  if getent group "${group}" >/dev/null 2>&1; then
    run_sudo usermod -aG "${group}" "${ROBOT_USER}"
  fi
done

log "Ensuring working directories exist in ${ROBOT_ROOT}"
run_sudo mkdir -p "${ROBOT_ROOT}/jetson/logs" "${ROBOT_ROOT}/ros_ws/src"
run_sudo chown -R "${ROBOT_USER}:${ROBOT_USER}" "${ROBOT_ROOT}/jetson" "${ROBOT_ROOT}/ros_ws"

log "First-boot baseline complete."
log "Next steps:"
log "  1) ./jetson/scripts/install_jetson_deps.sh"
log "  2) ./jetson/scripts/setup_udev_rules.sh"
log "  3) ./jetson/scripts/setup_bb_usb_network.sh"
log "  4) ./jetson/scripts/run_stack.sh"
