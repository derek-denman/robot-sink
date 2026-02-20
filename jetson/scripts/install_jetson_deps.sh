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

TARGET_USER="${ROBOT_USER:-${SUDO_USER:-${USER:-ubuntu}}}"
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6 || true)"
if [[ -z "${TARGET_HOME}" ]]; then
  TARGET_HOME="/home/${TARGET_USER}"
fi

if [[ ! -f /etc/os-release ]]; then
  log "Cannot detect OS version (/etc/os-release missing)."
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}"

log "Detected Ubuntu codename: ${CODENAME}"

if ! has_internet; then
  log "Internet access is required for dependency installation."
  log "Connect ethernet or Wi-Fi, then rerun this script."
  exit 1
fi

run_sudo apt-get update
run_sudo apt-get install -y \
  software-properties-common \
  curl \
  gnupg \
  lsb-release \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-serial \
  net-tools \
  iproute2 \
  iputils-ping \
  ethtool \
  usbutils \
  udev \
  iptables \
  chrony \
  libusb-1.0-0-dev

if [[ "${CODENAME}" == "jammy" ]]; then
  log "Configuring ROS 2 apt repository for Humble."
  run_sudo mkdir -p /etc/apt/keyrings
  if [[ ! -f /etc/apt/keyrings/ros-archive-keyring.gpg ]]; then
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
      run_sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
  fi

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" | \
    run_sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  run_sudo apt-get update
  run_sudo apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rplidar-ros \
    ros-dev-tools

  if ! pip3 show depthai >/dev/null 2>&1; then
    pip3 install --user depthai
  fi
else
  log "ROS 2 Humble binary packages are supported on Ubuntu 22.04 (jammy)."
  log "This system is '${CODENAME}'. Install Humble from source or use Docker on this image."
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  run_sudo rosdep init
fi
rosdep update

BASHRC="${TARGET_HOME}/.bashrc"
ROS_SETUP_LINE="source /opt/ros/humble/setup.bash"
WS_SETUP_LINE="source \$HOME/robot-sink/ros_ws/install/setup.bash"

if [[ -f /opt/ros/humble/setup.bash ]] && ! grep -Fq "${ROS_SETUP_LINE}" "${BASHRC}"; then
  echo "${ROS_SETUP_LINE}" >>"${BASHRC}"
fi

if ! grep -Fq "${WS_SETUP_LINE}" "${BASHRC}"; then
  echo "${WS_SETUP_LINE}" >>"${BASHRC}"
fi

ROBOT_ROOT="${ROBOT_ROOT:-${TARGET_HOME}/robot-sink}"
if [[ -d "${ROBOT_ROOT}/ros_ws" && -f /opt/ros/humble/setup.bash ]]; then
  log "Building ros_ws (if packages are present)."
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  (cd "${ROBOT_ROOT}/ros_ws" && colcon build --symlink-install)
fi

log "Dependency install complete."
