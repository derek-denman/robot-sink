#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

log() {
  echo "[${SCRIPT_NAME}] $*"
}

safe_source() {
  local file="$1"
  local had_u=0

  case $- in
    *u*) had_u=1 ;;
  esac

  set +u
  # shellcheck disable=SC1090
  source "${file}"
  local rc=$?

  if [[ ${had_u} -eq 1 ]]; then
    set -u
  fi

  return ${rc}
}

run_sudo() {
  if [[ ${EUID} -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

run_as_target_user() {
  if [[ ${EUID} -eq 0 ]]; then
    sudo -u "${TARGET_USER}" -H "$@"
  else
    "$@"
  fi
}

has_internet() {
  ping -c 1 -W 1 1.1.1.1 >/dev/null 2>&1
}

pkg_available() {
  apt-cache show "$1" >/dev/null 2>&1
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

# Install enough tooling to manage repositories on fresh images.
run_sudo apt-get install -y \
  software-properties-common \
  curl \
  gnupg \
  lsb-release

if command -v add-apt-repository >/dev/null 2>&1; then
  run_sudo add-apt-repository -y universe >/dev/null || true
fi

run_sudo apt-get update
run_sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-venv \
  python3-serial \
  net-tools \
  iproute2 \
  iputils-ping \
  ethtool \
  usbutils \
  udev \
  iptables \
  chrony \
  libusb-1.0-0-dev \
  python3-aiohttp \
  python3-yaml

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
    ros-humble-rplidar-ros

  if pkg_available ros-humble-foxglove-bridge; then
    run_sudo apt-get install -y ros-humble-foxglove-bridge
  else
    log "Package ros-humble-foxglove-bridge not found in apt cache; skipping."
  fi

  if pkg_available ros-dev-tools; then
    run_sudo apt-get install -y ros-dev-tools
  fi

  if ! pip3 show depthai >/dev/null 2>&1; then
    run_as_target_user pip3 install --user depthai
  fi
else
  log "ROS 2 Humble binary packages are supported on Ubuntu 22.04 (jammy)."
  log "This system is '${CODENAME}'. Install Humble from source or use Docker on this image."
fi

MISSING_TOOLS=()

if pkg_available python3-colcon-common-extensions; then
  run_sudo apt-get install -y python3-colcon-common-extensions
else
  MISSING_TOOLS+=(colcon-common-extensions)
fi

if pkg_available python3-vcstool; then
  run_sudo apt-get install -y python3-vcstool
else
  MISSING_TOOLS+=(vcstool)
fi

if pkg_available python3-rosdep; then
  run_sudo apt-get install -y python3-rosdep
elif pkg_available python3-rosdep2; then
  run_sudo apt-get install -y python3-rosdep2
else
  MISSING_TOOLS+=(rosdep)
fi

if [[ ${#MISSING_TOOLS[@]} -gt 0 ]]; then
  log "Falling back to pip for missing tools: ${MISSING_TOOLS[*]}"
  run_as_target_user pip3 install --user "${MISSING_TOOLS[@]}"
fi

ROSDEP_BIN="$(command -v rosdep || true)"
if [[ -z "${ROSDEP_BIN}" && -x "${TARGET_HOME}/.local/bin/rosdep" ]]; then
  ROSDEP_BIN="${TARGET_HOME}/.local/bin/rosdep"
fi

if [[ -n "${ROSDEP_BIN}" ]]; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run_sudo "${ROSDEP_BIN}" init
  fi
  run_as_target_user "${ROSDEP_BIN}" update
else
  log "rosdep command not found after installation; skipping rosdep init/update."
fi

BASHRC="${TARGET_HOME}/.bashrc"
ROS_SETUP_LINE="source /opt/ros/humble/setup.bash"
WS_SETUP_LINE="source \$HOME/robot-sink/ros_ws/install/setup.bash"

if [[ ! -f "${BASHRC}" ]]; then
  run_as_target_user touch "${BASHRC}"
fi

if [[ -f /opt/ros/humble/setup.bash ]] && ! grep -Fq "${ROS_SETUP_LINE}" "${BASHRC}"; then
  echo "${ROS_SETUP_LINE}" | run_as_target_user tee -a "${BASHRC}" >/dev/null
fi

if ! grep -Fq "${WS_SETUP_LINE}" "${BASHRC}"; then
  echo "${WS_SETUP_LINE}" | run_as_target_user tee -a "${BASHRC}" >/dev/null
fi

COLCON_BIN="$(command -v colcon || true)"
if [[ -z "${COLCON_BIN}" && -x "${TARGET_HOME}/.local/bin/colcon" ]]; then
  COLCON_BIN="${TARGET_HOME}/.local/bin/colcon"
fi

ROBOT_ROOT="${ROBOT_ROOT:-${TARGET_HOME}/robot-sink}"
if [[ -d "${ROBOT_ROOT}/ros_ws" && -f /opt/ros/humble/setup.bash && -n "${COLCON_BIN}" ]]; then
  log "Building ros_ws (if packages are present)."
  safe_source /opt/ros/humble/setup.bash
  (cd "${ROBOT_ROOT}/ros_ws" && "${COLCON_BIN}" build --symlink-install)
fi

log "Dependency install complete."
