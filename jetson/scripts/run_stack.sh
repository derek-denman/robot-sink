#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

log() {
  echo "[${SCRIPT_NAME}] $*"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOG_DIR="${JETSON_LOG_DIR:-${REPO_ROOT}/jetson/logs}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"

mkdir -p "${LOG_DIR}"

if [[ "${SETUP_BB_NETWORK:-1}" == "1" ]]; then
  log "Configuring BeagleBone USB networking"
  if ! "${SCRIPT_DIR}/setup_bb_usb_network.sh"; then
    log "Network setup returned non-zero; continuing for bench use."
  fi
fi

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  log "Sourced /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
else
  log "ROS setup file not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
fi

if [[ -f "${REPO_ROOT}/ros_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/ros_ws/install/setup.bash"
  log "Sourced ros_ws install/setup.bash"
fi

PIDS=()
NAMES=()

start_cmd() {
  local name="$1"
  local cmd="$2"
  local logfile="${LOG_DIR}/${name}.log"

  log "Starting ${name}: ${cmd}"
  bash -lc "${cmd}" >>"${logfile}" 2>&1 &

  local pid="$!"
  PIDS+=("${pid}")
  NAMES+=("${name}")
}

cleanup() {
  log "Stopping stack processes"
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait || true
}

trap cleanup EXIT INT TERM

ros_pkg_exists() {
  local pkg="$1"
  command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix "${pkg}" >/dev/null 2>&1
}

if [[ -n "${OAK_LAUNCH_CMD:-}" ]]; then
  start_cmd "oakd" "${OAK_LAUNCH_CMD}"
elif ros_pkg_exists depthai_ros_driver; then
  start_cmd "oakd" "ros2 launch depthai_ros_driver camera.launch.py"
else
  start_cmd "oakd" "python3 ${SCRIPT_DIR}/runtime_stub.py --name oakd --hint 'depthai_ros_driver not found; running stub'"
fi

if [[ -n "${RPLIDAR_LAUNCH_CMD:-}" ]]; then
  start_cmd "rplidar" "${RPLIDAR_LAUNCH_CMD}"
elif ros_pkg_exists rplidar_ros; then
  start_cmd "rplidar" "ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyRPLIDAR frame_id:=laser"
else
  start_cmd "rplidar" "python3 ${SCRIPT_DIR}/runtime_stub.py --name rplidar --check-device /dev/ttyRPLIDAR --hint 'rplidar_ros not found; running stub'"
fi

if [[ -n "${ROARM_CMD:-}" ]]; then
  start_cmd "roarm" "${ROARM_CMD}"
else
  start_cmd "roarm" "python3 ${SCRIPT_DIR}/runtime_stub.py --name roarm --check-device /dev/ttyROARM --hint 'RoArm ROS node not configured; running serial stub'"
fi

if [[ -n "${TASK_STACK_CMD:-}" ]]; then
  start_cmd "task" "${TASK_STACK_CMD}"
else
  start_cmd "task" "python3 ${SCRIPT_DIR}/runtime_stub.py --name task-stack --hint 'Task stack placeholder running'"
fi

if [[ -n "${BB_BRIDGE_CMD:-}" ]]; then
  start_cmd "bb-bridge" "${BB_BRIDGE_CMD}"
else
  start_cmd "bb-bridge" "python3 ${SCRIPT_DIR}/bb_bridge_stub.py --host ${BB_HOST:-192.168.7.2} --port ${BB_PORT:-8765}"
fi

log "Stack running. Logs: ${LOG_DIR}"
while true; do
  for idx in "${!PIDS[@]}"; do
    pid="${PIDS[$idx]}"
    name="${NAMES[$idx]}"
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      log "Process exited unexpectedly: ${name} (pid ${pid})"
      exit 1
    fi
  done
  sleep 2
done
