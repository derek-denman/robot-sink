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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LOG_DIR="${JETSON_LOG_DIR:-${REPO_ROOT}/jetson/logs}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
export ROBOT_ROOT="${ROBOT_ROOT:-${REPO_ROOT}}"
ROBOT_CONSOLE_CONFIG="${ROBOT_CONSOLE_CONFIG:-${ROBOT_ROOT}/jetson/console/console_config.yaml}"
ROBOT_CONSOLE_WEB_ROOT="${ROBOT_CONSOLE_WEB_ROOT:-${ROBOT_ROOT}/jetson/console/web}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
export ROBOT_CONSOLE_CONFIG ROBOT_CONSOLE_WEB_ROOT FOXGLOVE_PORT

mkdir -p "${LOG_DIR}"

if [[ "${SETUP_BB_NETWORK:-1}" == "1" ]]; then
  log "Configuring BeagleBone USB networking"
  if ! "${SCRIPT_DIR}/setup_bb_usb_network.sh"; then
    log "Network setup returned non-zero; continuing for bench use."
  fi
fi

if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  safe_source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  log "Sourced /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
else
  log "ROS setup file not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
fi

if [[ -f "${REPO_ROOT}/ros_ws/install/setup.bash" ]]; then
  safe_source "${REPO_ROOT}/ros_ws/install/setup.bash"
  log "Sourced ros_ws install/setup.bash"
fi

PIDS=()
NAMES=()
CMDS=()

start_cmd() {
  local name="$1"
  local cmd="$2"
  local logfile="${LOG_DIR}/${name}.log"

  log "Starting ${name}: ${cmd}"
  bash -lc "${cmd}" >>"${logfile}" 2>&1 &

  local pid="$!"
  PIDS+=("${pid}")
  NAMES+=("${name}")
  CMDS+=("${cmd}")
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

is_critical_process() {
  local name="$1"
  case "${name}" in
    robot-console|foxglove)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

ros_pkg_exists() {
  local pkg="$1"
  command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix "${pkg}" >/dev/null 2>&1
}

if [[ -n "${FOXGLOVE_CMD:-}" ]]; then
  start_cmd "foxglove" "${FOXGLOVE_CMD}"
elif ros_pkg_exists foxglove_bridge; then
  start_cmd "foxglove" "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${FOXGLOVE_PORT}"
else
  start_cmd "foxglove" "python3 ${SCRIPT_DIR}/runtime_stub.py --name foxglove --hint 'foxglove_bridge not found; bridge unavailable'"
fi

if [[ -n "${ROBOT_CONSOLE_CMD:-}" ]]; then
  start_cmd "robot-console" "${ROBOT_CONSOLE_CMD}"
elif ros_pkg_exists robot_console; then
  start_cmd "robot-console" "ros2 launch robot_console robot_console.launch.py start_foxglove:=false config_file:=${ROBOT_CONSOLE_CONFIG} web_root:=${ROBOT_CONSOLE_WEB_ROOT} http_port:=${ROBOT_CONSOLE_PORT:-8080} foxglove_port:=${FOXGLOVE_PORT}"
elif [[ -d "${ROBOT_ROOT}/ros_ws/src/robot_console/robot_console" ]]; then
  start_cmd "robot-console" "PYTHONPATH=${ROBOT_ROOT}/ros_ws/src/robot_console:${PYTHONPATH:-} python3 -m robot_console.api_node --ros-args -p config_file:=${ROBOT_CONSOLE_CONFIG} -p web_root:=${ROBOT_CONSOLE_WEB_ROOT} -p http_port:=${ROBOT_CONSOLE_PORT:-8080} -p foxglove_port:=${FOXGLOVE_PORT}"
else
  start_cmd "robot-console" "python3 ${SCRIPT_DIR}/runtime_stub.py --name robot-console --hint 'robot_console package missing; build ros_ws first'"
fi

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
      if is_critical_process "${name}"; then
        log "Critical process exited unexpectedly: ${name} (pid ${pid})"
        exit 1
      fi

      log "Optional process exited: ${name} (pid ${pid}); keeping core stack alive."
      unset 'PIDS[idx]'
      unset 'NAMES[idx]'
      unset 'CMDS[idx]'
    fi
  done
  sleep 2
done
