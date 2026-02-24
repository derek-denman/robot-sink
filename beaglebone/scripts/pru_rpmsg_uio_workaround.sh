#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

UENV_PATH="/boot/uEnv.txt"
BACKUP_GLOB="/boot/uEnv.txt.robot-sink.uio-workaround.bak-*"
ENABLE_SERVICES_SCRIPT="${REPO_ROOT}/beaglebone/scripts/enable_services.sh"
INSTALL_DEPS_SCRIPT="${REPO_ROOT}/beaglebone/scripts/install_deps.sh"
DAEMON_REQS="${REPO_ROOT}/beaglebone/host_daemon/requirements.txt"
VENV_PY="${REPO_ROOT}/beaglebone/host_daemon/.venv/bin/python"
VENV_DIR="${REPO_ROOT}/beaglebone/host_daemon/.venv"

MODE=""

usage() {
  cat <<'USAGE'
Usage:
  ./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --plan
  ./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --apply
  ./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --revert

Modes:
  --plan    Show current state and planned persistent workaround changes.
  --apply   Back up /boot/uEnv.txt, force PRU UIO overlay, remove addr4 overlay, and enable live daemon service.
  --revert  Restore newest backup and restore daemon service to normal mode.
USAGE
}

log() {
  echo "[pru_rpmsg_uio_workaround] $*"
}

die() {
  echo "[pru_rpmsg_uio_workaround] ERROR: $*" >&2
  exit 1
}

require_uenv() {
  [[ -f "${UENV_PATH}" ]] || die "${UENV_PATH} not found"
}

get_uenv_key() {
  local key="$1"
  local value=""

  value="$(grep -E "^[[:space:]]*${key}=" "${UENV_PATH}" | tail -n 1 | cut -d= -f2- || true)"
  printf '%s' "${value}"
}

set_uenv_key() {
  local key="$1"
  local value="$2"
  local tmp

  tmp="$(mktemp)"
  cp "${UENV_PATH}" "${tmp}"

  if grep -Eq "^[[:space:]]*${key}=" "${tmp}"; then
    sed -E "s|^[[:space:]]*${key}=.*|${key}=${value}|" "${tmp}" > "${tmp}.new"
    mv "${tmp}.new" "${tmp}"
  else
    printf '\n%s=%s\n' "${key}" "${value}" >> "${tmp}"
  fi

  sudo install -m 0644 "${tmp}" "${UENV_PATH}"
  rm -f "${tmp}"
}

remove_uenv_key() {
  local key="$1"
  local tmp

  tmp="$(mktemp)"
  cp "${UENV_PATH}" "${tmp}"
  sed -E "/^[[:space:]]*${key}=.*/d" "${tmp}" > "${tmp}.new"
  mv "${tmp}.new" "${tmp}"
  sudo install -m 0644 "${tmp}" "${UENV_PATH}"
  rm -f "${tmp}"
}

latest_backup() {
  ls -1t ${BACKUP_GLOB} 2>/dev/null | head -n 1 || true
}

backup_uenv() {
  local stamp backup
  stamp="$(date +%Y%m%d-%H%M%S)"
  backup="/boot/uEnv.txt.robot-sink.uio-workaround.bak-${stamp}"
  sudo cp "${UENV_PATH}" "${backup}"
  log "created backup ${backup}"
}

installed_service_execstart() {
  if [[ -r /etc/systemd/system/bbb-base-daemon.service ]]; then
    grep -E '^ExecStart=' /etc/systemd/system/bbb-base-daemon.service | tail -n 1 || true
    return
  fi

  if command -v systemctl >/dev/null 2>&1; then
    systemctl cat bbb-base-daemon.service 2>/dev/null | grep -E '^ExecStart=' | tail -n 1 || true
  fi
}

service_uses_dry_run() {
  local exec_line
  exec_line="$(installed_service_execstart)"
  if [[ -z "${exec_line}" ]]; then
    return 1
  fi
  printf '%s\n' "${exec_line}" | grep -Eq -- '(^|[[:space:]])--dry-run($|[[:space:]])'
}

ensure_service_prereqs() {
  if [[ -x "${VENV_PY}" ]]; then
    return 0
  fi

  [[ -x "${INSTALL_DEPS_SCRIPT}" ]] || die "missing ${INSTALL_DEPS_SCRIPT}"
  log "host daemon virtualenv missing; running install_deps.sh"
  if ! "${INSTALL_DEPS_SCRIPT}"; then
    log "install_deps.sh failed; attempting minimal host-daemon venv bootstrap"
    [[ -f "${DAEMON_REQS}" ]] || die "missing ${DAEMON_REQS}"
    command -v python3 >/dev/null 2>&1 || die "python3 is required"
    python3 -m venv "${VENV_DIR}"
    "${VENV_DIR}/bin/python" -m pip install -U pip wheel
    "${VENV_DIR}/bin/python" -m pip install -r "${DAEMON_REQS}"
  fi

  [[ -x "${VENV_PY}" ]] || die "virtualenv still missing at ${VENV_PY} after install_deps.sh"
}

parse_args() {
  [[ $# -gt 0 ]] || { usage; exit 1; }

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --plan)
        MODE="plan"
        shift
        ;;
      --apply)
        MODE="apply"
        shift
        ;;
      --revert)
        MODE="revert"
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        die "unknown argument: $1"
        ;;
    esac
  done

  [[ -n "${MODE}" ]] || die "choose one of --plan/--apply/--revert"
}

print_state() {
  local running_kernel configured_kernel configured_overlay configured_addr4
  local exec_line

  running_kernel="$(uname -r)"
  configured_kernel="$(get_uenv_key uname_r)"
  configured_overlay="$(get_uenv_key uboot_overlay_pru)"
  configured_addr4="$(get_uenv_key uboot_overlay_addr4)"
  exec_line="$(installed_service_execstart)"

  log "running kernel: ${running_kernel}"
  log "uEnv uname_r: ${configured_kernel:-<unset>}"
  log "uEnv uboot_overlay_pru: ${configured_overlay:-<unset>}"
  log "uEnv uboot_overlay_addr4: ${configured_addr4:-<unset>}"
  if [[ -n "${exec_line}" ]]; then
    log "bbb-base-daemon ExecStart: ${exec_line}"
  else
    log "bbb-base-daemon ExecStart: <service not installed yet>"
  fi
}

print_plan() {
  local overlay addr4
  local action_count=0

  overlay="$(get_uenv_key uboot_overlay_pru)"
  addr4="$(get_uenv_key uboot_overlay_addr4)"

  if [[ "${overlay}" != "AM335X-PRU-UIO-00A0.dtbo" ]]; then
    log "plan: set uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo"
    action_count=$((action_count + 1))
  fi

  if [[ -n "${addr4}" ]]; then
    log "plan: remove uboot_overlay_addr4=${addr4}"
    action_count=$((action_count + 1))
  fi

  if service_uses_dry_run; then
    log "plan: reinstall/restart bbb-base-daemon.service in live mode (without --dry-run)"
    action_count=$((action_count + 1))
  fi

  if [[ ${action_count} -eq 0 ]]; then
    log "plan: no changes required"
  fi
}

apply_workaround() {
  local overlay addr4

  [[ -x "${ENABLE_SERVICES_SCRIPT}" ]] || die "missing ${ENABLE_SERVICES_SCRIPT}"

  overlay="$(get_uenv_key uboot_overlay_pru)"
  addr4="$(get_uenv_key uboot_overlay_addr4)"

  backup_uenv

  if [[ "${overlay}" != "AM335X-PRU-UIO-00A0.dtbo" ]]; then
    set_uenv_key "uboot_overlay_pru" "AM335X-PRU-UIO-00A0.dtbo"
    log "updated uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo"
  else
    log "uboot_overlay_pru already set to UIO"
  fi

  if [[ -n "${addr4}" ]]; then
    remove_uenv_key "uboot_overlay_addr4"
    log "removed uboot_overlay_addr4"
  else
    log "uboot_overlay_addr4 already absent"
  fi

  ensure_service_prereqs
  "${ENABLE_SERVICES_SCRIPT}"
  log "installed/restarted bbb-base-daemon.service in live mode"
  log "reboot required for PRU overlay changes to take effect"
}

revert_workaround() {
  local backup

  [[ -x "${ENABLE_SERVICES_SCRIPT}" ]] || die "missing ${ENABLE_SERVICES_SCRIPT}"

  backup="$(latest_backup)"
  [[ -n "${backup}" ]] || die "no backups found matching ${BACKUP_GLOB}"

  sudo cp "${backup}" "${UENV_PATH}"
  log "restored ${UENV_PATH} from ${backup}"

  ensure_service_prereqs
  "${ENABLE_SERVICES_SCRIPT}"
  log "installed/restarted bbb-base-daemon.service in normal mode"
  log "reboot required for restored PRU overlay/kernel settings to take effect"
}

main() {
  parse_args "$@"
  require_uenv
  print_state

  case "${MODE}" in
    plan)
      print_plan
      ;;
    apply)
      print_plan
      apply_workaround
      ;;
    revert)
      revert_workaround
      ;;
    *)
      die "unsupported mode ${MODE}"
      ;;
  esac
}

main "$@"
