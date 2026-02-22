#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

UENV_PATH="/boot/uEnv.txt"
BACKUP_GLOB="/boot/uEnv.txt.pru-rproc-irq-fix.bak-*"

OVERLAY_SRC="${REPO_ROOT}/beaglebone/overlays/BB-PRU-RPROC-IRQ-FIX.dts"
OVERLAY_DST="/lib/firmware/BB-PRU-RPROC-IRQ-FIX.dtbo"
UENV_KEY="uboot_overlay_addr4"
UENV_VALUE="${OVERLAY_DST}"

MODE=""

usage() {
  cat <<'USAGE'
Usage:
  ./beaglebone/scripts/pru_rproc_irq_fix.sh --plan
  ./beaglebone/scripts/pru_rproc_irq_fix.sh --apply
  ./beaglebone/scripts/pru_rproc_irq_fix.sh --revert

Modes:
  --plan    Show current state and planned changes.
  --apply   Compile/install overlay and set uEnv overlay entry (with backup).
  --revert  Restore the latest /boot/uEnv.txt backup created by this script.
USAGE
}

log() {
  echo "[pru_rproc_irq_fix] $*"
}

die() {
  echo "[pru_rproc_irq_fix] ERROR: $*" >&2
  exit 1
}

require_uenv() {
  [[ -f "${UENV_PATH}" ]] || die "${UENV_PATH} not found"
}

get_uenv_key() {
  local key="$1"
  local value=""
  value="$(grep -E "^[[:space:]]*${key}=" "${UENV_PATH}" | tail -n1 | cut -d= -f2- || true)"
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

latest_backup() {
  ls -1t ${BACKUP_GLOB} 2>/dev/null | head -n1 || true
}

backup_uenv() {
  local stamp backup
  stamp="$(date +%Y%m%d-%H%M%S)"
  backup="/boot/uEnv.txt.pru-rproc-irq-fix.bak-${stamp}"
  sudo cp "${UENV_PATH}" "${backup}"
  log "created backup ${backup}"
}

detect_irq_mapping_error() {
  dmesg -T | egrep -i 'irq vring not found|unable to get vring interrupt|irq kick not found|unable to get kick interrupt|boot failed: -6' >/dev/null 2>&1
}

show_plan() {
  log "running kernel: $(uname -r)"
  log "uEnv uname_r: $(get_uenv_key uname_r)"
  log "uEnv uboot_overlay_pru: $(get_uenv_key uboot_overlay_pru)"
  log "uEnv ${UENV_KEY}: $(get_uenv_key "${UENV_KEY}")"
  log "overlay source: ${OVERLAY_SRC}"
  log "overlay destination: ${OVERLAY_DST}"

  if [[ -f "${OVERLAY_DST}" ]]; then
    log "overlay destination file exists"
  else
    log "overlay destination file does not exist"
  fi

  if detect_irq_mapping_error; then
    log "detected PRU RPMsg IRQ mapping failure signature in dmesg (-6 / missing vring|kick IRQ)"
  fi

  cat <<'EONEXT'
[pru_rproc_irq_fix] plan:
  - compile beaglebone/overlays/BB-PRU-RPROC-IRQ-FIX.dts into /lib/firmware/BB-PRU-RPROC-IRQ-FIX.dtbo
  - overlay enforces PRU interrupt-names = "vring", "kick" for pru0/pru1
  - set uboot_overlay_addr4=/lib/firmware/BB-PRU-RPROC-IRQ-FIX.dtbo in /boot/uEnv.txt
  - reboot, then redeploy firmware
EONEXT
}

apply_fix() {
  [[ -f "${OVERLAY_SRC}" ]] || die "overlay source missing: ${OVERLAY_SRC}"
  command -v dtc >/dev/null 2>&1 || die "dtc not found; install device-tree-compiler"

  local tmp
  tmp="$(mktemp --suffix=.dtbo)"
  dtc -@ -I dts -O dtb -o "${tmp}" "${OVERLAY_SRC}"

  sudo install -m 0644 "${tmp}" "${OVERLAY_DST}"
  rm -f "${tmp}"

  backup_uenv
  set_uenv_key "${UENV_KEY}" "${UENV_VALUE}"

  log "installed ${OVERLAY_DST}"
  log "updated ${UENV_KEY}=${UENV_VALUE}"
  log "reboot required"
}

revert_fix() {
  local backup
  backup="$(latest_backup)"
  [[ -n "${backup}" ]] || die "no backups found matching ${BACKUP_GLOB}"

  sudo cp "${backup}" "${UENV_PATH}"
  log "restored ${UENV_PATH} from ${backup}"
  log "reboot required"
}

parse_args() {
  [[ $# -eq 1 ]] || { usage; exit 1; }
  case "$1" in
    --plan) MODE="plan" ;;
    --apply) MODE="apply" ;;
    --revert) MODE="revert" ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 1 ;;
  esac
}

main() {
  parse_args "$@"
  require_uenv

  case "${MODE}" in
    plan)
      show_plan
      ;;
    apply)
      show_plan
      apply_fix
      ;;
    revert)
      revert_fix
      ;;
  esac
}

main "$@"
