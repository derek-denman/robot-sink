#!/usr/bin/env bash
set -euo pipefail

UENV_PATH="/boot/uEnv.txt"
BACKUP_GLOB="/boot/uEnv.txt.robot-sink.kernel-hop.bak-*"

MODE=""
TARGET_KERNEL=""
TARGET_OVERLAY=""

PLAN_REASON=""
PLAN_KERNEL=""
PLAN_OVERLAY=""
PLAN_NEEDS_UNAME_CHANGE=0
PLAN_NEEDS_OVERLAY_CHANGE=0
KICK_OOPS_DETECTED=0

usage() {
  cat <<'USAGE'
Usage:
  ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --plan [--kernel <uname_r>] [--overlay <dtbo>]
  ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --apply [--kernel <uname_r>] [--overlay <dtbo>]
  ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --revert

Modes:
  --plan     Print inventory and planned kernel/overlay changes.
  --apply    Backup /boot/uEnv.txt then apply plan.
  --revert   Restore most recent backup made by this script.

Options:
  --kernel   Explicit target uname_r. Must already exist as /boot/vmlinuz-<uname_r>.
  --overlay  Explicit target uboot_overlay_pru for selected kernel.
USAGE
}

log() {
  echo "[pru_rpmsg_kernel_hop] $*"
}

die() {
  echo "[pru_rpmsg_kernel_hop] ERROR: $*" >&2
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

list_installed_kernels() {
  local image
  for image in /boot/vmlinuz-*; do
    [[ -e "${image}" ]] || continue
    basename "${image}" | sed 's/^vmlinuz-//'
  done | LC_ALL=C sort
}

kernel_installed() {
  local kernel="$1"
  [[ -f "/boot/vmlinuz-${kernel}" ]]
}

overlays_dir_for_kernel() {
  local kernel="$1"
  local candidate

  for candidate in \
    "/boot/dtbs/${kernel}/overlays" \
    "/boot/dtb/${kernel}/overlays" \
    "/boot/firmware/dtbs/${kernel}/overlays"; do
    if [[ -d "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

list_pru_overlays_for_kernel() {
  local kernel="$1"
  local overlay_dir

  overlay_dir="$(overlays_dir_for_kernel "${kernel}" || true)"
  [[ -n "${overlay_dir}" ]] || return 0

  find "${overlay_dir}" -maxdepth 1 -type f -name 'AM335X-PRU-*.dtbo' -print \
    | sed 's|.*/||' \
    | LC_ALL=C sort
}

pick_best_rproc_overlay() {
  local kernel="$1"
  local tag
  local name
  local -a rproc=()

  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    if [[ "${name}" == *"RPROC"* ]]; then
      rproc+=("${name}")
    fi
  done < <(list_pru_overlays_for_kernel "${kernel}")

  [[ ${#rproc[@]} -gt 0 ]] || return 1

  tag="$(printf '%s' "${kernel}" | sed -E 's/^([0-9]+\.[0-9]+).*/\1/' | tr '.' '-')"
  for name in "${rproc[@]}"; do
    if [[ "${name}" == *"${tag}"* ]]; then
      printf '%s\n' "${name}"
      return 0
    fi
  done

  printf '%s\n' "${rproc[0]}"
}

kernel_has_rproc_overlay() {
  local kernel="$1"
  pick_best_rproc_overlay "${kernel}" >/dev/null 2>&1
}

detect_kick_oops_signature() {
  local hints

  hints="$(dmesg -T | egrep -i 'remoteproc|rproc-virtio|pru|rpmsg|oops|segfault|bug' | tail -n 160 || true)"
  if printf '%s\n' "${hints}" | grep -Eqi 'pru_rproc_kick|internal error: oops|oops: [0-9]|kick method not defined|boot failed: -22'; then
    return 0
  fi

  return 1
}

latest_backup() {
  ls -1t ${BACKUP_GLOB} 2>/dev/null | head -n 1 || true
}

backup_uenv() {
  local stamp backup
  stamp="$(date +%Y%m%d-%H%M%S)"
  backup="/boot/uEnv.txt.robot-sink.kernel-hop.bak-${stamp}"

  sudo cp "${UENV_PATH}" "${backup}"
  log "created backup ${backup}"
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

show_inventory() {
  local running selected_kernel selected_overlay kernel overlay_dir

  running="$(uname -r)"
  selected_kernel="$(get_uenv_key uname_r)"
  selected_overlay="$(get_uenv_key uboot_overlay_pru)"

  log "running kernel: ${running}"
  log "uEnv uname_r: ${selected_kernel:-<unset>}"
  log "uEnv uboot_overlay_pru: ${selected_overlay:-<unset>}"

  log "installed kernels from /boot/vmlinuz-*"
  list_installed_kernels | sed 's/^/  - /'

  while IFS= read -r kernel; do
    [[ -n "${kernel}" ]] || continue
    overlay_dir="$(overlays_dir_for_kernel "${kernel}" || true)"
    if [[ -z "${overlay_dir}" ]]; then
      log "kernel ${kernel}: overlays dir missing"
      continue
    fi

    log "kernel ${kernel}: overlays dir ${overlay_dir}"
    if ! list_pru_overlays_for_kernel "${kernel}" | sed 's/^/  - /'; then
      true
    fi
  done < <(list_installed_kernels)

  if command -v apt-cache >/dev/null 2>&1; then
    log "apt-cache TI kernel packages (if index available):"
    apt-cache search '^linux-image-.*-ti-r[0-9]+$' 2>/dev/null | awk '{print "  - "$1}' | LC_ALL=C sort || true
  fi
}

validate_overlay_for_kernel() {
  local kernel="$1"
  local overlay="$2"

  [[ -n "${overlay}" ]] || return 1
  list_pru_overlays_for_kernel "${kernel}" | grep -Fxq "${overlay}"
}

build_plan() {
  local running selected_kernel selected_overlay effective_kernel candidate_kernel
  local best_overlay chosen_kernel chosen_overlay

  running="$(uname -r)"
  selected_kernel="$(get_uenv_key uname_r)"
  selected_overlay="$(get_uenv_key uboot_overlay_pru)"
  effective_kernel="${selected_kernel:-${running}}"
  KICK_OOPS_DETECTED=0

  if detect_kick_oops_signature; then
    KICK_OOPS_DETECTED=1
  fi

  PLAN_REASON="No change required."
  PLAN_KERNEL="${effective_kernel}"
  PLAN_OVERLAY="${selected_overlay}"
  PLAN_NEEDS_UNAME_CHANGE=0
  PLAN_NEEDS_OVERLAY_CHANGE=0

  if [[ -n "${TARGET_KERNEL}" ]]; then
    kernel_installed "${TARGET_KERNEL}" || die "--kernel ${TARGET_KERNEL} is not installed under /boot/vmlinuz-*"
    chosen_kernel="${TARGET_KERNEL}"
  else
    chosen_kernel="${effective_kernel}"
    if ! kernel_has_rproc_overlay "${chosen_kernel}"; then
      chosen_kernel=""
      while IFS= read -r candidate_kernel; do
        [[ -n "${candidate_kernel}" ]] || continue
        if kernel_has_rproc_overlay "${candidate_kernel}"; then
          chosen_kernel="${candidate_kernel}"
          if [[ "${candidate_kernel}" == *"-ti-"* ]]; then
            break
          fi
        fi
      done < <(list_installed_kernels)
    fi
  fi

  if [[ -z "${chosen_kernel}" ]]; then
    PLAN_REASON="No installed kernel exposes AM335X-PRU-RPROC overlays. Install a compatible TI kernel and retry."
    PLAN_KERNEL="${effective_kernel}"
    PLAN_OVERLAY="${selected_overlay}"
    return
  fi

  if [[ -z "${TARGET_KERNEL}" ]] && [[ ${KICK_OOPS_DETECTED} -eq 1 ]]; then
    while IFS= read -r candidate_kernel; do
      [[ -n "${candidate_kernel}" ]] || continue
      [[ "${candidate_kernel}" == "${effective_kernel}" ]] && continue
      if kernel_has_rproc_overlay "${candidate_kernel}"; then
        chosen_kernel="${candidate_kernel}"
        break
      fi
    done < <(list_installed_kernels)
  fi

  if [[ -n "${TARGET_OVERLAY}" ]]; then
    validate_overlay_for_kernel "${chosen_kernel}" "${TARGET_OVERLAY}" || \
      die "--overlay ${TARGET_OVERLAY} not found under overlays for kernel ${chosen_kernel}"
    chosen_overlay="${TARGET_OVERLAY}"
  else
    best_overlay="$(pick_best_rproc_overlay "${chosen_kernel}" || true)"
    if [[ -n "${best_overlay}" ]]; then
      chosen_overlay="${best_overlay}"
    else
      chosen_overlay="${selected_overlay}"
    fi
  fi

  PLAN_KERNEL="${chosen_kernel}"
  PLAN_OVERLAY="${chosen_overlay}"

  if [[ "${PLAN_KERNEL}" != "${effective_kernel}" ]]; then
    PLAN_NEEDS_UNAME_CHANGE=1
    if [[ ${KICK_OOPS_DETECTED} -eq 1 ]]; then
      PLAN_REASON="Current kernel shows RPMsg kick-path Oops; switch to alternate installed kernel with PRU RPROC overlay."
    else
      PLAN_REASON="Switch to installed kernel with PRU RPROC overlay support for RPMsg stability."
    fi
  fi

  if [[ -n "${PLAN_OVERLAY}" && "${PLAN_OVERLAY}" != "${selected_overlay}" ]]; then
    PLAN_NEEDS_OVERLAY_CHANGE=1
    if [[ "${selected_overlay}" == *"UIO"* ]]; then
      PLAN_REASON="Switch PRU overlay from UIO to RPROC for RPMsg firmware."
    elif [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 0 ]]; then
      PLAN_REASON="Select a matching AM335X-PRU-RPROC overlay for current kernel."
    fi
  fi

  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 0 && ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 0 ]]; then
    PLAN_REASON="No change required."
  fi
}

show_plan() {
  log "plan: ${PLAN_REASON}"

  if [[ ${KICK_OOPS_DETECTED} -eq 1 ]]; then
    log "observed kick-path failure signature in recent dmesg."
  fi

  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 1 ]]; then
    log "planned uname_r=${PLAN_KERNEL}"
  else
    log "planned uname_r: no change"
  fi

  if [[ ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 1 ]]; then
    log "planned uboot_overlay_pru=${PLAN_OVERLAY}"
  else
    log "planned uboot_overlay_pru: no change"
  fi

  if [[ "${PLAN_REASON}" == "No installed kernel exposes AM335X-PRU-RPROC overlays. Install a compatible TI kernel and retry." ]]; then
    cat <<'EONEXT'
[pru_rpmsg_kernel_hop] next steps:
  - install an available TI kernel package shown above (for example 5.10-ti or 6.1-ti)
  - confirm /boot/vmlinuz-<uname_r> exists
  - rerun --plan, then --apply, reboot, and redeploy firmware
EONEXT
  fi
}

show_dmesg_summary() {
  local hints

  hints="$(dmesg -T | egrep -i 'remoteproc|rproc-virtio|pru|rpmsg|oops|segfault|bug' | tail -n 120 || true)"
  if [[ -z "${hints}" ]]; then
    return
  fi

  log "last 120 dmesg lines (remoteproc/rpmsg/pru/oops):"
  printf '%s\n' "${hints}"

  if printf '%s\n' "${hints}" | grep -Eqi 'pru_rproc_kick|internal error: oops|boot failed: -22|kick method not defined'; then
    log "detected RPMsg kick-path failure/oops signature. Kernel hop is recommended before further RPMsg traffic tests."
  fi
}

apply_plan() {
  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 0 && ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 0 ]]; then
    log "nothing to apply"
    return
  fi

  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 1 ]]; then
    kernel_installed "${PLAN_KERNEL}" || die "refusing to set uname_r=${PLAN_KERNEL}; kernel is not installed"
  fi

  if [[ ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 1 ]]; then
    validate_overlay_for_kernel "${PLAN_KERNEL}" "${PLAN_OVERLAY}" || \
      die "refusing to set uboot_overlay_pru=${PLAN_OVERLAY}; overlay not present for kernel ${PLAN_KERNEL}"
  fi

  backup_uenv

  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 1 ]]; then
    set_uenv_key "uname_r" "${PLAN_KERNEL}"
    log "updated uname_r=${PLAN_KERNEL}"
  fi

  if [[ ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 1 ]]; then
    set_uenv_key "uboot_overlay_pru" "${PLAN_OVERLAY}"
    log "updated uboot_overlay_pru=${PLAN_OVERLAY}"
  fi

  log "applied changes to ${UENV_PATH}; reboot required"
}

revert_plan() {
  local backup
  backup="$(latest_backup)"
  [[ -n "${backup}" ]] || die "no backups found matching ${BACKUP_GLOB}"

  sudo cp "${backup}" "${UENV_PATH}"
  log "restored ${UENV_PATH} from ${backup}; reboot required"
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
      --kernel)
        TARGET_KERNEL="${2:?missing value for --kernel}"
        shift 2
        ;;
      --overlay)
        TARGET_OVERLAY="${2:?missing value for --overlay}"
        shift 2
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

  if [[ "${MODE}" == "revert" ]] && [[ -n "${TARGET_KERNEL}" || -n "${TARGET_OVERLAY}" ]]; then
    die "--kernel/--overlay are not valid with --revert"
  fi
}

main() {
  parse_args "$@"
  require_uenv

  if [[ "${MODE}" == "revert" ]]; then
    revert_plan
    exit 0
  fi

  show_inventory
  build_plan
  show_plan
  show_dmesg_summary

  if [[ "${MODE}" == "apply" ]]; then
    apply_plan
  fi
}

main "$@"
