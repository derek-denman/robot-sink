#!/usr/bin/env bash
set -euo pipefail

UENV_PATH="/boot/uEnv.txt"
BACKUP_GLOB="/boot/uEnv.txt.robot-sink.bak-*"

MODE=""

usage() {
  cat <<'USAGE'
Usage:
  ./beaglebone/scripts/pru_rpmsg_fixup.sh --plan
  ./beaglebone/scripts/pru_rpmsg_fixup.sh --apply
  ./beaglebone/scripts/pru_rpmsg_fixup.sh --revert

Modes:
  --plan    Print kernel/overlay inventory and what would change.
  --apply   Backup /boot/uEnv.txt and apply planned safe changes.
  --revert  Restore the most recent /boot/uEnv.txt backup created by this script.
USAGE
}

log() {
  echo "[pru_rpmsg_fixup] $*"
}

die() {
  echo "[pru_rpmsg_fixup] ERROR: $*" >&2
  exit 1
}

require_uenv() {
  [[ -f "${UENV_PATH}" ]] || die "${UENV_PATH} not found"
}

get_uenv_key() {
  local key="$1"
  local value=""

  if [[ -r "${UENV_PATH}" ]]; then
    value="$(grep -E "^[[:space:]]*${key}=" "${UENV_PATH}" | tail -n 1 | cut -d= -f2- || true)"
  fi

  printf '%s' "${value}"
}

list_installed_kernels() {
  local k

  for k in /boot/vmlinuz-*; do
    [[ -e "${k}" ]] || continue
    basename "${k}" | sed 's/^vmlinuz-//'
  done
}

overlay_dir_for_kernel() {
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
  local overlay_dir="$2"

  if [[ -z "${overlay_dir}" ]]; then
    return 0
  fi

  find "${overlay_dir}" -maxdepth 1 -type f -name 'AM335X-PRU-*.dtbo' -print \
    | sed 's|.*/||' \
    | LC_ALL=C sort
}

pick_best_rproc_overlay() {
  local kernel="$1"
  local overlay_dir="$2"
  local kernel_tag
  local name
  local -a rproc=()

  if [[ -z "${overlay_dir}" ]]; then
    return 1
  fi

  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    if [[ "${name}" == *"RPROC"* ]]; then
      rproc+=("${name}")
    fi
  done < <(list_pru_overlays_for_kernel "${kernel}" "${overlay_dir}")

  if [[ ${#rproc[@]} -eq 0 ]]; then
    return 1
  fi

  kernel_tag="$(printf '%s' "${kernel}" | sed -E 's/^([0-9]+\.[0-9]+).*/\1/' | tr '.' '-')"
  for name in "${rproc[@]}"; do
    if [[ "${name}" == *"${kernel_tag}"* ]]; then
      printf '%s\n' "${name}"
      return 0
    fi
  done

  printf '%s\n' "${rproc[0]}"
}

latest_backup_path() {
  ls -1t ${BACKUP_GLOB} 2>/dev/null | head -n 1 || true
}

backup_uenv() {
  local stamp backup_path

  stamp="$(date +%Y%m%d-%H%M%S)"
  backup_path="/boot/uEnv.txt.robot-sink.bak-${stamp}"

  sudo cp "${UENV_PATH}" "${backup_path}"
  log "created backup ${backup_path}"
}

apply_uenv_value() {
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

show_dmesg_hint() {
  local hints

  if ! command -v dmesg >/dev/null 2>&1; then
    return
  fi

  hints="$(dmesg -T | egrep -i 'remoteproc|rproc-virtio|pru|rpmsg' | tail -n 50 || true)"
  if [[ -z "${hints}" ]]; then
    return
  fi

  log "last 50 dmesg lines (remoteproc/rproc-virtio/pru/rpmsg):"
  printf '%s\n' "${hints}"

  if printf '%s\n' "${hints}" | grep -Eqi 'kick method not defined|boot failed: -22|failed to probe subdevices.*-22'; then
    log "detected .kick / -22 failure pattern. Kernel/overlay change is required for RPMsg remoteproc firmware."
  fi
}

PLAN_REASON=""
PLAN_TARGET_KERNEL=""
PLAN_TARGET_OVERLAY=""
PLAN_NEEDS_UNAME_CHANGE=0
PLAN_NEEDS_OVERLAY_CHANGE=0

build_plan() {
  local running_kernel selected_kernel selected_overlay current_kernel
  local overlay_dir current_best_rproc candidate_k candidate_dir candidate_overlay
  local -a installed_kernels=()

  running_kernel="$(uname -r)"
  selected_kernel="$(get_uenv_key uname_r)"
  selected_overlay="$(get_uenv_key uboot_overlay_pru)"

  while IFS= read -r candidate_k; do
    [[ -n "${candidate_k}" ]] || continue
    installed_kernels+=("${candidate_k}")
  done < <(list_installed_kernels)

  PLAN_REASON="No change required."
  PLAN_TARGET_KERNEL="${selected_kernel:-${running_kernel}}"
  PLAN_TARGET_OVERLAY="${selected_overlay}"
  PLAN_NEEDS_UNAME_CHANGE=0
  PLAN_NEEDS_OVERLAY_CHANGE=0

  current_kernel="${selected_kernel:-${running_kernel}}"
  overlay_dir="$(overlay_dir_for_kernel "${current_kernel}" || true)"
  current_best_rproc="$(pick_best_rproc_overlay "${current_kernel}" "${overlay_dir}" || true)"

  if [[ "${selected_overlay}" == *"UIO"* ]]; then
    if [[ -n "${current_best_rproc}" ]]; then
      PLAN_REASON="Current PRU overlay is UIO; switch to RPROC for RPMsg firmware."
      PLAN_TARGET_OVERLAY="${current_best_rproc}"
      PLAN_NEEDS_OVERLAY_CHANGE=1
      return
    fi
  fi

  if [[ -z "${current_best_rproc}" ]]; then
    for candidate_k in "${installed_kernels[@]}"; do
      candidate_dir="$(overlay_dir_for_kernel "${candidate_k}" || true)"
      candidate_overlay="$(pick_best_rproc_overlay "${candidate_k}" "${candidate_dir}" || true)"
      if [[ -n "${candidate_overlay}" ]]; then
        PLAN_REASON="Current kernel has no PRU RPROC overlay; switch to installed compatible kernel+overlay."
        PLAN_TARGET_KERNEL="${candidate_k}"
        PLAN_TARGET_OVERLAY="${candidate_overlay}"
        if [[ "${candidate_k}" != "${current_kernel}" ]]; then
          PLAN_NEEDS_UNAME_CHANGE=1
        fi
        PLAN_NEEDS_OVERLAY_CHANGE=1
        return
      fi
    done

    PLAN_REASON="No installed kernel provides an AM335X-PRU-RPROC overlay."
    PLAN_TARGET_KERNEL="${current_kernel}"
    PLAN_TARGET_OVERLAY="${selected_overlay}"
    PLAN_NEEDS_UNAME_CHANGE=0
    PLAN_NEEDS_OVERLAY_CHANGE=0
  fi
}

print_overlay_inventory_for_kernel() {
  local kernel="$1"
  local overlay_dir

  overlay_dir="$(overlay_dir_for_kernel "${kernel}" || true)"
  if [[ -n "${overlay_dir}" ]]; then
    log "  overlays dir: ${overlay_dir}"
    list_pru_overlays_for_kernel "${kernel}" "${overlay_dir}" | sed 's/^/  - /'
  else
    log "  overlays dir not found for ${kernel}"
  fi
}

print_inventory() {
  local running_kernel selected_kernel selected_overlay current_kernel
  local kernel ov_dir

  running_kernel="$(uname -r)"
  selected_kernel="$(get_uenv_key uname_r)"
  selected_overlay="$(get_uenv_key uboot_overlay_pru)"
  current_kernel="${selected_kernel:-${running_kernel}}"

  log "running kernel: ${running_kernel}"
  log "uEnv uname_r: ${selected_kernel:-<unset>}"
  log "uEnv uboot_overlay_pru: ${selected_overlay:-<unset>}"

  log "installed kernels from /boot/vmlinuz-*"
  list_installed_kernels | sed 's/^/  - /'

  log "PRU overlays for running kernel (${running_kernel}):"
  print_overlay_inventory_for_kernel "${running_kernel}"
  if [[ "${current_kernel}" != "${running_kernel}" ]]; then
    log "PRU overlays for current selected kernel (${current_kernel}):"
    print_overlay_inventory_for_kernel "${current_kernel}"
  fi

  log "PRU RPROC overlay availability by installed kernel:"
  while IFS= read -r kernel; do
    [[ -n "${kernel}" ]] || continue
    ov_dir="$(overlay_dir_for_kernel "${kernel}" || true)"
    if [[ -z "${ov_dir}" ]]; then
      echo "  - ${kernel}: overlays dir missing"
      continue
    fi

    if pick_best_rproc_overlay "${kernel}" "${ov_dir}" >/dev/null; then
      echo "  - ${kernel}: RPROC overlay available"
    else
      echo "  - ${kernel}: no RPROC overlay"
    fi
  done < <(list_installed_kernels)
}

print_plan() {
  log "plan: ${PLAN_REASON}"

  if [[ ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 1 ]]; then
    log "planned uboot_overlay_pru=${PLAN_TARGET_OVERLAY}"
  else
    log "planned uboot_overlay_pru: no change"
  fi

  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 1 ]]; then
    log "planned uname_r=${PLAN_TARGET_KERNEL}"
  else
    log "planned uname_r: no change"
  fi

  if [[ "${PLAN_REASON}" == "No installed kernel provides an AM335X-PRU-RPROC overlay." ]]; then
    cat <<'EOT'
[pru_rpmsg_fixup] next steps:
  - install a compatible TI kernel series (commonly 5.10-ti) that ships AM335X-PRU-RPROC overlays
  - confirm the kernel is present under /boot/vmlinuz-*
  - rerun: ./beaglebone/scripts/pru_rpmsg_fixup.sh --plan
EOT
  fi
}

apply_plan() {
  local current_kernel

  current_kernel="$(get_uenv_key uname_r)"
  current_kernel="${current_kernel:-$(uname -r)}"

  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 1 ]]; then
    if [[ ! -f "/boot/vmlinuz-${PLAN_TARGET_KERNEL}" ]]; then
      die "refusing to set uname_r=${PLAN_TARGET_KERNEL}; /boot/vmlinuz-${PLAN_TARGET_KERNEL} not found"
    fi
  fi

  if [[ ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 0 && ${PLAN_NEEDS_UNAME_CHANGE} -eq 0 ]]; then
    log "nothing to apply"
    return
  fi

  backup_uenv

  if [[ ${PLAN_NEEDS_UNAME_CHANGE} -eq 1 ]]; then
    apply_uenv_value "uname_r" "${PLAN_TARGET_KERNEL}"
    log "updated uname_r=${PLAN_TARGET_KERNEL}"
  fi

  if [[ ${PLAN_NEEDS_OVERLAY_CHANGE} -eq 1 ]]; then
    apply_uenv_value "uboot_overlay_pru" "${PLAN_TARGET_OVERLAY}"
    log "updated uboot_overlay_pru=${PLAN_TARGET_OVERLAY}"
  fi

  log "applied changes to ${UENV_PATH}; reboot required before redeploying firmware"
}

revert_last_backup() {
  local backup

  backup="$(latest_backup_path)"
  [[ -n "${backup}" ]] || die "no backups found matching ${BACKUP_GLOB}"

  sudo cp "${backup}" "${UENV_PATH}"
  log "restored ${UENV_PATH} from ${backup}"
  log "reboot required"
}

parse_args() {
  if [[ $# -ne 1 ]]; then
    usage
    exit 1
  fi

  case "$1" in
    --plan)
      MODE="plan"
      ;;
    --apply)
      MODE="apply"
      ;;
    --revert)
      MODE="revert"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main() {
  parse_args "$@"
  require_uenv

  if [[ "${MODE}" == "revert" ]]; then
    revert_last_backup
    exit 0
  fi

  print_inventory
  build_plan
  print_plan
  show_dmesg_hint

  if [[ "${MODE}" == "apply" ]]; then
    apply_plan
  fi
}

main "$@"
