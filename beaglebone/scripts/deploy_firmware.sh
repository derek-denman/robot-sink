#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PRU0_FW_SRC="${REPO_ROOT}/beaglebone/pru_fw/pru0_encoders/am335x-pru0-fw"
PRU1_FW_SRC="${REPO_ROOT}/beaglebone/pru_fw/pru1_sabertooth/am335x-pru1-fw"

PRU0_FW_DST="/lib/firmware/am335x-pru0-fw"
PRU1_FW_DST="/lib/firmware/am335x-pru1-fw"

DEFAULT_PRU0_RP="/sys/class/remoteproc/remoteproc1"
DEFAULT_PRU1_RP="/sys/class/remoteproc/remoteproc2"
RPROC_STOP_TIMEOUT="10s"
RPROC_START_TIMEOUT="10s"

FIXUP_SCRIPT="${REPO_ROOT}/beaglebone/scripts/pru_rpmsg_fixup.sh"
KERNEL_HOP_SCRIPT="${REPO_ROOT}/beaglebone/scripts/pru_rpmsg_kernel_hop.sh"

if [[ ! -f "${PRU0_FW_SRC}" || ! -f "${PRU1_FW_SRC}" ]]; then
  echo "[deploy_firmware] ERROR: firmware binaries are missing." >&2
  echo "[deploy_firmware] run ./beaglebone/scripts/build_pru.sh first." >&2
  exit 1
fi

sysfs_read() {
  local path="$1"
  if [[ -r "${path}" ]]; then
    tr -d '\n' < "${path}"
  else
    printf '%s' "n/a"
  fi
}

uenv_get() {
  local key="$1"
  local value=""

  if [[ -r /boot/uEnv.txt ]]; then
    value="$(grep -E "^[[:space:]]*${key}=" /boot/uEnv.txt | tail -n 1 | cut -d= -f2- || true)"
  fi

  printf '%s' "${value}"
}

print_remoteproc_table() {
  local rp

  echo "[deploy_firmware] remoteproc state table:"
  for rp in /sys/class/remoteproc/remoteproc*; do
    [[ -d "${rp}" ]] || continue
    printf "  %s name=%s firmware=%s state=%s\n" \
      "${rp}" \
      "$(sysfs_read "${rp}/name")" \
      "$(sysfs_read "${rp}/firmware")" \
      "$(sysfs_read "${rp}/state")"
  done
}

print_kernel_overlay_context() {
  local running_kernel selected_kernel selected_overlay

  running_kernel="$(uname -r)"
  selected_kernel="$(uenv_get uname_r)"
  selected_overlay="$(uenv_get uboot_overlay_pru)"

  echo "[deploy_firmware] kernel/overlay context:"
  echo "  uname -r=${running_kernel}"
  if [[ -r /boot/uEnv.txt ]]; then
    echo "  /boot/uEnv.txt uname_r=${selected_kernel:-<unset>}"
    echo "  /boot/uEnv.txt uboot_overlay_pru=${selected_overlay:-<unset>}"
  else
    echo "  /boot/uEnv.txt not readable"
  fi
}

collect_filtered_dmesg() {
  if command -v dmesg >/dev/null 2>&1; then
    dmesg -T | egrep -i 'remoteproc|rproc-virtio|pru|rpmsg|oops|segfault|bug' | tail -n 120 || true
  fi
}

print_rpmsg_nodes() {
  echo "[deploy_firmware] rpmsg device nodes:"
  ls -l /dev/rpmsg* /dev/ttyRPMSG* 2>/dev/null || echo "  none"
}

print_rpmsg_boot_failure_guidance() {
  echo "[deploy_firmware] detected PRU remoteproc RPMsg boot failure (.kick method not defined / Boot failed: -22)." >&2
  echo "[deploy_firmware] remediation options:" >&2
  if [[ -x "${FIXUP_SCRIPT}" ]]; then
    echo "  1) Inspect and plan fixups:" >&2
    echo "     ./beaglebone/scripts/pru_rpmsg_fixup.sh --plan" >&2
    echo "  2) Apply recommended uEnv/kernel overlay changes (with backup):" >&2
    echo "     ./beaglebone/scripts/pru_rpmsg_fixup.sh --apply" >&2
    echo "  3) Reboot and redeploy:" >&2
    echo "     sudo reboot" >&2
    echo "     ./beaglebone/scripts/deploy_firmware.sh" >&2
  else
    echo "  - add/run beaglebone/scripts/pru_rpmsg_fixup.sh --plan" >&2
    echo "  - switch to a kernel + PRU RPROC overlay pair known to support RPMsg kick" >&2
    echo "  - reboot, then rerun deploy_firmware.sh" >&2
  fi
}

print_rpmsg_kick_oops_guidance() {
  echo "[deploy_firmware] detected kernel Oops in pru_rproc_kick during RPMsg traffic." >&2
  echo "[deploy_firmware] remediation options:" >&2
  if [[ -x "${KERNEL_HOP_SCRIPT}" ]]; then
    echo "  1) Plan safer kernel/overlay combination:" >&2
    echo "     ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --plan" >&2
    echo "  2) Apply plan, reboot, redeploy:" >&2
    echo "     ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --apply" >&2
    echo "     sudo reboot" >&2
    echo "     ./beaglebone/scripts/deploy_firmware.sh" >&2
  fi
  if [[ -x "${FIXUP_SCRIPT}" ]]; then
    echo "  3) Fallback overlay/kernel remediation path:" >&2
    echo "     ./beaglebone/scripts/pru_rpmsg_fixup.sh --plan" >&2
    echo "     ./beaglebone/scripts/pru_rpmsg_fixup.sh --apply" >&2
  fi
}

print_failure_diagnostics() {
  local dmesg_hints="$1"

  print_remoteproc_table >&2
  print_kernel_overlay_context >&2
  print_rpmsg_nodes >&2

  if [[ -n "${dmesg_hints}" ]]; then
    echo "[deploy_firmware] recent kernel log (remoteproc/rproc-virtio/pru/rpmsg):" >&2
    printf '%s\n' "${dmesg_hints}" >&2
  else
    echo "[deploy_firmware] no filtered dmesg lines available." >&2
  fi

  if printf '%s\n' "${dmesg_hints}" | grep -Eqi 'kick method not defined|boot failed: -22|failed to probe subdevices.*-22'; then
    print_rpmsg_boot_failure_guidance
  fi

  if printf '%s\n' "${dmesg_hints}" | grep -Eqi 'pru_rproc_kick|internal error: oops|oops: [0-9]'; then
    print_rpmsg_kick_oops_guidance
  fi
}

discover_pru_remoteprocs() {
  local rp
  local name
  local fw
  local -a candidates=()

  PRU0_RP=""
  PRU1_RP=""

  if [[ -d "${DEFAULT_PRU0_RP}" && -d "${DEFAULT_PRU1_RP}" ]]; then
    PRU0_RP="${DEFAULT_PRU0_RP}"
    PRU1_RP="${DEFAULT_PRU1_RP}"
    return 0
  fi

  echo "[deploy_firmware] WARNING: remoteproc1/remoteproc2 not both present; using name-based fallback." >&2

  for rp in /sys/class/remoteproc/remoteproc*; do
    [[ -d "${rp}" ]] || continue
    name="$(sysfs_read "${rp}/name" | tr '[:upper:]' '[:lower:]')"
    fw="$(sysfs_read "${rp}/firmware" | tr '[:upper:]' '[:lower:]')"

    if [[ "${name}" == *"pru"* || "${fw}" == *"am335x-pru"* ]]; then
      candidates+=("${rp}")
    fi
  done

  if [[ ${#candidates[@]} -ge 2 ]]; then
    PRU0_RP="${candidates[0]}"
    PRU1_RP="${candidates[1]}"
    return 0
  fi

  return 1
}

stop_remoteproc_if_running() {
  local rp="$1"
  local state

  state="$(sysfs_read "${rp}/state")"
  if [[ "${state}" == "running" ]]; then
    echo "[deploy_firmware] stopping ${rp}"
    if ! write_remoteproc_state "${rp}" "stop" "${RPROC_STOP_TIMEOUT}"; then
      echo "[deploy_firmware] ERROR: timed out while stopping ${rp}." >&2
      return 1
    fi
    state="$(sysfs_read "${rp}/state")"
    if [[ "${state}" == "running" ]]; then
      echo "[deploy_firmware] ERROR: ${rp} still running after stop request." >&2
      return 1
    fi
  else
    echo "[deploy_firmware] skip stop ${rp} (state=${state})"
  fi

  return 0
}

set_remoteproc_firmware() {
  local rp="$1"
  local fw_basename="$2"

  if [[ ! -e "${rp}/firmware" ]]; then
    return
  fi

  echo "${fw_basename}" | sudo tee "${rp}/firmware" >/dev/null || true
}

write_remoteproc_state() {
  local rp="$1"
  local desired="$2"
  local timeout_value="$3"
  local state_file="${rp}/state"
  local write_cmd

  write_cmd="echo '${desired}' > '${state_file}'"
  if command -v timeout >/dev/null 2>&1; then
    timeout "${timeout_value}" sudo sh -c "${write_cmd}"
  else
    sudo sh -c "${write_cmd}"
  fi
}

start_remoteproc() {
  local rp="$1"
  local state

  state="$(sysfs_read "${rp}/state")"
  if [[ "${state}" == "running" ]]; then
    echo "[deploy_firmware] ${rp} already running"
    return 0
  fi

  if [[ "${state}" == "attached" ]]; then
    echo "[deploy_firmware] detaching ${rp} (state=attached)"
    if ! write_remoteproc_state "${rp}" "detach" "${RPROC_START_TIMEOUT}"; then
      echo "[deploy_firmware] WARNING: detach timed out for ${rp}" >&2
    fi
  fi

  echo "[deploy_firmware] starting ${rp}"
  if ! write_remoteproc_state "${rp}" "start" "${RPROC_START_TIMEOUT}"; then
    return 1
  fi

  state="$(sysfs_read "${rp}/state")"
  [[ "${state}" == "running" ]]
}

if [[ -f /boot/uEnv.txt ]] && grep -Eq '^[[:space:]]*uboot_overlay_pru=.*UIO' /boot/uEnv.txt; then
  echo "[deploy_firmware] WARNING: /boot/uEnv.txt selects a PRU UIO overlay." >&2
  echo "[deploy_firmware] RPMsg firmware expects a PRU RPROC overlay/kernel pair." >&2
fi

if command -v file >/dev/null 2>&1; then
  echo "[deploy_firmware] firmware file types:"
  file "${PRU0_FW_SRC}" || true
  file "${PRU1_FW_SRC}" || true
fi

if ! discover_pru_remoteprocs; then
  echo "[deploy_firmware] ERROR: could not resolve PRU remoteproc devices." >&2
  print_failure_diagnostics "$(collect_filtered_dmesg)"
  exit 1
fi

echo "[deploy_firmware] PRU0 remoteproc=${PRU0_RP}"
echo "[deploy_firmware] PRU1 remoteproc=${PRU1_RP}"

stop_failed=0
stop_remoteproc_if_running "${PRU0_RP}" || stop_failed=1
stop_remoteproc_if_running "${PRU1_RP}" || stop_failed=1

if [[ ${stop_failed} -ne 0 ]]; then
  echo "[deploy_firmware] ERROR: unable to stop one or more running PRU cores." >&2
  echo "[deploy_firmware] hint: this often indicates kernel-side lockup; reboot before redeploying." >&2
  print_failure_diagnostics "$(collect_filtered_dmesg)"
  exit 1
fi

sudo install -m 0644 "${PRU0_FW_SRC}" "${PRU0_FW_DST}"
sudo install -m 0644 "${PRU1_FW_SRC}" "${PRU1_FW_DST}"

set_remoteproc_firmware "${PRU0_RP}" "am335x-pru0-fw"
set_remoteproc_firmware "${PRU1_RP}" "am335x-pru1-fw"

start_failed=0
start_remoteproc "${PRU0_RP}" || start_failed=1
start_remoteproc "${PRU1_RP}" || start_failed=1

if [[ ${start_failed} -ne 0 ]]; then
  echo "[deploy_firmware] ERROR: failed to start one or more PRU remoteprocs." >&2
  print_failure_diagnostics "$(collect_filtered_dmesg)"
  exit 1
fi

print_remoteproc_table
echo "[deploy_firmware] deployed firmware to ${PRU0_FW_DST} and ${PRU1_FW_DST}"
