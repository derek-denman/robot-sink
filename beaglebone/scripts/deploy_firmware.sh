#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PRU0_FW_SRC="${REPO_ROOT}/beaglebone/pru_fw/pru0_encoders/am335x-pru0-fw"
PRU1_FW_SRC="${REPO_ROOT}/beaglebone/pru_fw/pru1_sabertooth/am335x-pru1-fw"

PRU0_FW_DST="/lib/firmware/am335x-pru0-fw"
PRU1_FW_DST="/lib/firmware/am335x-pru1-fw"

if [[ ! -f "${PRU0_FW_SRC}" || ! -f "${PRU1_FW_SRC}" ]]; then
  echo "Firmware binaries missing. Run beaglebone/scripts/build_pru.sh first." >&2
  exit 1
fi

if [[ -f /boot/uEnv.txt ]] && grep -Eq '^[[:space:]]*uboot_overlay_pru=.*UIO' /boot/uEnv.txt; then
  echo "[deploy_firmware] WARNING: /boot/uEnv.txt currently selects a PRU UIO overlay."
  echo "[deploy_firmware] RPMsg firmware requires a PRU RPROC overlay/kernel combo."
fi

if command -v file >/dev/null 2>&1; then
  echo "[deploy_firmware] firmware file types:"
  file "${PRU0_FW_SRC}" || true
  file "${PRU1_FW_SRC}" || true
fi

sysfs_read() {
  local path="$1"
  if [[ -r "${path}" ]]; then
    tr -d '\n' < "${path}"
  else
    printf '%s' "n/a"
  fi
}

print_remoteproc_table() {
  local rp
  echo "[deploy_firmware] remoteproc inventory:"
  for rp in /sys/class/remoteproc/remoteproc*; do
    [[ -d "${rp}" ]] || continue
    printf "  %s name=%s firmware=%s state=%s\n" \
      "${rp}" \
      "$(sysfs_read "${rp}/name")" \
      "$(sysfs_read "${rp}/firmware")" \
      "$(sysfs_read "${rp}/state")"
  done
}

discover_pru_remoteprocs() {
  local rp
  local name
  local fw
  local -a pru_candidates=()

  PRU0_RP=""
  PRU1_RP=""

  # First pass: explicit pru0/pru1 in remoteproc name or firmware.
  for rp in /sys/class/remoteproc/remoteproc*; do
    [[ -d "${rp}" ]] || continue
    name="$(sysfs_read "${rp}/name" | tr '[:upper:]' '[:lower:]')"
    fw="$(sysfs_read "${rp}/firmware" | tr '[:upper:]' '[:lower:]')"

    if [[ -z "${PRU0_RP}" ]] && [[ "${name}" =~ pru0|pru-0|pru_0|\.0|/0|_0$ ]]; then
      PRU0_RP="${rp}"
      continue
    fi
    if [[ -z "${PRU1_RP}" ]] && [[ "${name}" =~ pru1|pru-1|pru_1|\.1|/1|_1$ ]]; then
      PRU1_RP="${rp}"
      continue
    fi
    if [[ -z "${PRU0_RP}" ]] && [[ "${fw}" == *"am335x-pru0-fw"* ]]; then
      PRU0_RP="${rp}"
      continue
    fi
    if [[ -z "${PRU1_RP}" ]] && [[ "${fw}" == *"am335x-pru1-fw"* ]]; then
      PRU1_RP="${rp}"
      continue
    fi
  done

  # Second pass: any PRU-like remoteproc names.
  for rp in /sys/class/remoteproc/remoteproc*; do
    [[ -d "${rp}" ]] || continue
    name="$(sysfs_read "${rp}/name" | tr '[:upper:]' '[:lower:]')"
    if [[ "${name}" == *"pru"* ]]; then
      pru_candidates+=("${rp}")
    fi
  done

  if [[ -z "${PRU0_RP}" ]] && [[ ${#pru_candidates[@]} -ge 1 ]]; then
    PRU0_RP="${pru_candidates[0]}"
  fi
  if [[ -z "${PRU1_RP}" ]] && [[ ${#pru_candidates[@]} -ge 2 ]]; then
    if [[ "${pru_candidates[0]}" == "${PRU0_RP}" ]]; then
      PRU1_RP="${pru_candidates[1]}"
    else
      PRU1_RP="${pru_candidates[0]}"
    fi
  fi

  if [[ -z "${PRU0_RP}" || -z "${PRU1_RP}" ]]; then
    echo "[deploy_firmware] ERROR: could not detect both PRU remoteproc devices." >&2
    print_remoteproc_table >&2
    return 1
  fi

  if [[ "${PRU0_RP}" == "${PRU1_RP}" ]]; then
    echo "[deploy_firmware] ERROR: PRU0 and PRU1 resolved to the same remoteproc (${PRU0_RP})." >&2
    print_remoteproc_table >&2
    return 1
  fi

  return 0
}

stop_remoteproc_if_running() {
  local rp="$1"
  local state
  state="$(sysfs_read "${rp}/state")"
  if [[ "${state}" == "running" ]]; then
    echo "[deploy_firmware] stopping ${rp}"
    echo stop | sudo tee "${rp}/state" >/dev/null
  else
    echo "[deploy_firmware] skip stop ${rp} (state=${state})"
  fi
}

set_remoteproc_firmware() {
  local rp="$1"
  local fw_basename="$2"
  if [[ -w "${rp}/firmware" || -e "${rp}/firmware" ]]; then
    echo "${fw_basename}" | sudo tee "${rp}/firmware" >/dev/null || true
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
    # Some kernels use "attached" for externally loaded firmware.
    echo "[deploy_firmware] detaching ${rp} (state=attached)"
    echo detach | sudo tee "${rp}/state" >/dev/null || true
  fi

  echo "[deploy_firmware] starting ${rp}"
  if ! echo start | sudo tee "${rp}/state" >/dev/null; then
    echo "[deploy_firmware] ERROR: failed to start ${rp}." >&2
    print_remoteproc_table >&2
    if command -v dmesg >/dev/null 2>&1; then
      local hints
      hints="$(dmesg -T | grep -Ei 'remoteproc|pru|firmware|resource table|elf|kick method not defined' | tail -n 80 || true)"
      echo "[deploy_firmware] recent kernel hints:" >&2
      printf '%s\n' "${hints}" >&2
      if printf '%s\n' "${hints}" | grep -q "kick method not defined"; then
        echo "[deploy_firmware] detected kernel/DT limitation: PRU remoteproc has no RPMsg kick support." >&2
        echo "[deploy_firmware] use a kernel+DT that supports PRU RPMsg, or disable RPMsg vdev for diagnostics." >&2
      fi
    fi
    return 1
  fi
}

discover_pru_remoteprocs
echo "[deploy_firmware] PRU0=${PRU0_RP}"
echo "[deploy_firmware] PRU1=${PRU1_RP}"

stop_remoteproc_if_running "${PRU0_RP}"
stop_remoteproc_if_running "${PRU1_RP}"

sudo install -m 0644 "${PRU0_FW_SRC}" "${PRU0_FW_DST}"
sudo install -m 0644 "${PRU1_FW_SRC}" "${PRU1_FW_DST}"

set_remoteproc_firmware "${PRU0_RP}" "am335x-pru0-fw"
set_remoteproc_firmware "${PRU1_RP}" "am335x-pru1-fw"

start_remoteproc "${PRU0_RP}"
start_remoteproc "${PRU1_RP}"

print_remoteproc_table

echo "Deployed PRU firmware to ${PRU0_FW_DST} and ${PRU1_FW_DST}"
