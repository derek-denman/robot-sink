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

find_remoteproc() {
  local needle="$1"
  local rp
  for rp in /sys/class/remoteproc/remoteproc*; do
    [[ -e "${rp}/name" ]] || continue
    if grep -qi "${needle}" "${rp}/name"; then
      echo "${rp}"
      return 0
    fi
  done
  return 1
}

PRU0_RP="$(find_remoteproc "pru0" || true)"
PRU1_RP="$(find_remoteproc "pru1" || true)"

if [[ -z "${PRU0_RP}" ]]; then
  PRU0_RP="/sys/class/remoteproc/remoteproc1"
fi
if [[ -z "${PRU1_RP}" ]]; then
  PRU1_RP="/sys/class/remoteproc/remoteproc2"
fi

stop_remoteproc() {
  local rp="$1"
  if [[ -e "${rp}/state" ]]; then
    echo stop | sudo tee "${rp}/state" >/dev/null || true
  fi
}

start_remoteproc() {
  local rp="$1"
  if [[ -e "${rp}/state" ]]; then
    echo start | sudo tee "${rp}/state" >/dev/null
  fi
}

stop_remoteproc "${PRU0_RP}"
stop_remoteproc "${PRU1_RP}"

sudo install -m 0644 "${PRU0_FW_SRC}" "${PRU0_FW_DST}"
sudo install -m 0644 "${PRU1_FW_SRC}" "${PRU1_FW_DST}"

start_remoteproc "${PRU0_RP}"
start_remoteproc "${PRU1_RP}"

echo "Deployed PRU firmware to ${PRU0_FW_DST} and ${PRU1_FW_DST}"
