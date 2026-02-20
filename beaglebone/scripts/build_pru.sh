#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

find_pru_gcc() {
  local candidates=()

  if [[ -n "${PRU_CC:-}" ]]; then
    candidates+=("${PRU_CC}")
  fi

  candidates+=(
    pru-gcc
    /usr/bin/pru-gcc
    /usr/local/bin/pru-gcc
    pru-gcc-13
    pru-gcc-12
    pru-gcc-11
    pru-gcc-10
  )

  local cc
  for cc in "${candidates[@]}"; do
    if command -v "${cc}" >/dev/null 2>&1; then
      command -v "${cc}"
      return 0
    fi
  done

  return 1
}

find_clpru() {
  local candidates=()

  if [[ -n "${PRU_CLPRU:-}" ]]; then
    candidates+=("${PRU_CLPRU}")
  fi

  candidates+=(
    clpru
    /usr/bin/clpru
    /usr/local/bin/clpru
    /usr/share/ti/cgt-pru/bin/clpru
    /usr/share/ti/ti-cgt-pru_2.3.3/bin/clpru
    /opt/ti/cgt-pru/bin/clpru
  )

  local cc
  for cc in "${candidates[@]}"; do
    if command -v "${cc}" >/dev/null 2>&1; then
      command -v "${cc}"
      return 0
    fi
  done

  return 1
}

find_lnkpru() {
  local candidates=()

  if [[ -n "${PRU_LNKPRU:-}" ]]; then
    candidates+=("${PRU_LNKPRU}")
  fi

  candidates+=(
    lnkpru
    /usr/bin/lnkpru
    /usr/local/bin/lnkpru
    /usr/share/ti/cgt-pru/bin/lnkpru
    /usr/share/ti/ti-cgt-pru_2.3.3/bin/lnkpru
    /opt/ti/cgt-pru/bin/lnkpru
  )

  local linker
  for linker in "${candidates[@]}"; do
    if command -v "${linker}" >/dev/null 2>&1; then
      command -v "${linker}"
      return 0
    fi
  done

  return 1
}

find_pru_ssp() {
  local candidates=()
  local home_dir="${HOME:-}"

  if [[ -n "${PRU_SSP:-}" ]]; then
    candidates+=("${PRU_SSP}")
  fi

  if [[ -n "${home_dir}" ]]; then
    candidates+=(
      "${home_dir}/pru-software-support-package"
      "${home_dir}/src/pru-software-support-package"
      "${home_dir}/git/pru-software-support-package"
    )
  fi

  candidates+=(
    /usr/lib/pru-software-support-package
    /usr/share/pru-software-support-package
    /usr/share/ti/pru-software-support-package
    /opt/ti/pru-software-support-package
    /opt/pru-software-support-package
  )

  local path
  for path in "${candidates[@]}"; do
    if [[ -f "${path}/include/pru_cfg.h" || -f "${path}/include/am335x/pru_cfg.h" ]]; then
      printf '%s\n' "${path}"
      return 0
    fi
  done

  return 1
}

find_pru_cmd_file() {
  local ssp="$1"
  local candidates=()

  if [[ -n "${PRU_CMD_FILE:-}" ]]; then
    candidates+=("${PRU_CMD_FILE}")
  fi

  candidates+=(
    "${ssp}/include/am335x-pru.cmd"
    "${ssp}/include/AM335x_PRU.cmd"
    "${ssp}/labs/lab_2/AM335x_PRU.cmd"
    "${ssp}/examples/am335x/AM335x_PRU.cmd"
  )

  local cmd
  for cmd in "${candidates[@]}"; do
    if [[ -f "${cmd}" ]]; then
      printf '%s\n' "${cmd}"
      return 0
    fi
  done

  return 1
}

find_ti_cgt_libdir() {
  local clpru_bin="$1"
  local candidates=()

  if [[ -n "${PRU_TI_CGT_LIBDIR:-}" ]]; then
    candidates+=("${PRU_TI_CGT_LIBDIR}")
  fi

  if [[ -n "${clpru_bin}" ]]; then
    local clpru_dir
    clpru_dir="$(cd "$(dirname "${clpru_bin}")" && pwd)"
    candidates+=("${clpru_dir}/../lib")
  fi

  candidates+=(
    /usr/share/ti/cgt-pru/lib
    /usr/share/ti/ti-cgt-pru_2.3.3/lib
    /opt/ti/cgt-pru/lib
  )

  local dir
  for dir in "${candidates[@]}"; do
    if [[ -f "${dir}/libc.a" ]]; then
      printf '%s\n' "${dir}"
      return 0
    fi
  done

  return 1
}

find_pru_rpmsg_lib() {
  local ssp="$1"
  local candidates=()

  if [[ -n "${PRU_TI_RPMSG_LIB:-}" ]]; then
    candidates+=("${PRU_TI_RPMSG_LIB}")
  fi

  candidates+=(
    "${ssp}/lib/rpmsg_lib.lib"
    "${ssp}/lib/pru_rpmsg.lib"
    "${ssp}/lib/pru_rpmsg_lib.lib"
    "${ssp}/libs/rpmsg_lib.lib"
  )

  local lib
  for lib in "${candidates[@]}"; do
    if [[ -f "${lib}" ]]; then
      printf '%s\n' "${lib}"
      return 0
    fi
  done

  lib="$(
    find "${ssp}" -maxdepth 6 -type f \
      \( -name 'rpmsg_lib.lib' -o -name 'pru_rpmsg.lib' -o -name 'pru_rpmsg_lib.lib' \) \
      | head -n 1
  )"

  if [[ -n "${lib}" ]]; then
    printf '%s\n' "${lib}"
    return 0
  fi

  return 1
}

find_pru_rpmsg_src() {
  local ssp="$1"
  local candidates=()

  if [[ -n "${PRU_TI_RPMSG_SRC:-}" ]]; then
    candidates+=("${PRU_TI_RPMSG_SRC}")
  fi

  candidates+=(
    "${ssp}/src/pru_rpmsg.c"
    "${ssp}/lib/src/pru_rpmsg.c"
    "${ssp}/examples/am335x/PRU_RPMsg_Echo_Interrupt0/pru_rpmsg.c"
  )

  local src
  for src in "${candidates[@]}"; do
    if [[ -f "${src}" ]]; then
      printf '%s\n' "${src}"
      return 0
    fi
  done

  src="$(
    find "${ssp}" -maxdepth 8 -type f -name 'pru_rpmsg.c' \
      | head -n 1
  )"

  if [[ -n "${src}" ]]; then
    printf '%s\n' "${src}"
    return 0
  fi

  return 1
}

find_pru_virtqueue_src() {
  local ssp="$1"
  local candidates=()

  if [[ -n "${PRU_TI_VIRTQUEUE_SRC:-}" ]]; then
    candidates+=("${PRU_TI_VIRTQUEUE_SRC}")
  fi

  candidates+=(
    "${ssp}/lib/src/rpmsg_lib/pru_virtqueue.c"
    "${ssp}/lib/src/virtqueue/pru_virtqueue.c"
    "${ssp}/lib/src/virtio_ring/pru_virtqueue.c"
    "${ssp}/src/pru_virtqueue.c"
  )

  local src
  for src in "${candidates[@]}"; do
    if [[ -f "${src}" ]]; then
      printf '%s\n' "${src}"
      return 0
    fi
  done

  src="$(
    find "${ssp}" -maxdepth 8 -type f -name 'pru_virtqueue.c' \
      | head -n 1
  )"

  if [[ -n "${src}" ]]; then
    printf '%s\n' "${src}"
    return 0
  fi

  return 1
}

PRU_GCC_BIN="$(find_pru_gcc || true)"
PRU_CLPRU_BIN="$(find_clpru || true)"
PRU_LNKPRU_BIN="$(find_lnkpru || true)"
PRU_SSP_DIR="$(find_pru_ssp || true)"
PRU_CMD_FILE_PATH=""
PRU_TI_CGT_LIBDIR_PATH=""
PRU_TI_RPMSG_LIB_PATH=""
PRU_TI_RPMSG_SRC_PATH=""
PRU_TI_VIRTQUEUE_SRC_PATH=""

if [[ -z "${PRU_SSP_DIR}" ]]; then
  echo "[build_pru] ERROR: could not find PRU Software Support Package headers." >&2
  echo "[build_pru] Expected include/pru_cfg.h or include/am335x/pru_cfg.h under PRU_SSP." >&2
  echo "[build_pru] Install package: pru-software-support-package (or set PRU_SSP=/path/to/ssp)." >&2
  exit 1
fi

PRU_CMD_FILE_PATH="$(find_pru_cmd_file "${PRU_SSP_DIR}" || true)"
if [[ -z "${PRU_CMD_FILE_PATH}" ]]; then
  echo "[build_pru] ERROR: could not locate an AM335x PRU linker command file." >&2
  echo "[build_pru] Set PRU_CMD_FILE=/path/to/am335x-pru.cmd and retry." >&2
  exit 1
fi

echo "[build_pru] using PRU_SSP=${PRU_SSP_DIR}"
echo "[build_pru] using PRU_CMD_FILE=${PRU_CMD_FILE_PATH}"

if [[ -n "${PRU_GCC_BIN}" ]]; then
  echo "[build_pru] toolchain=gcc PRU_CC=${PRU_GCC_BIN}"
  make -C "${REPO_ROOT}/beaglebone/pru_fw" clean >/dev/null 2>&1 || true
  make -C "${REPO_ROOT}/beaglebone/pru_fw" all \
    TOOLCHAIN=gcc \
    PRU_CC="${PRU_GCC_BIN}" \
    PRU_SSP="${PRU_SSP_DIR}" \
    PRU_CMD_FILE="${PRU_CMD_FILE_PATH}"
  exit 0
fi

if [[ -n "${PRU_CLPRU_BIN}" && -n "${PRU_LNKPRU_BIN}" ]]; then
  PRU_TI_CGT_LIBDIR_PATH="$(find_ti_cgt_libdir "${PRU_CLPRU_BIN}" || true)"
  PRU_TI_RPMSG_LIB_PATH="$(find_pru_rpmsg_lib "${PRU_SSP_DIR}" || true)"
  # Prefer source pair so objects are rebuilt with matching ABI.
  PRU_TI_RPMSG_SRC_PATH="$(find_pru_rpmsg_src "${PRU_SSP_DIR}" || true)"
  if [[ -n "${PRU_TI_RPMSG_SRC_PATH}" ]]; then
    PRU_TI_VIRTQUEUE_SRC_PATH="$(find_pru_virtqueue_src "${PRU_SSP_DIR}" || true)"
  fi

  if [[ -n "${PRU_TI_RPMSG_SRC_PATH}" && -z "${PRU_TI_VIRTQUEUE_SRC_PATH}" ]]; then
    if [[ -n "${PRU_TI_RPMSG_LIB_PATH}" ]]; then
      echo "[build_pru] note: pru_rpmsg.c found but pru_virtqueue.c missing; falling back to RPMsg library"
      PRU_TI_RPMSG_SRC_PATH=""
    else
      echo "[build_pru] ERROR: found pru_rpmsg.c but missing pru_virtqueue.c." >&2
      echo "[build_pru] Set PRU_TI_VIRTQUEUE_SRC=/path/to/pru_virtqueue.c or provide PRU_TI_RPMSG_LIB." >&2
      exit 1
    fi
  fi

  if [[ -z "${PRU_TI_RPMSG_LIB_PATH}" && -z "${PRU_TI_RPMSG_SRC_PATH}" ]]; then
    echo "[build_pru] ERROR: TI toolchain found, but no RPMsg implementation found in PRU_SSP." >&2
    echo "[build_pru] Provide one of:" >&2
    echo "[build_pru]   PRU_TI_RPMSG_LIB=/path/to/rpmsg_lib.lib" >&2
    echo "[build_pru]   PRU_TI_RPMSG_SRC=/path/to/pru_rpmsg.c" >&2
    exit 1
  fi

  echo "[build_pru] toolchain=ti PRU_CLPRU=${PRU_CLPRU_BIN} PRU_LNKPRU=${PRU_LNKPRU_BIN}"
  if [[ -n "${PRU_TI_CGT_LIBDIR_PATH}" ]]; then
    echo "[build_pru] using PRU_TI_CGT_LIBDIR=${PRU_TI_CGT_LIBDIR_PATH}"
  fi
  if [[ -n "${PRU_TI_RPMSG_LIB_PATH}" ]]; then
    echo "[build_pru] using PRU_TI_RPMSG_LIB=${PRU_TI_RPMSG_LIB_PATH}"
  else
    echo "[build_pru] using PRU_TI_RPMSG_SRC=${PRU_TI_RPMSG_SRC_PATH}"
    echo "[build_pru] using PRU_TI_VIRTQUEUE_SRC=${PRU_TI_VIRTQUEUE_SRC_PATH}"
  fi

  make -C "${REPO_ROOT}/beaglebone/pru_fw" clean >/dev/null 2>&1 || true
  make -C "${REPO_ROOT}/beaglebone/pru_fw" all \
    TOOLCHAIN=ti \
    PRU_CLPRU="${PRU_CLPRU_BIN}" \
    PRU_LNKPRU="${PRU_LNKPRU_BIN}" \
    PRU_SSP="${PRU_SSP_DIR}" \
    PRU_CMD_FILE="${PRU_CMD_FILE_PATH}" \
    PRU_TI_CGT_LIBDIR="${PRU_TI_CGT_LIBDIR_PATH}" \
    PRU_TI_RPMSG_LIB="${PRU_TI_RPMSG_LIB_PATH}" \
    PRU_TI_RPMSG_SRC="${PRU_TI_RPMSG_SRC_PATH}" \
    PRU_TI_VIRTQUEUE_SRC="${PRU_TI_VIRTQUEUE_SRC_PATH}"
  exit 0
fi

echo "[build_pru] ERROR: no supported PRU compiler found." >&2
echo "[build_pru] Install gcc-pru (preferred) or ti-pru-cgt-v2.3 (provides clpru/lnkpru)." >&2
exit 1
