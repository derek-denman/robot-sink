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

find_pru_objcopy() {
  local candidates=()

  if [[ -n "${PRU_OBJCOPY:-}" ]]; then
    candidates+=("${PRU_OBJCOPY}")
  fi

  candidates+=(
    pru-objcopy
    llvm-objcopy
    objcopy
  )

  local tool
  for tool in "${candidates[@]}"; do
    if command -v "${tool}" >/dev/null 2>&1; then
      command -v "${tool}"
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
    "${ssp}/examples/am335x/PRU_access_const_table/AM335x_PRU.cmd"
    "${ssp}/labs/Hands_on_Labs/lab_2/AM335x_PRU.cmd"
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

find_ti_cgt_includedir() {
  local clpru_bin="$1"
  local candidates=()

  if [[ -n "${PRU_TI_CGT_INCLUDEDIR:-}" ]]; then
    candidates+=("${PRU_TI_CGT_INCLUDEDIR}")
  fi

  if [[ -n "${clpru_bin}" ]]; then
    local clpru_dir
    clpru_dir="$(cd "$(dirname "${clpru_bin}")" && pwd)"
    candidates+=("${clpru_dir}/../include")
  fi

  candidates+=(
    /usr/share/ti/cgt-pru/include
    /usr/share/ti/ti-cgt-pru_2.3.3/include
    /opt/ti/cgt-pru/include
  )

  local dir
  for dir in "${candidates[@]}"; do
    if [[ -f "${dir}/stddef.h" ]]; then
      printf '%s\n' "${dir}"
      return 0
    fi
  done

  return 1
}

PRU_GCC_BIN="$(find_pru_gcc || true)"
PRU_CLPRU_BIN="$(find_clpru || true)"
PRU_LNKPRU_BIN="$(find_lnkpru || true)"
PRU_OBJCOPY_BIN="$(find_pru_objcopy || true)"
PRU_SSP_DIR="$(find_pru_ssp || true)"
PRU_CMD_FILE_PATH=""
PRU_TI_CGT_LIBDIR_PATH=""
PRU_TI_CGT_INCLUDEDIR_PATH=""

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

if [[ -z "${PRU_OBJCOPY_BIN}" ]]; then
  echo "[build_pru] ERROR: no objcopy-compatible tool found for generating firmware .bin files." >&2
  echo "[build_pru] Install one of: pru-objcopy, llvm-objcopy, or objcopy." >&2
  exit 1
fi

echo "[build_pru] using PRU_SSP=${PRU_SSP_DIR}"
echo "[build_pru] using PRU_CMD_FILE=${PRU_CMD_FILE_PATH}"
echo "[build_pru] using PRU_OBJCOPY=${PRU_OBJCOPY_BIN}"

if [[ -n "${PRU_CLPRU_BIN}" && -n "${PRU_LNKPRU_BIN}" ]]; then
  PRU_TI_CGT_LIBDIR_PATH="$(find_ti_cgt_libdir "${PRU_CLPRU_BIN}" || true)"
  PRU_TI_CGT_INCLUDEDIR_PATH="$(find_ti_cgt_includedir "${PRU_CLPRU_BIN}" || true)"

  echo "[build_pru] toolchain=ti PRU_CLPRU=${PRU_CLPRU_BIN} PRU_LNKPRU=${PRU_LNKPRU_BIN}"
  if [[ -n "${PRU_TI_CGT_LIBDIR_PATH}" ]]; then
    echo "[build_pru] using PRU_TI_CGT_LIBDIR=${PRU_TI_CGT_LIBDIR_PATH}"
  fi
  if [[ -n "${PRU_TI_CGT_INCLUDEDIR_PATH}" ]]; then
    echo "[build_pru] using PRU_TI_CGT_INCLUDEDIR=${PRU_TI_CGT_INCLUDEDIR_PATH}"
  else
    echo "[build_pru] WARNING: TI CGT include dir not auto-detected; set PRU_TI_CGT_INCLUDEDIR if build fails on stddef.h." >&2
  fi

  make -C "${REPO_ROOT}/beaglebone/pru_fw" clean >/dev/null 2>&1 || true
  make -C "${REPO_ROOT}/beaglebone/pru_fw" all \
    TOOLCHAIN=ti \
    PRU_CLPRU="${PRU_CLPRU_BIN}" \
    PRU_LNKPRU="${PRU_LNKPRU_BIN}" \
    PRU_OBJCOPY="${PRU_OBJCOPY_BIN}" \
    PRU_SSP="${PRU_SSP_DIR}" \
    PRU_CMD_FILE="${PRU_CMD_FILE_PATH}" \
    PRU_TI_CGT_LIBDIR="${PRU_TI_CGT_LIBDIR_PATH}" \
    PRU_TI_CGT_INCLUDEDIR="${PRU_TI_CGT_INCLUDEDIR_PATH}"
  exit 0
fi

if [[ -n "${PRU_GCC_BIN}" ]]; then
  echo "[build_pru] toolchain=gcc PRU_CC=${PRU_GCC_BIN}"
  make -C "${REPO_ROOT}/beaglebone/pru_fw" clean >/dev/null 2>&1 || true
  make -C "${REPO_ROOT}/beaglebone/pru_fw" all \
    TOOLCHAIN=gcc \
    PRU_CC="${PRU_GCC_BIN}" \
    PRU_OBJCOPY="${PRU_OBJCOPY_BIN}" \
    PRU_SSP="${PRU_SSP_DIR}" \
    PRU_CMD_FILE="${PRU_CMD_FILE_PATH}"
  exit 0
fi

echo "[build_pru] ERROR: no supported PRU compiler found." >&2
echo "[build_pru] Install gcc-pru (preferred) or ti-pru-cgt-v2.3 (provides clpru/lnkpru)." >&2
exit 1
