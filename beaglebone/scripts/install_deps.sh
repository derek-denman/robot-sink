#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "[install_deps] repo root: ${REPO_ROOT}"

if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update

  CANDIDATE_PKGS=(
    make
    python3
    python3-pip
    python3-venv
    gcc-pru
    binutils-pru
    libc6-pru
    device-tree-compiler
    git
  )

  INSTALL_PKGS=()
  for pkg in "${CANDIDATE_PKGS[@]}"; do
    if apt-cache show "${pkg}" >/dev/null 2>&1; then
      INSTALL_PKGS+=("${pkg}")
    fi
  done

  if ((${#INSTALL_PKGS[@]} > 0)); then
    sudo apt-get install -y "${INSTALL_PKGS[@]}"
  fi
fi

python3 -m pip install --user -r "${REPO_ROOT}/beaglebone/host_daemon/requirements.txt"

if [[ -d /sys/class/remoteproc ]]; then
  echo "[install_deps] remoteproc available"
fi

echo "[install_deps] done"
