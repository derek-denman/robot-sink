#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "[install_deps] repo root: ${REPO_ROOT}"

apt_pkg_installable() {
  local pkg="$1"
  local candidate=""

  if ! apt-cache show "${pkg}" >/dev/null 2>&1; then
    return 1
  fi

  candidate="$(apt-cache policy "${pkg}" 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"
  [[ -n "${candidate}" && "${candidate}" != "(none)" ]]
}

if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update

  if ! apt-cache show gcc-pru >/dev/null 2>&1; then
    if [[ -f /etc/bbb.io/templates/apt/beagle.list && ! -f /etc/apt/sources.list.d/beagle.list ]]; then
      echo "[install_deps] enabling bb.org apt feed from /etc/bbb.io/templates/apt/beagle.list"
      sudo install -m 0644 /etc/bbb.io/templates/apt/beagle.list /etc/apt/sources.list.d/beagle.list
      sudo apt-get update
    fi
  fi

  BASE_PKGS=(
    make
    git
    device-tree-compiler
    python3
    python3-venv
    python3-pip
    gcc-pru
    binutils-pru
    libc6-pru
    ti-pru-cgt-v2.3
  )

  PRU_SSP_ALTERNATES=(
    pru-software-support-package
    ti-pru-software-v6.3
    ti-pru-software-v6.2
    ti-pru-software-v6.1
    ti-pru-software-v6.0
    ti-pru-software-v5.9
  )

  INSTALL_PKGS=()
  for pkg in "${BASE_PKGS[@]}"; do
    if apt_pkg_installable "${pkg}"; then
      INSTALL_PKGS+=("${pkg}")
    fi
  done

  for pkg in "${PRU_SSP_ALTERNATES[@]}"; do
    if apt_pkg_installable "${pkg}"; then
      INSTALL_PKGS+=("${pkg}")
      break
    fi
  done

  if ((${#INSTALL_PKGS[@]} > 0)); then
    echo "[install_deps] installing apt packages: ${INSTALL_PKGS[*]}"
    sudo apt-get install -y "${INSTALL_PKGS[@]}"
  fi
fi

REQ="${REPO_ROOT}/beaglebone/host_daemon/requirements.txt"
VENV="${REPO_ROOT}/beaglebone/host_daemon/.venv"

echo "[install_deps] creating/updating venv: ${VENV}"
python3 -m venv "${VENV}"
"${VENV}/bin/python" -m pip install -U pip wheel
"${VENV}/bin/python" -m pip install -r "${REQ}"

if [[ -d /sys/class/remoteproc ]]; then
  echo "[install_deps] remoteproc available"
fi

if command -v pru-gcc >/dev/null 2>&1; then
  echo "[install_deps] pru-gcc found: $(command -v pru-gcc)"
else
  echo "[install_deps] WARNING: pru-gcc not found in PATH."
  echo "[install_deps] On Debian/BeagleBone, verify bb.org apt repos are enabled and install gcc-pru."
  echo "[install_deps] If apt cannot locate gcc-pru, install bbb.io-keyring + beagle.list first."
  if command -v clpru >/dev/null 2>&1; then
    echo "[install_deps] NOTE: clpru is installed. build_pru.sh can use clpru/lnkpru fallback."
  fi
fi

if command -v clpru >/dev/null 2>&1; then
  echo "[install_deps] clpru found: $(command -v clpru)"
fi
if command -v lnkpru >/dev/null 2>&1; then
  echo "[install_deps] lnkpru found: $(command -v lnkpru)"
fi

if [[ -f /usr/lib/pru-software-support-package/include/pru_cfg.h || -f /usr/lib/pru-software-support-package/include/am335x/pru_cfg.h ]]; then
  echo "[install_deps] PRU SSP headers found in /usr/lib/pru-software-support-package/include"
elif [[ -f /usr/share/pru-software-support-package/include/pru_cfg.h || -f /usr/share/pru-software-support-package/include/am335x/pru_cfg.h ]]; then
  echo "[install_deps] PRU SSP headers found in /usr/share/pru-software-support-package/include"
elif [[ -f /usr/share/ti/pru-software-support-package/include/pru_cfg.h || -f /usr/share/ti/pru-software-support-package/include/am335x/pru_cfg.h ]]; then
  echo "[install_deps] PRU SSP headers found in /usr/share/ti/pru-software-support-package/include"
else
  echo "[install_deps] WARNING: PRU SSP headers not found in standard locations."
fi

echo "[install_deps] done"
echo
echo "To run the daemon:"
echo "  cd ${REPO_ROOT}/beaglebone/host_daemon"
echo "  . .venv/bin/activate"
echo "  python bbb_base_daemon.py --config config.yaml"
