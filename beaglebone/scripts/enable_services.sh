#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SERVICE_SRC="${REPO_ROOT}/beaglebone/host_daemon/systemd/bbb-base-daemon.service"
SERVICE_DST="/etc/systemd/system/bbb-base-daemon.service"
VENV_PY="${REPO_ROOT}/beaglebone/host_daemon/.venv/bin/python"
MODE="normal"

usage() {
  cat <<'USAGE'
Usage:
  ./beaglebone/scripts/enable_services.sh [--dry-run]

Options:
  --dry-run  Start bbb-base-daemon with --dry-run so it never opens RPMsg/GPIO.
USAGE
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --dry-run)
        MODE="dry-run"
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "[enable_services] ERROR: unknown argument: $1" >&2
        usage
        exit 1
        ;;
    esac
  done
}

service_flags() {
  if [[ "${MODE}" == "dry-run" ]]; then
    printf '%s' " --dry-run"
  else
    printf '%s' ""
  fi
}

parse_args "$@"

if [[ ! -x "${VENV_PY}" ]]; then
  echo "[enable_services] ERROR: virtualenv python missing at ${VENV_PY}" >&2
  echo "[enable_services] run ./beaglebone/scripts/install_deps.sh first" >&2
  exit 1
fi

TMP_SERVICE="$(mktemp)"
FLAGS="$(service_flags)"
FLAGS_ESCAPED="$(printf '%s' "${FLAGS}" | sed -e 's/[\\/&]/\\&/g')"
sed -e "s|@REPO_ROOT@|${REPO_ROOT}|g" \
    -e "s|@DAEMON_FLAGS@|${FLAGS_ESCAPED}|g" \
    "${SERVICE_SRC}" > "${TMP_SERVICE}"
sudo install -m 0644 "${TMP_SERVICE}" "${SERVICE_DST}"
rm -f "${TMP_SERVICE}"
sudo systemctl daemon-reload
sudo systemctl enable bbb-base-daemon.service
sudo systemctl restart bbb-base-daemon.service

if [[ "${MODE}" == "dry-run" ]]; then
  echo "bbb-base-daemon.service enabled and restarted in dry-run mode"
else
  echo "bbb-base-daemon.service enabled and restarted"
fi
