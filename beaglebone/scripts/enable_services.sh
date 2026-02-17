#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SERVICE_SRC="${REPO_ROOT}/beaglebone/host_daemon/systemd/bbb-base-daemon.service"
SERVICE_DST="/etc/systemd/system/bbb-base-daemon.service"

TMP_SERVICE="$(mktemp)"
sed "s|@REPO_ROOT@|${REPO_ROOT}|g" "${SERVICE_SRC}" > "${TMP_SERVICE}"
sudo install -m 0644 "${TMP_SERVICE}" "${SERVICE_DST}"
rm -f "${TMP_SERVICE}"
sudo systemctl daemon-reload
sudo systemctl enable bbb-base-daemon.service
sudo systemctl restart bbb-base-daemon.service

echo "bbb-base-daemon.service enabled and restarted"
