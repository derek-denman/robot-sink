#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

log() {
  echo "[${SCRIPT_NAME}] $*"
}

run_sudo() {
  if [[ ${EUID} -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

JETSON_USB_IP="${JETSON_USB_IP:-192.168.7.1}"
JETSON_USB_PREFIX="${JETSON_USB_PREFIX:-24}"
BB_USB_IP="${BB_USB_IP:-192.168.7.2}"
BB_USB_IFACE="${BB_USB_IFACE:-}"
BB_USB_ENABLE_NAT="${BB_USB_ENABLE_NAT:-0}"

find_usb_eth_candidates() {
  local iface
  for path in /sys/class/net/*; do
    iface="$(basename "${path}")"
    [[ "${iface}" == "lo" ]] && continue

    if [[ -f "${path}/type" ]] && [[ "$(cat "${path}/type")" != "1" ]]; then
      continue
    fi

    local dev_link
    dev_link="$(readlink -f "${path}/device" 2>/dev/null || true)"
    if [[ "${dev_link}" == *"/usb"* || "${iface}" == usb* || "${iface}" == enx* ]]; then
      echo "${iface}"
    fi
  done
}

choose_iface() {
  if [[ -n "${BB_USB_IFACE}" ]]; then
    if ip link show "${BB_USB_IFACE}" >/dev/null 2>&1; then
      echo "${BB_USB_IFACE}"
      return
    fi
    log "Requested BB_USB_IFACE='${BB_USB_IFACE}' does not exist."
    exit 1
  fi

  mapfile -t candidates < <(find_usb_eth_candidates)

  if [[ ${#candidates[@]} -eq 0 ]]; then
    log "No USB ethernet candidate interfaces found."
    log "Connect the BeagleBone USB cable and retry."
    exit 1
  fi

  if [[ ${#candidates[@]} -eq 1 ]]; then
    echo "${candidates[0]}"
    return
  fi

  local iface
  for iface in "${candidates[@]}"; do
    if ip -4 addr show dev "${iface}" | grep -q "${JETSON_USB_IP}"; then
      echo "${iface}"
      return
    fi
  done

  for iface in "${candidates[@]}"; do
    if [[ -f "/sys/class/net/${iface}/carrier" ]] && [[ "$(cat "/sys/class/net/${iface}/carrier")" == "1" ]]; then
      echo "${iface}"
      return
    fi
  done

  log "Multiple USB ethernet candidates: ${candidates[*]}"
  log "Picking first candidate. Override with BB_USB_IFACE=<iface>."
  echo "${candidates[0]}"
}

enable_nat() {
  local bb_iface="$1"
  local uplink
  uplink="$(ip route show default | awk '/default/ {print $5; exit}')"

  if [[ -z "${uplink}" ]]; then
    log "No default route found. NAT not configured."
    return
  fi

  if [[ "${uplink}" == "${bb_iface}" ]]; then
    log "Default route uses BB interface; skipping NAT."
    return
  fi

  if ! command -v iptables >/dev/null 2>&1; then
    log "iptables not installed; cannot enable NAT."
    return
  fi

  log "Enabling IPv4 forwarding and NAT via uplink ${uplink}."
  run_sudo sysctl -w net.ipv4.ip_forward=1 >/dev/null
  run_sudo mkdir -p /etc/sysctl.d
  echo "net.ipv4.ip_forward=1" | run_sudo tee /etc/sysctl.d/99-robot-nat.conf >/dev/null

  run_sudo iptables -t nat -C POSTROUTING -s 192.168.7.0/24 -o "${uplink}" -j MASQUERADE 2>/dev/null || \
    run_sudo iptables -t nat -A POSTROUTING -s 192.168.7.0/24 -o "${uplink}" -j MASQUERADE

  run_sudo iptables -C FORWARD -i "${uplink}" -o "${bb_iface}" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
    run_sudo iptables -A FORWARD -i "${uplink}" -o "${bb_iface}" -m state --state RELATED,ESTABLISHED -j ACCEPT

  run_sudo iptables -C FORWARD -i "${bb_iface}" -o "${uplink}" -j ACCEPT 2>/dev/null || \
    run_sudo iptables -A FORWARD -i "${bb_iface}" -o "${uplink}" -j ACCEPT

  log "NAT rules applied (non-persistent firewall backend specific)."
}

main() {
  local iface
  iface="$(choose_iface)"

  log "Using interface: ${iface}"
  run_sudo ip link set "${iface}" up
  run_sudo ip addr replace "${JETSON_USB_IP}/${JETSON_USB_PREFIX}" dev "${iface}"

  log "Assigned ${JETSON_USB_IP}/${JETSON_USB_PREFIX} on ${iface}"
  log "Expected BeagleBone address: ${BB_USB_IP}"

  if [[ "${BB_USB_ENABLE_NAT}" == "1" ]]; then
    enable_nat "${iface}"
  else
    log "NAT disabled (set BB_USB_ENABLE_NAT=1 to enable)."
  fi

  if ping -c 2 -W 1 "${BB_USB_IP}" >/dev/null 2>&1; then
    log "Connectivity OK: BeagleBone responded at ${BB_USB_IP}."
  else
    log "BeagleBone did not respond at ${BB_USB_IP} yet."
  fi
}

main "$@"
