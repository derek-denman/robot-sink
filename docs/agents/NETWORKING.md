# Networking

Back: [Agent Index](INDEX.md) | [Root Start Here](../../AGENTS.md)

## Mac <-> BeagleBone (USB Ethernet)
Current stable path:
- BeagleBone `usb0` static address: `192.168.7.2/24`
- Persistent config: `/etc/systemd/network/usb0.network`
- `DHCPServer=on` on BeagleBone for stable host-side lease behavior

Example BeagleBone config target:
```ini
# /etc/systemd/network/usb0.network
[Match]
Name=usb0

[Network]
Address=192.168.7.2/24
DHCPServer=on
```

Apply on BeagleBone:
```bash
sudo systemctl restart systemd-networkd
ip -4 addr show usb0
```

Mac-side interface/service mapping:
```bash
networksetup -listnetworkserviceorder
```
- Map hardware port `en11` to its service name (often `TI_AM335x_BeagleBone_Black`).

Fallback (static on Mac):
- Set Mac USB interface to `192.168.7.1/24` and SSH:
```bash
ssh debian@192.168.7.2
```

## Jetson <-> BeagleBone
Canonical setup script on Jetson:
```bash
./jetson/scripts/setup_bb_usb_network.sh
```
Optional NAT through Jetson uplink:
```bash
BB_USB_ENABLE_NAT=1 ./jetson/scripts/setup_bb_usb_network.sh
```

## Internet Sharing Modes
`DHCPServer=on` on BeagleBone can conflict with host Internet Sharing DHCP behavior. Use one mode at a time:

### Mode A: SSH-reliable mode (preferred for bring-up)
- Keep BeagleBone `usb0` at `192.168.7.2/24` with `DHCPServer=on`.
- Do not enable competing host DHCP on the same USB link.
- Use direct SSH to `192.168.7.2`.

### Mode B: NAT-through-host mode
- Disable BeagleBone DHCP server on `usb0` (or otherwise avoid dual DHCP).
- Use host-provided sharing/NAT, or on Jetson use `BB_USB_ENABLE_NAT=1`.
- Re-test with `ping 192.168.7.2` and `ssh debian@192.168.7.2` after switching.
