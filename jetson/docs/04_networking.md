# 04 - Networking (Jetson <-> BeagleBone over USB)

Primary link target:

- Jetson USB-ethernet interface: `192.168.7.1/24`
- BeagleBone USB side: `192.168.7.2/24`

The BeagleBone side is already set to `192.168.7.2` in this project.

## Configure with Script

On Jetson:

```bash
cd ~/robot-sink
./jetson/scripts/setup_bb_usb_network.sh
```

With NAT enabled (so BeagleBone can egress internet through Jetson uplink):

```bash
BB_USB_ENABLE_NAT=1 ./jetson/scripts/setup_bb_usb_network.sh
```

If auto-detection chooses the wrong interface:

```bash
BB_USB_IFACE=<iface> ./jetson/scripts/setup_bb_usb_network.sh
```

## What the Script Does

1. Detects USB ethernet candidate interface(s)
2. Brings interface up
3. Assigns `192.168.7.1/24`
4. Optionally enables IPv4 forwarding + NAT rules
5. Pings `192.168.7.2`

## Manual Debug Commands

Find interfaces:

```bash
ip link
ip -4 addr
dmesg -T | tail -n 100
```

If NetworkManager is present:

```bash
nmcli device status
nmcli -f GENERAL.DEVICE,GENERAL.TYPE,GENERAL.STATE,IP4.ADDRESS dev show
```

Check BeagleBone reachability:

```bash
ping -c 3 192.168.7.2
ssh debian@192.168.7.2
```

## Fallback Transport

If USB networking is unavailable, use USB serial for base-controller comms until network gadget path is fixed.
