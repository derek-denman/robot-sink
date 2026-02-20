# 05 - USB Devices and udev Rules

This setup creates stable serial aliases:

- RPLIDAR A1 -> `/dev/ttyRPLIDAR`
- RoArm-M2-S -> `/dev/ttyROARM`

Rules file: `jetson/udev/99-robot-devices.rules`

## Install Rules

```bash
cd ~/robot-sink
./jetson/scripts/setup_udev_rules.sh
```

Replug RPLIDAR and RoArm after install.

## Verify Device IDs

```bash
lsusb
udevadm info -a -n /dev/ttyUSB0 | less
udevadm info -a -n /dev/ttyUSB1 | less
```

Look for `idVendor`, `idProduct`, and serial attributes.

## Verify Stable Aliases

```bash
ls -l /dev/ttyRPLIDAR
ls -l /dev/ttyROARM
```

Expected:

- Symlinks exist and point to active ttyUSB/ttyACM devices
- Group ownership includes `dialout`

## If Both Devices Use Same USB-UART Chipset

Refine the udev rule with serial matching:

```bash
udevadm info -a -n /dev/ttyUSBX | grep -m1 '{serial}'
```

Then add `ATTRS{serial}=="<serial-value>"` to the rule and reload.
