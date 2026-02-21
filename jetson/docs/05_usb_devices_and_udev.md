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

## If `/dev/ttyROARM` Is Missing

1. Check all serial candidates:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

2. Inspect IDs for each candidate:

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL'
udevadm info --query=property --name=/dev/ttyUSB1 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL'
udevadm info --query=property --name=/dev/ttyACM0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL'
```

3. If both adapters are `ID_VENDOR_ID=10c4` and `ID_MODEL_ID=ea60`, use `ID_MODEL` to split:

- `CP2102_USB_to_UART_Bridge_Controller` -> `/dev/ttyRPLIDAR`
- `CP2102N_USB_to_UART_Bridge_Controller` -> `/dev/ttyROARM`

4. If both report the same `ID_MODEL`, pin by serial:

- Add `ATTRS{serial}=="<rplidar-serial>"` to the RPLIDAR rule
- Add `ATTRS{serial}=="<roarm-serial>"` to the RoArm rule

5. Reload rules:

```bash
./jetson/scripts/setup_udev_rules.sh
```

## If Both Devices Use Same USB-UART Chipset

Refine the udev rule with serial matching:

```bash
udevadm info -a -n /dev/ttyUSBX | grep -m1 '{serial}'
```

Then add `ATTRS{serial}=="<serial-value>"` to the rule and reload.
