# 09 - Troubleshooting

## Jetson Does Not Boot from NVMe

- Verify NVMe is physically seated.
- Confirm rootfs target during flash was NVMe.
- Reflash using `jetson/docs/02_flash_or_restore_os.md`.

## No SSH Access

```bash
sudo systemctl status ssh
hostname -I
ip -4 addr
```

If no network, connect HDMI keyboard/mouse and configure ethernet/Wi-Fi locally.

## BeagleBone USB Interface Not Found

Run:

```bash
ip link
ip -4 addr
dmesg -T | tail -n 100
nmcli device status
```

Then force interface:

```bash
BB_USB_IFACE=<iface> ./jetson/scripts/setup_bb_usb_network.sh
```

## Cannot Reach BeagleBone at 192.168.7.2

- Confirm USB cable supports data, not power-only.
- Check BB side is up and assigned `192.168.7.2/24`.
- Verify Jetson side has `192.168.7.1/24` on the correct USB interface.

## `/dev/ttyRPLIDAR` or `/dev/ttyROARM` Missing

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
./jetson/scripts/setup_udev_rules.sh
udevadm info -a -n /dev/ttyUSB0 | less
```

If multiple identical USB-UART bridges exist, refine rules using serial attribute.

## ROS 2 Humble Packages Not Installing

Check OS base:

```bash
. /etc/os-release
echo "$VERSION_CODENAME"
```

- `jammy`: Humble apt path should work.
- `focal`: use source build or Docker for Humble.

## `run_stack.sh` Exits Early

Inspect logs:

```bash
ls -lah jetson/logs
sed -n '1,160p' jetson/logs/*.log
```

Common causes:

- Missing ROS package binaries
- Missing USB device permissions
- Node command typo in environment override

## Systemd Service Fails

```bash
sudo systemctl status robot-jetson-stack.service
sudo journalctl -u robot-jetson-stack.service -n 200 --no-pager
```

Reinstall service definition:

```bash
./jetson/scripts/enable_services.sh --user ubuntu
```
