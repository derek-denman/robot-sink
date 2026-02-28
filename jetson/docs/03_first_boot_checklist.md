# 03 - First Boot Checklist

Run this once after first successful login.

## 1) Verify Storage and Platform

```bash
lsblk
findmnt /
uname -a
cat /etc/os-release
```

Expected:

- Root filesystem on NVMe
- Ubuntu release known (`jammy` preferred for ROS 2 Humble apt path)

## 2) Bring Network Up

Use ethernet if possible for initial package install. If headless and Wi-Fi is needed:

```bash
nmcli device status
nmcli device wifi list
nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD"
```

## 3) Enable SSH

```bash
sudo systemctl enable --now ssh
hostname -I
```

## 4) Sync Repo to Jetson (From Dev Machine)

From repo root on your dev machine:

```bash
./jetson/scripts/sync_to_jetson.sh ubuntu@JETSON_IP:/home/ubuntu/robot-sink
```

Notes:
- `sync_to_jetson.sh` skips `jetson/console/web` by default to avoid overwriting a newer web build on Jetson during backend-only syncs.
- To explicitly sync repo web assets, use:

```bash
./jetson/scripts/sync_to_jetson.sh --include-web ubuntu@JETSON_IP:/home/ubuntu/robot-sink
```

## 5) Run Jetson Bring-Up Scripts (On Jetson)

```bash
cd ~/robot-sink
./jetson/scripts/jetson_first_boot.sh
./jetson/scripts/install_jetson_deps.sh
./jetson/scripts/setup_udev_rules.sh
./jetson/scripts/setup_bb_usb_network.sh
```

Optional Docker:

```bash
./jetson/scripts/setup_docker_optional.sh
```
