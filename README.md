# Living-Room Toy Tidy Robot

A compact 4WD skid-steer mobile manipulator that navigates indoor space, finds toys, and picks with a RoArm while avoiding non-target objects.

## Architecture (Current)

- Jetson Orin NX (main compute): perception, task logic, ROS 2 runtime, arm + lidar + camera drivers
- BeagleBone base controller: deterministic base I/O and safety path for drivetrain
- Primary Jetson <-> BeagleBone link: USB networking (`192.168.7.1` Jetson, `192.168.7.2` BeagleBone)
- Fallback Jetson <-> BeagleBone link: USB serial

## Hardware Baseline

Jetson side (this repo bring-up targets this exact kit):

- Waveshare Jetson Orin NX AI Dev Kit (`JETSON-ORIN-IO-BASE-B` family)
- Jetson Orin NX 16GB module
- 256GB NVMe SSD (included with kit)
- AW-CB375NF Wi-Fi/Bluetooth card + external antennas
- OAK-D Lite (USB3)
- RPLIDAR A1 (USB-serial)
- RoArm-M2-S (USB-serial)

Important:

- The Orin NX module in this kit does not provide built-in storage.
- Boot/rootfs is on NVMe.

## Software Baseline

- JetPack 6.x preferred (Ubuntu 22.04 base)
- ROS 2 Humble
- Optional Docker for containerized workflows

## Bring-Up Quick Start

### 1) Provision Jetson hardware

Follow:

- `jetson/docs/01_provision_new_kit.md`
- `jetson/docs/02_flash_or_restore_os.md`
- `jetson/docs/03_first_boot_checklist.md`

### 2) Sync repo from dev machine to Jetson

From repo root on your dev machine:

```bash
./jetson/scripts/sync_to_jetson.sh ubuntu@JETSON_IP:/home/ubuntu/robot-sink
```

Or sync both Jetson + BeagleBone in one step:

```bash
./sync_nodes.sh
```

### 3) SSH to Jetson and run bring-up scripts

```bash
ssh ubuntu@JETSON_IP
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

### 4) Build and run stack

```bash
source /opt/ros/humble/setup.bash
cd ~/robot-sink/ros_ws
colcon build --symlink-install
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

### 5) Control UI (Robot Console)

Robot Console is hosted on the Jetson and served by the `robot_console` ROS backend.

- Open from Mac/Linux/Windows browser: `http://JETSON_IP:8080`
- Telemetry WebSocket: `ws://JETSON_IP:8080/ws`
- Built-in visualizer: live lidar canvas + OAK-D MJPEG stream + FPS/age status
- Foxglove bridge: `ws://JETSON_IP:8765`
- UI source: `jetson/console/ui/` (React + TypeScript + Ant Design)
- Runtime assets served from: `jetson/console/web/`

If Foxglove Web blocks insecure websocket connections, use Foxglove Desktop or tunnel from your Mac:

```bash
ssh -N -L 8765:127.0.0.1:8765 jetson@JETSON_IP
```

Then connect Foxglove to `ws://localhost:8765`.

See `jetson/console/README.md` for controls, configuration, and troubleshooting.

### 6) Optional boot-time service

```bash
./jetson/scripts/enable_services.sh --user ubuntu --now
```

## Docs Map

Jetson bring-up docs:

- `jetson/README.md`
- `jetson/docs/00_overview.md`
- `jetson/docs/04_networking.md`
- `jetson/docs/05_usb_devices_and_udev.md`
- `jetson/docs/08_run_robot_stack.md`
- `jetson/docs/09_troubleshooting.md`

BeagleBone base-controller docs:

- `beaglebone/docs/bringup.md`
- `beaglebone/docs/wiring_and_pins.md`
- `beaglebone/docs/troubleshooting.md`

Top-level docs pointer:

- `docs/jetson.md`

## Developer Docs / Agent Docs

- `AGENTS.md` (repo start-here for future agents)
- `docs/agents/INDEX.md` (agent handbook index)
- `beaglebone/docs/` (BeagleBone bring-up and troubleshooting entry point)
- `jetson/docs/` (Jetson bring-up and runtime entry point)
- `docs/README.md` (top-level docs index)

## Repository Layout

```text
.
├── README.md
├── beaglebone/
│   ├── docs/
│   ├── host_daemon/
│   ├── pru_fw/
│   └── scripts/
├── jetson/
│   ├── README.md
│   ├── console/
│   ├── docs/
│   ├── scripts/
│   ├── systemd/
│   └── udev/
├── ros_ws/
│   ├── src/
│   └── docker/
└── docs/
    └── jetson.md
```

## Safety Notes

- Use a physical E-stop and conservative acceleration limits.
- Keep arm motion zone clear during test runs.
- Validate motor stop/fail-safe behavior before full-speed driving.
