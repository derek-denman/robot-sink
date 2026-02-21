# 00 - Overview

## Target Hardware (Exact Kit)

This bring-up assumes exactly:

- Waveshare Jetson Orin NX AI Dev Kit (`JETSON-ORIN-IO-BASE-B` family)
- Jetson Orin NX 16GB module
- 256GB NVMe SSD included with the kit
- AW-CB375NF Wi-Fi/Bluetooth card with external antennas

Important:

- The Orin NX module in this kit has no built-in storage.
- Boot/rootfs is expected on the included NVMe SSD.

## Runtime Architecture

- Jetson: perception, task logic, arm comms, ROS 2 runtime
- BeagleBone: deterministic base control (motor/encoder/safety daemon)
- Jetson <-> BeagleBone primary link: USB networking (`192.168.7.1` <-> `192.168.7.2`)
- Jetson <-> BeagleBone fallback link: USB serial

## Jetson USB Peripherals

- OAK-D Lite over USB3
- RPLIDAR A1 over USB-serial (`/dev/ttyRPLIDAR` via udev)
- RoArm-M2-S over USB-serial (`/dev/ttyROARM` via udev)

## Software Baseline

- Preferred OS: JetPack 6.x (Ubuntu 22.04 base)
- ROS 2: Humble
- Optional: Docker engine

If the board is on JetPack 5.x / Ubuntu 20.04, ROS 2 Humble apt binaries are not the standard path. See `07_ros2_workspace.md` for options.
