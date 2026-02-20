# Jetson Bring-Up (Waveshare Orin NX Kit)

This directory contains the full Jetson-side provisioning, bring-up, runtime, and service setup for the robot stack.

## Quick Start

1. Provision the Jetson kit (new hardware): `jetson/docs/01_provision_new_kit.md`
2. Flash/reflash if needed: `jetson/docs/02_flash_or_restore_os.md`
3. Complete first boot checks: `jetson/docs/03_first_boot_checklist.md`
4. Sync repo to Jetson from your dev machine:

```bash
./jetson/scripts/sync_to_jetson.sh ubuntu@JETSON_IP:/home/ubuntu/robot-sink
```

5. SSH to Jetson and run setup scripts:

```bash
cd ~/robot-sink
./jetson/scripts/jetson_first_boot.sh
./jetson/scripts/install_jetson_deps.sh
./jetson/scripts/setup_udev_rules.sh
./jetson/scripts/setup_bb_usb_network.sh
```

6. Run the stack manually:

```bash
./jetson/scripts/run_stack.sh
```

7. Optional: enable systemd auto-start:

```bash
./jetson/scripts/enable_services.sh --user ubuntu --now
```

## Documents

- `docs/00_overview.md`: architecture and assumptions for this exact kit.
- `docs/01_provision_new_kit.md`: first power-on workflow (pre-flashed and reflash paths).
- `docs/02_flash_or_restore_os.md`: repeatable flash/restore process.
- `docs/03_first_boot_checklist.md`: immediate post-boot checklist.
- `docs/04_networking.md`: Jetson <-> BeagleBone USB networking + NAT option.
- `docs/05_usb_devices_and_udev.md`: stable serial aliases for RPLIDAR/RoArm.
- `docs/06_docker_optional.md`: optional Docker install and usage.
- `docs/07_ros2_workspace.md`: ROS 2 Humble workspace setup/build.
- `docs/08_run_robot_stack.md`: bench demo run commands + service.
- `docs/09_troubleshooting.md`: common bring-up failures and fixes.

## Scripts

- `scripts/sync_to_jetson.sh`: copy only Jetson-relevant files via `rsync`.
- `scripts/jetson_first_boot.sh`: first-boot baseline config.
- `scripts/install_jetson_deps.sh`: install apt/pip dependencies and ROS 2.
- `scripts/setup_udev_rules.sh`: install/reload robot USB udev rules.
- `scripts/setup_bb_usb_network.sh`: configure USB network link to BeagleBone.
- `scripts/setup_docker_optional.sh`: optional Docker setup.
- `scripts/run_stack.sh`: start minimum bench stack (real nodes or stubs).
- `scripts/enable_services.sh`: install+enable systemd service.
