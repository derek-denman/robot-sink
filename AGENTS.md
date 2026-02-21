# AGENTS: Start Here

## Architecture Overview (10 lines)
1. Jetson is the main compute node for sensors, autonomy, and task logic.
2. OAK-D Lite is connected to Jetson over USB 3.x for RGB-D perception.
3. RPLIDAR A1 is connected to Jetson over USB serial.
4. RoArm-M2-S is connected to Jetson over USB serial.
5. BeagleBone is the deterministic base controller for low-level drivetrain I/O.
6. PRU0 on BeagleBone handles quadrature encoder counting.
7. PRU1 on BeagleBone handles Sabertooth serial motor control.
8. Sabertooth 2x60 control path is serial to S1 plus a dedicated stop line to S2.
9. Safety gating is fail-safe: boot STOP, explicit ARM required, watchdog stop on timeout.
10. Jetson and BeagleBone communicate primarily over USB networking (`192.168.7.x`).

## Repo Map
- `beaglebone/`: base-controller stack (`docs/`, `scripts/`, `pru_fw/`, `host_daemon/`, `tools/`, `gui/`).
- `jetson/`: Jetson bring-up/runtime (`docs/`, `scripts/`, `udev/`, `systemd/`).
- `ros_ws/src/`: robot ROS workspace (`base_bringup`, `base_description`, `base_hardware`, `nav2_config`, `perception`, `manipulation`, `task_manager`).
- `docs/`: top-level project docs (`bringup.md`, `calibration.md`, `safety.md`, `demo_runbook.md`, `jetson.md`).
- `journal/`: dated engineering notes (`journal/YYYY-MM-DD.md`).
- `sync_bb.sh`: repo-root helper to sync BeagleBone payload over SSH/rsync.
- `hardware/`, `bom/`: hardware assets and bill-of-materials references.

## Golden Commands
### Sync BeagleBone payload (default: only `beaglebone/`)
```bash
./sync_bb.sh
```
```bash
./sync_bb.sh --host 192.168.7.2 --user debian --dest ~/robot-sink
./sync_bb.sh --all
./sync_bb.sh --dry-run
```

### Sync Jetson payload
```bash
./jetson/scripts/sync_to_jetson.sh ubuntu@JETSON_IP:/home/ubuntu/robot-sink
```
```bash
# Uses exclude list at jetson/scripts/sync_to_jetson.exclude
./jetson/scripts/sync_to_jetson.sh --dry-run ubuntu@JETSON_IP:/home/ubuntu/robot-sink
```

### Jetson run
```bash
./jetson/scripts/run_stack.sh
```

### BeagleBone build/deploy
```bash
./beaglebone/scripts/install_deps.sh
./beaglebone/scripts/build_pru.sh
./beaglebone/scripts/deploy_firmware.sh
./beaglebone/scripts/enable_services.sh
```

## How Future Agents Should Work Here
- Always use a feature branch; do not work directly on shared branches.
- Prefer minimal diffs and surgical patches over broad rewrites.
- Do not paste huge logs; capture the last ~80 lines and point to file paths.
- Update docs whenever behavior, scripts, or expected outputs change.
- Include explicit "How to test" steps in PR descriptions.

## More Agent Docs
- `docs/agents/INDEX.md`
