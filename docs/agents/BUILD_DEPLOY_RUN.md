# Build, Deploy, Run

Back: [Agent Index](INDEX.md) | [Root Start Here](../../AGENTS.md)

## Known-Good Bringup Order (Bench)

### 1) Jetson
```bash
cd jetson/scripts
./run_stack.sh
```

Verify serial udev aliases:
```bash
ls -l /dev/ttyRPLIDAR /dev/ttyROARM
```

Verify logs and process health:
```bash
ls -lah ../../jetson/logs
ps -ef | grep -E 'run_stack.sh|runtime_stub.py|bb_bridge_stub.py|ros2' | grep -v grep
```

### 2) BeagleBone
Install deps (creates/updates host daemon venv):
```bash
./beaglebone/scripts/install_deps.sh
```

If PRU SSP is not available from apt, set environment:
```bash
export PRU_SSP=/home/debian/pru-software-support-package
export PRU_CMD_FILE=/home/debian/pru-software-support-package/include/am335x-pru.cmd
```

Build and deploy PRU firmware:
```bash
./beaglebone/scripts/build_pru.sh
./beaglebone/scripts/deploy_firmware.sh
```

Start host daemon (manual first), then service:
```bash
python3 beaglebone/host_daemon/bbb_base_daemon.py --config beaglebone/host_daemon/config.yaml
# after manual validation
./beaglebone/scripts/enable_services.sh
```

### 3) ROS workspace
Current `ros_ws/src` directories:
- `ros_ws/src/base_bringup`
- `ros_ws/src/base_description`
- `ros_ws/src/base_hardware`
- `ros_ws/src/nav2_config`
- `ros_ws/src/perception`
- `ros_ws/src/manipulation`
- `ros_ws/src/task_manager`

Add a new package and build:
```bash
cd ros_ws/src
ros2 pkg create --build-type ament_python my_new_pkg
cd ..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Expected Outputs

### Remoteproc state checks
```bash
for rp in /sys/class/remoteproc/remoteproc*; do
  echo "$rp firmware=$(cat "$rp/firmware" 2>/dev/null) state=$(cat "$rp/state" 2>/dev/null)"
done
```
Expected: both PRU remoteprocs show firmware names `am335x-pru0-fw` / `am335x-pru1-fw` and `state=running`.

### RPMsg device nodes (if applicable)
```bash
ls -l /dev/rpmsg_pru30 /dev/rpmsg_pru31
```
Expected: nodes exist; if names differ, update `beaglebone/host_daemon/config.yaml` (`pru.encoder_rpmsg_dev`, `pru.motor_rpmsg_dev`).
