# 08 - Run Robot Stack

## Manual Bench Run

On Jetson:

```bash
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

Default behavior:

- Configures BB USB networking (`192.168.7.1/24`)
- Starts `foxglove_bridge`, Robot Console backend, OAK-D, RPLIDAR, RoArm, task-stack, and BB bridge processes
- Uses real ROS nodes if installed, otherwise starts stubs
- Writes logs to `jetson/logs/`

## Robot Console Quick Start

Once the stack is running:

- Open `http://<jetson>:8080`
- WebSocket telemetry path is `ws://<jetson>:8080/ws`
- Foxglove bridge endpoint is `ws://<jetson>:8765`

Robot Console assets/config:

- Static UI: `jetson/console/web/`
- Config: `jetson/console/console_config.yaml`
- Layout seed: `jetson/console/layouts/foxglove_layout.json`

Robot Console log files:

- `jetson/logs/robot-console.log`
- `jetson/logs/foxglove.log`

### Acceptance Test Steps

1. Start stack:

```bash
./jetson/scripts/run_stack.sh
```

2. Open UI:

```text
http://<jetson>:8080
```

3. Verify telemetry websocket updates (status bar/dashboard refresh continuously).
4. Manual test: set mode `Manual`, hold deadman, publish teleop command, and verify downstream base command path receives `cmd_vel`.
5. Verify Foxglove connection to `ws://<jetson>:8765`.
6. Start/stop recording in UI and confirm bag directory created in configured bag storage path.

## Override Launch Commands

Use env vars to replace defaults with your real packages/launch files:

```bash
OAK_LAUNCH_CMD='ros2 launch depthai_ros_driver camera.launch.py' \
RPLIDAR_LAUNCH_CMD='ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyRPLIDAR' \
ROARM_CMD='ros2 run roarm_driver roarm_serial_node --ros-args -p port:=/dev/ttyROARM' \
TASK_STACK_CMD='ros2 launch task_manager bench_demo.launch.py' \
FOXGLOVE_CMD='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765' \
ROBOT_CONSOLE_CMD='ros2 launch robot_console robot_console.launch.py start_foxglove:=false' \
./jetson/scripts/run_stack.sh
```

## Enable Auto-Start at Boot

Service is disabled by default until you enable it:

```bash
cd ~/robot-sink
./jetson/scripts/enable_services.sh --user ubuntu --now
```

Service controls:

```bash
sudo systemctl status robot-jetson-stack.service
sudo journalctl -u robot-jetson-stack.service -f
sudo systemctl disable robot-jetson-stack.service
```
