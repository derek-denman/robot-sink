# 08 - Run Robot Stack

## Manual Bench Run

On Jetson:

```bash
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

Default behavior:

- Configures BB USB networking (`192.168.7.1/24`)
- Starts OAK-D, RPLIDAR, RoArm, task-stack, and BB bridge processes
- Uses real ROS nodes if installed, otherwise starts stubs
- Writes logs to `jetson/logs/`

## Override Launch Commands

Use env vars to replace defaults with your real packages/launch files:

```bash
OAK_LAUNCH_CMD='ros2 launch depthai_ros_driver camera.launch.py' \
RPLIDAR_LAUNCH_CMD='ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyRPLIDAR' \
ROARM_CMD='ros2 run roarm_driver roarm_serial_node --ros-args -p port:=/dev/ttyROARM' \
TASK_STACK_CMD='ros2 launch task_manager bench_demo.launch.py' \
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
