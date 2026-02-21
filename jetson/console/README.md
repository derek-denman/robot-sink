# Robot Console (Jetson)

The Robot Console is a Jetson-hosted web UI + ROS 2 backend for:

- Mode switching and safety gating
- Commissioning checks
- Manual base/arm control
- Mapping/localization actions
- Navigation goals/recovery
- Pick/place and demo workflow controls
- Training bag recording and reliability telemetry

## Start

From the repository root on Jetson:

```bash
source /opt/ros/humble/setup.bash
cd ~/robot-sink/ros_ws
colcon build --symlink-install
source install/setup.bash
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

The stack starts `foxglove_bridge` and `robot_console` automatically.

## Open UI

Open from any machine on the same network:

```text
http://<jetson>:8080
```

Default port is configured in `jetson/console/console_config.yaml`.

## Foxglove Connection

Bridge endpoint:

```text
ws://<jetson>:8765
```

Use Foxglove Web or desktop and connect to the bridge URL above.

Recommended layout seed:

- `jetson/console/layouts/foxglove_layout.json`

## Configuration

Edit `jetson/console/console_config.yaml` for:

- HTTP host/port
- Foxglove port
- ROS topics
- Bag storage path
- Optional stack start/stop commands

## Troubleshooting

- Check stack logs under `jetson/logs/`:
  - `jetson/logs/robot-console.log`
  - `jetson/logs/foxglove.log`
- Check runtime service logs:

```bash
sudo journalctl -u robot-jetson-stack.service -f
```

- If UI loads but actions fail, verify the ROS graph is available and `ros_ws/install/setup.bash` is sourced in `run_stack.sh`.
- If `foxglove_bridge` is missing, install package dependencies and rebuild.

## How To Test

1. Start stack:

```bash
./jetson/scripts/run_stack.sh
```

2. Open UI:

```text
http://<jetson>:8080
```

3. Verify telemetry updates:
- Top status bar and dashboard tiles should update continuously.

4. Manual base command test:
- Set mode to `Manual`, hold deadman, press teleop button (or WASD).
- Confirm `cmd_vel` is seen by downstream base controller path.

5. Foxglove bridge test:
- Connect Foxglove client to `ws://<jetson>:8765`.
- Verify map/scan/TF/image topics are visible.

6. Rosbag start/stop test:
- Start recording from Dashboard or Training tab.
- Stop recording and verify a new bag directory under configured bag storage path.
