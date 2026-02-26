# 08 - Run Robot Stack

## Manual Bench Run

On Jetson:

```bash
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

Default behavior:

- Attempts BB USB networking setup (continues on failure for bench use)
- Starts `foxglove_bridge`, `robot_console`, OAK-D, RPLIDAR, RoArm stub, task stub, and BB bridge
- Uses real ROS nodes when installed; falls back to stubs where needed
- Writes logs under `jetson/logs/`
- Uses OAK params from `jetson/config/depthai_camera.yaml` when present (override with `OAK_PARAMS_FILE=...`)

## Robot Console Quick Start

When stack is running:

- Console UI: `http://<jetson>:8080`
- Console websocket: `ws://<jetson>:8080/ws`
- Foxglove bridge: `ws://<jetson>:8765`

Paths:

- UI source: `jetson/console/ui/`
- Production UI assets: `jetson/console/web/`
- Console config: `jetson/console/console_config.yaml`
- Foxglove layout seed: `jetson/console/layouts/foxglove_layout.json`

Built-in Mapping / Visualization page now includes:

- 2D occupancy map (`/map`) with live `/scan` overlay in map frame
- robot pose arrow from TF chain (`map -> odom -> base_link`)
- optional Nav2 path overlay from configurable path topic
- live 3D point-cloud panel (`PointCloud2` topic selectable)

## Build Steps

Backend package:

```bash
source /opt/ros/humble/setup.bash
cd ~/robot-sink/ros_ws
colcon build --symlink-install --packages-select robot_console
```

If UI source changed, rebuild assets:

```bash
export PATH="$HOME/.local/node/current/bin:$PATH"
cd ~/robot-sink/jetson/console/ui
npm install
npm run build:web
```

## Enable SLAM/Nav2 Runtime

`run_stack.sh` does not auto-launch SLAM Toolbox/Nav2. Mapping/Nav buttons require those nodes.

Install packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

Run SLAM:

```bash
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

Run Nav2:

```bash
mkdir -p "$HOME/robot-sink/ros_ws/src/nav2_config"
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
  "$HOME/robot-sink/ros_ws/src/nav2_config/nav2_params.yaml"

source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=$HOME/robot-sink/ros_ws/src/nav2_config/nav2_params.yaml
```

Verify:

```bash
ros2 action list | grep navigate_to_pose
ros2 service list | grep -E 'slam_toolbox|clear_entirely'
```

## Acceptance Test (Jetson)

1. Start stack:

```bash
./jetson/scripts/run_stack.sh
```

Expected startup lines:

- `Starting foxglove: ...`
- `Starting robot-console: ...`
- `Stack running. Logs: ...`

2. Verify API status:

```bash
python3 - <<'PY'
import json, urllib.request
with urllib.request.urlopen('http://127.0.0.1:8080/api/status', timeout=5) as r:
    d = json.load(r)
print('mode', d['mode'])
print('scan_fps', d['visualizer']['scan']['fps'])
print('camera_fps', d['visualizer']['camera']['fps'])
print('camera_topic', d['visualizer']['camera']['selected_topic'])
print('map_topic', d['visualizer']['map']['topic'])
print('map_connected', d['visualizer']['map']['connected'])
print('pointcloud_topic', d['visualizer']['pointcloud']['selected_topic'])
print('pointcloud_connected', d['visualizer']['pointcloud']['connected'])
PY
```

Expected: mode populated, scan FPS > 0, and map/pointcloud fields returned.

3. Verify websocket telemetry updates:

```bash
python3 - <<'PY'
import asyncio, json, aiohttp
async def main():
  async with aiohttp.ClientSession() as s:
    async with s.ws_connect('ws://127.0.0.1:8080/ws', timeout=5) as ws:
      for _ in range(5):
        msg = await ws.receive(timeout=4)
        if msg.type == aiohttp.WSMsgType.TEXT:
          print(json.loads(msg.data).get('type'))
asyncio.run(main())
PY
```

Expected: includes `status`, `scan`, and when available `map`, `map_overlay`, `pointcloud`.

4. Verify mode persistence and reconnect behavior:

```bash
python3 - <<'PY'
import json, urllib.request, asyncio, aiohttp
BASE='http://127.0.0.1:8080'
def post(path,payload):
  req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=6) as r:
    return json.load(r)
def mode():
  with urllib.request.urlopen(BASE+'/api/status',timeout=6) as r:
    return json.load(r)['mode']
async def ws_once():
  async with aiohttp.ClientSession() as s:
    async with s.ws_connect('ws://127.0.0.1:8080/ws', timeout=5) as ws:
      await ws.receive(timeout=3)
post('/api/mode', {'mode':'demo'})
print('mode_before', mode())
asyncio.run(ws_once())
print('mode_after_reconnect', mode())
post('/api/mode', {'mode':'manual'})
PY
```

Expected: same mode before and after websocket reconnect.

5. Verify camera topic selection persistence:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
  req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=6) as r:
    return json.load(r)
def selected():
  with urllib.request.urlopen(BASE+'/api/status',timeout=6) as r:
    return json.load(r)['visualizer']['camera']['selected_topic']
post('/api/camera/select', {'topic':'/oak/rgb/image_raw/compressed'})
for i in range(5):
  print(i, selected())
  time.sleep(0.5)
PY
```

Expected: selected topic remains stable over repeated status refreshes.

6. Verify manual safety gating + stop_all:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
  req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=6) as r:
    return json.load(r)
post('/api/mode', {'mode':'manual'})
post('/api/arm', {'armed':False})
print('disarmed_cmd', post('/api/manual/base', {'type':'motor_bank','left':20,'right':20}))
post('/api/arm', {'armed':True})
print('armed_cmd', post('/api/manual/base', {'type':'motor_bank','left':20,'right':15}))
time.sleep(1.0)
print('timeout_cmd', post('/api/manual/base', {'type':'motor_bank','left':20,'right':15}))
print('stop_all', post('/api/stop_all', {}))
PY
```

Expected: disarmed command rejected, armed command accepted, timeout command rejected, stop_all succeeds.

7. Verify lidar/camera ROS rates:

```bash
source /opt/ros/humble/setup.bash
timeout 8 ros2 topic hz /scan
# optional camera check (depends on healthy OAK stream)
timeout 8 ros2 topic hz /oak/rgb/image_raw
```

Expected: `/scan` non-zero; camera non-zero when OAK is publishing.

8. Verify mapping stack topics and TF:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '/map|/scan|cloud|points'
timeout 8 ros2 topic hz /scan
ros2 topic echo /map --once | sed -n '1,40p'
ros2 run tf2_ros tf2_echo map base_link | sed -n '1,20p'
```

Expected:

- `/scan` always present with non-zero rate.
- `/map` appears when `slam_toolbox` is running.
- `tf2_echo` shows valid `map -> base_link` updates when localization is active.

9. Verify 3D point cloud source (if available):

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E 'cloud|points'
# Example check when /cloud_map exists:
timeout 8 ros2 topic hz /cloud_map
```

Expected: non-zero rate for selected pointcloud topic when source pipeline is running.

10. Verify Foxglove bridge socket:

```bash
nc -vz 127.0.0.1 8765
```

Expected: connection succeeded.

11. Verify recording:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
  req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=8) as r:
    return json.load(r)
print(post('/api/recording/start', {'tags':'stack-test','topics':['/scan']}))
time.sleep(1.5)
print(post('/api/recording/stop', {}))
PY
ls -1dt ~/robot-sink/data/bags/* | head -n 2
```

Expected: start/stop `ok: true`, and a new bag path exists.

## Troubleshooting

- Foxglove bind conflict on `8765`:
  `run_stack.sh` now detects pre-bound port and assumes existing bridge instead of crashing stack.

- OAK camera not streaming (`X_LINK_DEVICE_ALREADY_IN_USE` in `jetson/logs/oakd.log`):

  ```bash
  pkill -f '[d]epthai_ros_driver camera.launch.py' || true
  pkill -f '[o]ak_container' || true
  ./jetson/scripts/run_stack.sh
  ```

- Stack process collisions from old manual runs:

  ```bash
  pkill -f '[j]etson/scripts/run_stack.sh' || true
  ```

## Enable Boot-Time Service

Install/enable service (disabled by default):

```bash
cd ~/robot-sink
./jetson/scripts/enable_services.sh --user jetson --now
```

Service commands:

```bash
sudo systemctl status robot-jetson-stack.service
sudo journalctl -u robot-jetson-stack.service -f
sudo systemctl disable robot-jetson-stack.service
```
