# 08 - Run Robot Stack

## Manual Bench Run

On Jetson:

```bash
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

Default behavior:

- Configures BB USB networking (`192.168.7.1/24`) when possible
- Starts `foxglove_bridge`, `robot_console`, OAK-D, RPLIDAR, RoArm stub, task stub, and BB bridge
- Uses real ROS nodes when installed, otherwise falls back to stubs
- Writes logs under `jetson/logs/`

## Robot Console Quick Start

When stack is running:

- Console UI: `http://<jetson>:8080`
- Console websocket: `ws://<jetson>:8080/ws`
- Foxglove bridge: `ws://<jetson>:8765`

Paths:

- Web assets: `jetson/console/web/`
- Config: `jetson/console/console_config.yaml`
- Foxglove layout seed: `jetson/console/layouts/foxglove_layout.json`

## Enable SLAM/Nav2 Runtime

`run_stack.sh` does not currently auto-launch SLAM Toolbox or Nav2. Console mapping/navigation buttons require those nodes to be running.

Install packages on Jetson:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

Run SLAM (mapping):

```bash
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

Run Nav2 (navigation):

```bash
mkdir -p "$HOME/robot-sink/ros_ws/src/nav2_config"
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
  "$HOME/robot-sink/ros_ws/src/nav2_config/nav2_params.yaml"

source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=$HOME/robot-sink/ros_ws/src/nav2_config/nav2_params.yaml
```

Verify availability:

```bash
ros2 action list | grep navigate_to_pose
ros2 service list | grep -E 'slam_toolbox|clear_entirely'
```

## Acceptance Test (Jetson)

1. Start stack:

```bash
./jetson/scripts/run_stack.sh
```

Expected:

- `Starting foxglove: ...`
- `Starting robot-console: ...`
- `Stack running. Logs: ...`

2. Confirm telemetry endpoint:

```bash
python3 - <<'PY'
import json, urllib.request
with urllib.request.urlopen('http://127.0.0.1:8080/api/status', timeout=5) as r:
    data = json.load(r)
print(data['mode'])
print(data['visualizer']['scan']['fps'], data['visualizer']['camera']['fps'])
PY
```

Expected: mode value present, stream FPS values non-zero when sensors are publishing.

3. Verify mode persistence behavior (refresh + reconnect):

```bash
python3 - <<'PY'
import asyncio, json, urllib.request, aiohttp
BASE='http://127.0.0.1:8080'
def post(path,payload):
    req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as r:
        return json.loads(r.read().decode())
def mode():
    with urllib.request.urlopen(BASE+'/api/status',timeout=5) as r:
        return json.loads(r.read().decode())['mode']
async def ws_mode_once():
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect('ws://127.0.0.1:8080/ws', timeout=5) as ws:
            msg = await ws.receive(timeout=3)
            return json.loads(msg.data)['data']['mode']
post('/api/mode', {'mode':'demo'})
print('refresh', mode())
print('ws1', asyncio.run(ws_mode_once()))
print('ws2', asyncio.run(ws_mode_once()))
post('/api/mode', {'mode':'manual'})
PY
```

Expected: same mode value from refresh and both websocket reconnects.

4. Verify manual + safety gating:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
    req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as r:
        return json.loads(r.read().decode())
post('/api/mode', {'mode':'manual'})
post('/api/arm', {'armed':True})
print('bank_armed', post('/api/motor_bank', {'left':20,'right':20}))
time.sleep(1.2)
print('bank_timeout', post('/api/motor_bank', {'left':20,'right':20}))
print('stop_all', post('/api/stop_all', {}))
PY
```

Expected: first command accepted, timeout command rejected (`robot_not_armed`), `stop_all` succeeds.

5. Verify lidar + camera topics:

```bash
source /opt/ros/humble/setup.bash
timeout 6s ros2 topic hz /scan
timeout 6s ros2 topic hz /oak/rgb/image_rect/compressed
```

Expected: non-zero rates.

6. Verify Foxglove bridge socket:

```bash
nc -vz 127.0.0.1 8765
```

Expected: connection succeeded.

7. Verify recording path:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
    req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=8) as r:
        return json.loads(r.read().decode())
print(post('/api/recording/start', {'tags':'stack-test','topics':['/scan']}))
time.sleep(1.5)
print(post('/api/recording/stop', {}))
PY
ls -1dt ~/robot-sink/data/bags/* | head -n 2
```

Expected: recording start/stop returns `ok: true`; new bag directory appears.

## Override Launch Commands

Use env vars to replace defaults with real launches:

```bash
OAK_LAUNCH_CMD='ros2 launch depthai_ros_driver camera.launch.py' \
RPLIDAR_LAUNCH_CMD='ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyRPLIDAR' \
ROARM_CMD='ros2 run roarm_driver roarm_serial_node --ros-args -p port:=/dev/ttyROARM' \
TASK_STACK_CMD='ros2 launch task_manager bench_demo.launch.py' \
FOXGLOVE_CMD='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765' \
ROBOT_CONSOLE_CMD='ros2 launch robot_console robot_console.launch.py start_foxglove:=false' \
./jetson/scripts/run_stack.sh
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
