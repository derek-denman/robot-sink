# Robot Console (Jetson)

Robot Console is a Jetson-hosted web UI + ROS 2 backend for robot mode management, safety gating, teleop, arm commands, mapping/navigation actions, commissioning checks, and reliability/demo workflows.

## Start

On Jetson:

```bash
source /opt/ros/humble/setup.bash
cd ~/robot-sink/ros_ws
colcon build --symlink-install --packages-select robot_console
source install/setup.bash
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

This starts:

- `robot_console` backend (`http://0.0.0.0:8080`)
- WebSocket telemetry (`ws://<jetson>:8080/ws`)
- Foxglove bridge (`ws://<jetson>:8765`)

## Open UI

From your laptop (same network):

```text
http://<jetson-ip>:8080
```

Example from macOS Safari/Chrome: `http://10.0.0.178:8080`

## Visualizers

Built-in visualizer in the UI:

- Live lidar canvas (`/scan` over console websocket)
- Live OAK-D MJPEG (`/stream/camera.mjpeg`)
- FPS + age + connection indicators for both

Foxglove (recommended full view):

- Connect to `ws://<jetson-ip>:8765`
- Default layout seed: `jetson/console/layouts/foxglove_layout.json`

If Foxglove Web blocks insecure websocket access, use Foxglove Desktop, or tunnel from Mac:

```bash
ssh -N -L 8765:127.0.0.1:8765 jetson@<jetson-ip>
```

Then connect Foxglove to:

```text
ws://localhost:8765
```

## Core UI Behavior

- Mode source of truth is backend mode (`/api/status`)
- UI stores last selected mode in `localStorage` and reconciles on websocket reconnect
- Robot starts disarmed
- Motion commands require manual mode + explicit arm
- Motion watchdog timeout auto-disarms + publishes stop
- `Stop All` publishes zero base command, cancels nav goal, stops arm, and disarms

## Config

Edit `jetson/console/console_config.yaml` for:

- HTTP host/port
- Foxglove port
- Topic names
- Stream rates (`scan_push_hz`, `camera_stream_hz`)
- Motor bank scaling
- Bag storage path

## Troubleshooting

Logs:

- `jetson/logs/robot-console.log`
- `jetson/logs/foxglove.log`
- `jetson/logs/oakd.log`
- `jetson/logs/rplidar.log`

Check service-template install status:

```bash
systemctl status robot-jetson-stack.service --no-pager
```

If unit is missing, install/enable with:

```bash
./jetson/scripts/enable_services.sh --user jetson
```

(Requires sudo privileges on Jetson.)

## How To Test

1. Start stack:

```bash
./jetson/scripts/run_stack.sh
```

Expected log lines in `/tmp/robot_stack_manual.log` or terminal:

- `Starting foxglove: ...`
- `Starting robot-console: ...`
- `Stack running. Logs: ...`

2. Verify API status:

```bash
python3 - <<'PY'
import json, urllib.request
with urllib.request.urlopen('http://127.0.0.1:8080/api/status', timeout=5) as r:
    data = json.load(r)
print('mode', data['mode'])
print('scan_fps', data['visualizer']['scan']['fps'])
print('camera_fps', data['visualizer']['camera']['fps'])
PY
```

Expected: `mode manual`, non-zero scan/camera fps when sensors are active.

3. Verify websocket telemetry:

```bash
python3 - <<'PY'
import asyncio, json, aiohttp
async def main():
    async with aiohttp.ClientSession() as s:
        async with s.ws_connect('ws://127.0.0.1:8080/ws', timeout=5) as ws:
            for _ in range(3):
                msg = await ws.receive(timeout=3)
                if msg.type == aiohttp.WSMsgType.TEXT:
                    print(json.loads(msg.data).get('type'))
asyncio.run(main())
PY
```

Expected: includes `status` and `scan` events.

4. Verify manual/safety gating:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
    req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=5) as r:
        return json.loads(r.read().decode())
print('disarmed_cmd', post('/api/cmd_vel', {'linear_x':0.1,'angular_z':0.0}))
post('/api/mode', {'mode':'manual'})
post('/api/arm', {'armed':True})
print('bank_armed', post('/api/motor_bank', {'left':25,'right':30}))
time.sleep(1.2)
print('bank_after_timeout', post('/api/motor_bank', {'left':25,'right':30}))
print('stop_all', post('/api/stop_all', {}))
PY
```

Expected:

- disarmed command rejected
- armed command accepted
- after timeout command rejected (`robot_not_armed`)
- `stop_all` returns `ok: true`

5. Verify Foxglove bridge:

```bash
nc -vz 127.0.0.1 8765
```

Expected: connection succeeded.

6. Verify recording:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
    req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=8) as r:
        return json.loads(r.read().decode())
print(post('/api/recording/start', {'tags':'console-test','topics':['/scan']}))
time.sleep(1.5)
print(post('/api/recording/stop', {}))
PY
ls -1dt ~/robot-sink/data/bags/* | head -n 2
```

Expected: start/stop both `ok: true`; new bag directory exists.
