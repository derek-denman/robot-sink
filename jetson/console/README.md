# Robot Console (Jetson)

Robot Console is the Jetson-hosted operator HMI and ROS 2 backend for mode/safety control, manual drive + arm, mapping/nav workflows, visual telemetry, and logs.

## Runtime URLs

- Console UI: `http://<jetson-ip>:8080`
- Telemetry WebSocket: `ws://<jetson-ip>:8080/ws`
- Foxglove bridge: `ws://<jetson-ip>:8765`

## Build + Run

Backend (Jetson):

```bash
source /opt/ros/humble/setup.bash
cd ~/robot-sink/ros_ws
colcon build --symlink-install --packages-select robot_console
source install/setup.bash
cd ~/robot-sink
./jetson/scripts/run_stack.sh
```

UI source lives in `jetson/console/ui` and production assets are served from `jetson/console/web`.

When you change the React UI, rebuild once:

```bash
export PATH="$HOME/.local/node/current/bin:$PATH"
cd ~/robot-sink/jetson/console/ui
npm install
npm run build:web
```

## HMI Layout

- Left navigation: Operations, Visualizer, Manual Base, Arm Control, Training/Data, Demo Runs, Logs
- Top bar: transport status, backend-authoritative mode selector, safety tags
- Right telemetry panel: live safety, sensor FPS/age, storage/temps, nav state, foxglove link
- Dark-mode Ant Design UI with keyboard/manual controls

## Safety + Persistence Behavior

- Starts DISARMED
- Motion gated by `manual` mode + explicit arm/deadman window
- Watchdog timeout disarms and publishes stop
- `Stop All` publishes zero base command, cancels nav goal, stops arm, and disarms
- Mode is backend source-of-truth and survives WS reconnects/refresh
- Camera topic selection is operator-sticky (no auto-overwrite while selected topic remains available)

## Visualizers

Built-in visualizer:

- Live lidar canvas from websocket `/scan` payloads
- Live camera MJPEG stream from `/stream/camera.mjpeg`
- FPS/age/connected indicators

Foxglove (recommended full view):

- Connect to `ws://<jetson-ip>:8765`
- Layout seed: `jetson/console/layouts/foxglove_layout.json`

If browser blocks insecure websocket, use Foxglove Desktop or SSH tunnel:

```bash
ssh -N -L 8765:127.0.0.1:8765 jetson@<jetson-ip>
```

Then connect to `ws://localhost:8765`.

## Logs UI

Logs page is read-only and backed by safe, whitelisted sources:

- `robot_console`, `run_stack`, `foxglove`, `oakd`, `rplidar`, `roarm`, `task`, `bb_bridge`
- dynamic sources: `nav2`, `slam` (resolved from latest ROS logs)

API endpoints:

- `GET /api/logs/sources`
- `GET /api/logs/stream?source=<id>&tail=<n>` (SSE)
- `GET /api/logs/diagnostics` (text bundle, optional download)

## Jetson Validation Commands

1. API + stream health:

```bash
python3 - <<'PY'
import json, urllib.request
with urllib.request.urlopen('http://127.0.0.1:8080/api/status', timeout=5) as r:
    d = json.load(r)
print('mode', d['mode'])
print('scan_fps', d['visualizer']['scan']['fps'])
print('camera_fps', d['visualizer']['camera']['fps'])
print('camera_topic', d['visualizer']['camera']['selected_topic'])
PY
```

Expected: `mode` present, `scan_fps > 0`; camera values non-zero when OAK is publishing.

2. WebSocket telemetry:

```bash
python3 - <<'PY'
import asyncio, json, aiohttp
async def main():
  async with aiohttp.ClientSession() as s:
    async with s.ws_connect('ws://127.0.0.1:8080/ws', timeout=5) as ws:
      for _ in range(5):
        m = await ws.receive(timeout=4)
        if m.type == aiohttp.WSMsgType.TEXT:
          print(json.loads(m.data).get('type'))
asyncio.run(main())
PY
```

Expected: includes `status` and `scan` events.

3. Mode persistence across reconnect:

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
print('before', mode())
asyncio.run(ws_once())
print('after_reconnect', mode())
post('/api/mode', {'mode':'manual'})
PY
```

Expected: `before` and `after_reconnect` stay `demo`.

4. Camera topic persistence:

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

Expected: selected topic remains `/oak/rgb/image_raw/compressed`.

5. Manual safety gating + stop_all:

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
print('disarmed', post('/api/manual/base', {'type':'motor_bank','left':20,'right':20}))
post('/api/arm', {'armed':True})
print('armed', post('/api/manual/base', {'type':'motor_bank','left':20,'right':15}))
time.sleep(1.0)
print('after_timeout', post('/api/manual/base', {'type':'motor_bank','left':20,'right':15}))
print('stop_all', post('/api/stop_all', {}))
PY
```

Expected: first command rejected, armed command accepted, timeout rejected, `stop_all` succeeds.

6. Foxglove socket:

```bash
nc -vz 127.0.0.1 8765
```

Expected: connection succeeded.

7. Recording:

```bash
python3 - <<'PY'
import json, time, urllib.request
BASE='http://127.0.0.1:8080'
def post(path,payload):
  req=urllib.request.Request(BASE+path,method='POST',data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=8) as r:
    return json.load(r)
print(post('/api/recording/start', {'tags':'console-test','topics':['/scan']}))
time.sleep(1.5)
print(post('/api/recording/stop', {}))
PY
ls -1dt ~/robot-sink/data/bags/* | head -n 2
```

Expected: start/stop `ok: true`, and a new bag directory appears.

## Troubleshooting

- `stack start/stop cmd not configured`:
  Default commands are now auto-derived; verify `/api/status` -> `capabilities.stack_start` and `capabilities.stack_stop` are `true`.

- Foxglove bind error (`Couldn't initialize websocket server: Bind Error`):
  `run_stack.sh` now detects pre-bound ports and assumes existing bridge/backend instead of crashing.

- OAK stream 0 FPS with `X_LINK_DEVICE_ALREADY_IN_USE` in `jetson/logs/oakd.log`:
  Another process still owns the OAK device. Stop duplicates and relaunch stack.

  ```bash
  pkill -f '[d]epthai_ros_driver camera.launch.py' || true
  pkill -f '[o]ak_container' || true
  ```

- SLAM/Nav actions unavailable:
  Install and launch required nodes (`slam_toolbox`, `nav2`) before using mapping/nav controls.
