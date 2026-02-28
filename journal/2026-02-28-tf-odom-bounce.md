# Journal — 2026-02-28 (TF Odom Bounce Fix)

## What I worked on
I implemented and validated a targeted fix for heading bounce in the fused 3D/BEV view by preventing duplicate `odom->base_link` TF publishers. Root cause confirmed: concurrent `base_bringup base_tf.launch.py` / `bbb_odom_tf_bridge` instances produce competing TF streams with alternating yaw.

---

## Topics covered

### 1) Root-cause hypothesis and plan
- Add a duplicate-process guard in `jetson/scripts/run_stack.sh` for `base_bringup`.
- Reuse existing runtime via `runtime_stub.py` when `base_tf.launch.py` / `bbb_odom_tf_bridge` already exists.
- Add one startup INFO reminder in `bbb_odom_tf_bridge.py` that only one instance should run.
- Update docs to prevent the double-launch footgun and add duplicate-process troubleshooting.
- Validate locally and on Jetson; capture evidence and commands.

### 2) Baseline Jetson reproduction before edits
- Confirmed duplicate base TF publishers were active before code changes.
- `pgrep -af "base_tf.launch.py|bbb_odom_tf_bridge|base_bringup"` showed two launch processes plus two `bbb_odom_tf_bridge` processes at the same time.
- `ros2 topic hz /tf` and `ros2 topic hz /odom` both reported ~71-72 Hz aggregate publish rates.
- `ros2 run tf2_ros tf2_echo odom base_link` alternated between two distinct headings/poses, including repeated jumps around:
  - yaw `123.366 deg` with translation near `[-38610, -519348]`
  - yaw `22.178 deg` with translation near `[-304991, -63094]`
- This matches the fused-view heading bounce symptom and confirms competing `odom->base_link` publishers.

### 3) `run_stack.sh` duplicate guard implementation
- Added duplicate checks in the `base_bringup` branch:
  - `process_running "[b]ase_bringup base_tf.launch.py"`
  - `process_running "[b]bb_odom_tf_bridge"`
- If either is running, `run_stack.sh` now logs:
  - `Detected existing base bringup process; reusing existing runtime.`
- In reuse mode, stack starts a `runtime_stub.py` process for `base-bringup` instead of launching a second bringup instance.
- Existing `BASE_BRINGUP_CMD` override precedence and `BB_BRIDGE_CMD` fallback branch were left unchanged.

### 4) Bridge singleton reminder log
- Added one startup INFO line in `bbb_odom_tf_bridge.py` immediately after the existing startup log.
- New message:
  - `Only run one instance of bbb_odom_tf_bridge/base_tf.launch.py to avoid competing odom->base_link TF.`
- No behavior changes were introduced in odom integration or TF publishing logic.

### 5) Docs: remove double-launch footgun
- Updated `README.md` to state that `run_stack.sh` auto-launches base TF when `base_bringup` is installed.
- Added explicit warning in docs not to manually run `ros2 launch base_bringup base_tf.launch.py` while using `run_stack.sh` (unless stack-managed base TF startup is disabled).
- Added duplicate-process troubleshooting snippet in all updated docs:
  - `pgrep -af "base_tf.launch.py|bbb_odom_tf_bridge|base_bringup"`
- Updated `docs/agents/DEBUG_PLAYBOOK.md` to replace unconditional manual launch advice with conditional guidance and duplicate-process checks.

---

## Commits from this session
- `0e0f1df` `fix(run_stack): avoid duplicate base_bringup launch`
- `86bd7c1` `fix(base_bringup): log singleton requirement for odom tf bridge`
- `ffe7a57` `docs: clarify base tf ownership and duplicate-process troubleshooting`

---

## Validation performed
- Baseline repro (pre-fix) on Jetson:
  - `pgrep -af "base_tf.launch.py|bbb_odom_tf_bridge|base_bringup"` -> duplicate launch + bridge processes observed.
  - `timeout 8 ros2 topic hz /tf` -> ~71-72 Hz.
  - `timeout 8 ros2 topic hz /odom` -> ~71-72 Hz.
  - `timeout 8 ros2 run tf2_ros tf2_echo odom base_link` -> discontinuous flip/flop between two yaw/pose solutions.

- Local static checks (post-edit):
  - `bash -n jetson/scripts/run_stack.sh` -> pass (syntax valid; non-fatal local shell locale warning only).
  - `python3 -m py_compile ros_ws/src/base_bringup/base_bringup/bbb_odom_tf_bridge.py` -> pass.

- Deploy + build on Jetson:
  - `./sync_nodes.sh --target jetson` -> sync succeeded (non-blocking rsync warnings for unrelated non-empty perception data dirs on remote delete step).
  - `cd ~/robot-sink/ros_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select base_bringup robot_console` -> both packages built successfully.

- Runtime test A (normal stack start):
  - Cleared old stack/base processes, started `./jetson/scripts/run_stack.sh`.
  - `pgrep -af "base_tf.launch.py|bbb_odom_tf_bridge|base_bringup|run_stack.sh"` showed one active base TF launch path:
    - one `ros2 launch base_bringup base_tf.launch.py`
    - one `bbb_odom_tf_bridge`
    - no duplicate second launch/bridge pair
  - `timeout 8 ros2 run tf2_ros tf2_echo odom base_link` showed stable heading (no alternating yaw values):
    - yaw remained near `0.000 deg` across samples
    - translation progressed continuously without jumping between two disconnected pose solutions

- Runtime test B (intentional double-launch attempt):
  - Started manual base TF first, then stack:
    - manual launch command (with explicit env for this Jetson shell):
      - `source /opt/ros/humble/setup.bash`
      - `export AMENT_PREFIX_PATH=/home/jetson/robot-sink/ros_ws/install/base_bringup:$AMENT_PREFIX_PATH`
      - `export PYTHONPATH=/home/jetson/robot-sink/ros_ws/build/base_bringup:$PYTHONPATH`
      - `ros2 launch base_bringup base_tf.launch.py`
    - then started `./jetson/scripts/run_stack.sh`
  - `tail -n 80 ~/robot-sink/jetson/logs/base-bringup.log` confirmed reuse path:
    - `base_bringup already running; using existing process`
    - repeating `alive` stub entries
  - `pgrep -af "run_stack.sh|base_tf.launch.py|bbb_odom_tf_bridge|runtime_stub.py --name base-bringup"` showed:
    - one manual `base_tf.launch.py`
    - one `bbb_odom_tf_bridge`
    - one `runtime_stub.py --name base-bringup` (stack reused existing runtime)
    - no second `base_tf.launch.py` launched by stack
  - `timeout 8 ros2 run tf2_ros tf2_echo odom base_link` again showed stable, non-flip/flop yaw near `0 deg` with continuous pose progression.

---

## Next steps
1. Optional: investigate why the generic Jetson shell environment does not always resolve `base_bringup` package metadata without explicit `AMENT_PREFIX_PATH` / `PYTHONPATH` exports when launched outside stack scripts.
2. Optional: add a lightweight health check endpoint/command for “single odom TF publisher” that can be run from CI or bring-up scripts.
