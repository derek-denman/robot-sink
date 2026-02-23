import asyncio
import contextlib
import copy
import json
import os
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from aiohttp import WSMsgType, web
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .diagnostics import DiagnosticsProvider
from .logs import ConsoleLogCatalog
from .modes import ModeManager, RobotMode
from .recording import RosbagRecorder
from .ros_adapters import RosAdapters


DEFAULT_CONFIG: Dict[str, Any] = {
    "server": {
        "host": "0.0.0.0",
        "http_port": 8080,
        "websocket_path": "/ws",
        "web_root": "",
    },
    "foxglove": {
        "port": 8765,
    },
    "safety": {
        "command_timeout_sec": 0.8,
    },
    "streaming": {
        "scan_push_hz": 8.0,
        "camera_stream_hz": 12.0,
    },
    "manual_control": {
        "bank_linear_scale": 0.8,
        "bank_angular_scale": 1.7,
    },
    "arm": {
        "joints": [
            {"name": "j1", "min": -3.14, "max": 3.14, "step": 0.01, "default": 0.0},
            {"name": "j2", "min": -3.14, "max": 3.14, "step": 0.01, "default": 0.0},
            {"name": "j3", "min": -3.14, "max": 3.14, "step": 0.01, "default": 0.0},
            {"name": "j4", "min": -3.14, "max": 3.14, "step": 0.01, "default": 0.0},
            {"name": "j5", "min": -3.14, "max": 3.14, "step": 0.01, "default": 0.0},
        ],
        "named_poses": ["stow", "ready", "pregrasp", "drop"],
    },
    "topics": {
        "cmd_vel": "/cmd_vel",
        "scan": "/scan",
        "odom": "/odom",
        "tf": "/tf",
        "tf_static": "/tf_static",
        "camera_topics": ["/oak/rgb/image_raw", "/oak/stereo/image_raw"],
        "camera_compressed_topics": ["/oak/rgb/image_rect/compressed"],
        "camera_stream_topic": "/oak/rgb/image_rect/compressed",
        "navigate_to_pose": "/navigate_to_pose",
        "initial_pose": "/initialpose",
        "slam_pause": "/slam_toolbox/pause_new_measurements",
        "slam_resume": "/slam_toolbox/resume",
        "slam_save_map": "/slam_toolbox/save_map",
        "clear_costmap_local": "/local_costmap/clear_entirely_local_costmap",
        "clear_costmap_global": "/global_costmap/clear_entirely_global_costmap",
        "arm_gripper": "/arm/gripper_cmd",
        "arm_pose": "/arm/pose_cmd",
        "arm_joint_jog": "/arm/joint_jog",
        "arm_joints": "/arm/joints_cmd",
        "arm_stop": "/arm/stop",
    },
    "bagging": {
        "storage_path": "~/robot-sink/data/bags",
        "default_topics": ["/cmd_vel", "/odom", "/scan", "/tf", "/tf_static"],
    },
    "stack": {
        "start_cmd": "",
        "stop_cmd": "",
    },
    "mapping": {
        "switch_to_localization_cmd": "",
        "start_cmd": "",
        "stop_cmd": "",
    },
    "reliability": {
        "temp_paths": [],
    },
    "logs": {
        "sources": {},
    },
}


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_console_config(config_file: Path) -> Dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    if config_file.exists():
        raw = yaml.safe_load(config_file.read_text(encoding="utf-8")) or {}
        if isinstance(raw, dict):
            deep_merge(config, raw)
    return config


class RobotConsoleApiNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_console_api")

        self.declare_parameter("config_file", "")
        self.declare_parameter("web_root", "")
        self.declare_parameter("http_host", "")
        self.declare_parameter("http_port", 0)
        self.declare_parameter("foxglove_port", 0)

        self._robot_root = Path(os.environ.get("ROBOT_ROOT", str(Path.cwd()))).expanduser().resolve()
        self._config = self._resolve_config()

        safety_cfg = self._config.get("safety", {})
        command_timeout = float(safety_cfg.get("command_timeout_sec", 0.8))

        bag_cfg = self._config.get("bagging", {})
        bag_path = Path(bag_cfg.get("storage_path", "~/robot-sink/data/bags")).expanduser()

        self.mode_manager = ModeManager(command_timeout_sec=command_timeout)
        self.adapters = RosAdapters(self, self._config)
        self.recorder = RosbagRecorder(
            bag_path,
            default_topics=bag_cfg.get("default_topics", []),
        )
        self.diagnostics = DiagnosticsProvider(
            adapters=self.adapters,
            recorder=self.recorder,
            bag_path=bag_path,
            temp_paths=self._config.get("reliability", {}).get("temp_paths", []),
        )
        self.log_catalog = ConsoleLogCatalog(
            self._robot_root,
            source_overrides=self._config.get("logs", {}).get("sources", {}),
        )

        self._ws_clients: set[web.WebSocketResponse] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._state_lock = threading.Lock()

        self._commissioning_results: Dict[str, Dict[str, Any]] = {}
        self._mapping_state: Dict[str, Any] = {
            "slam_active": False,
            "localization_mode": False,
            "last_saved_map": None,
        }
        self._pick_place_state: Dict[str, Any] = {
            "active": False,
            "stage": "idle",
            "retry_count": 0,
            "last_target": {"x": None, "y": None, "z": None, "confidence": None},
        }
        self._demo_state: Dict[str, Any] = {
            "active": False,
            "target_count": 0,
            "completed_count": 0,
        }
        self._demo_cancel = threading.Event()

        scan_push_hz = float(self._config.get("streaming", {}).get("scan_push_hz", 8.0))
        scan_push_hz = max(1.0, min(20.0, scan_push_hz))
        self._scan_push_period = 1.0 / scan_push_hz
        self._last_scan_push_stamp = 0.0

        self._heartbeat_timer = self.create_timer(0.1, self._watchdog_tick)
        self._status_timer = self.create_timer(0.4, self._status_tick)
        self._scan_timer = self.create_timer(self._scan_push_period, self._scan_tick)

        self.get_logger().info(
            f"Robot console API configured on {self._config['server']['host']}:"
            f"{self._config['server']['http_port']}"
        )

    def _resolve_config(self) -> Dict[str, Any]:
        robot_root = str(self._robot_root)

        param_config = self.get_parameter("config_file").get_parameter_value().string_value
        env_config = os.environ.get("ROBOT_CONSOLE_CONFIG", "")
        config_file = param_config or env_config or f"{robot_root}/jetson/console/console_config.yaml"

        config = load_console_config(Path(config_file).expanduser())

        param_web_root = self.get_parameter("web_root").get_parameter_value().string_value
        env_web_root = os.environ.get("ROBOT_CONSOLE_WEB_ROOT", "")
        config["server"]["web_root"] = (
            param_web_root
            or env_web_root
            or config["server"].get("web_root")
            or f"{robot_root}/jetson/console/web"
        )

        param_http_host = self.get_parameter("http_host").get_parameter_value().string_value
        if param_http_host:
            config["server"]["host"] = param_http_host

        param_http_port = self.get_parameter("http_port").get_parameter_value().integer_value
        if int(param_http_port) > 0:
            config["server"]["http_port"] = int(param_http_port)

        param_foxglove_port = self.get_parameter("foxglove_port").get_parameter_value().integer_value
        if int(param_foxglove_port) > 0:
            config["foxglove"]["port"] = int(param_foxglove_port)

        return config

    @property
    def config(self) -> Dict[str, Any]:
        return self._config

    def attach_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def snapshot(self) -> Dict[str, Any]:
        with self._state_lock:
            mapping_state = dict(self._mapping_state)
            pick_place_state = json.loads(json.dumps(self._pick_place_state))
            demo_state = dict(self._demo_state)
            commissioning = dict(self._commissioning_results)

        encoder_last = self.adapters.latest_encoder_update_unix()
        health = self.diagnostics.health_summary()
        reliability = self.diagnostics.reliability_snapshot()
        nav_status = self.adapters.navigation_status()

        scan_status = self.adapters.scan_stream_status()
        camera_status = self.adapters.camera_stream_status()
        adapter_caps = self.adapters.capabilities()
        switch_localization_cmd = (
            self._config.get("mapping", {}).get("switch_to_localization_cmd", "").strip()
        )
        mapping_start_available = adapter_caps.get("slam_resume", False) or self.mapping_command_available(
            "start_cmd"
        )
        mapping_stop_available = adapter_caps.get("slam_pause", False) or self.mapping_command_available("stop_cmd")
        mapping_start_stop_available = mapping_start_available or mapping_stop_available

        return {
            "timestamp_unix": time.time(),
            "mode": self.mode_manager.current_mode().value,
            "safety": self.mode_manager.safety_snapshot().to_dict(),
            "health": health,
            "dashboard": {
                "base_connected": health.get("base_connected", False),
                "last_encoder_update_unix": encoder_last,
                "topic_rates": self.adapters.topic_rates(),
                "nav_state": nav_status.get("state"),
                "temperatures": reliability.get("temperatures_c", []),
            },
            "mapping": mapping_state,
            "navigation": nav_status,
            "pick_place": pick_place_state,
            "demo": demo_state,
            "recording": {
                "status": self.recorder.status(),
                "recent": self.recorder.list_recent(limit=8),
            },
            "reliability": reliability,
            "commissioning": commissioning,
            "arm": {
                "joints": self._config.get("arm", {}).get("joints", []),
                "named_poses": self._config.get("arm", {}).get("named_poses", []),
            },
            "visualizer": {
                "scan": scan_status,
                "camera": camera_status,
            },
            "logs": {
                "sources": self.log_catalog.list_sources(),
            },
            "connection": {
                "ws_clients": len(self._ws_clients),
            },
            "capabilities": {
                "stack_start": self.stack_command_available("start_cmd"),
                "stack_stop": self.stack_command_available("stop_cmd"),
                "mapping_start": mapping_start_available,
                "mapping_stop": mapping_stop_available,
                "mapping_start_stop": mapping_start_stop_available,
                "mapping_save": adapter_caps.get("slam_save", False),
                "switch_localization": bool(switch_localization_cmd),
                "nav_goal": adapter_caps.get("nav_goal", False),
                "nav_cancel": adapter_caps.get("nav_cancel", False),
                "clear_costmaps": adapter_caps.get("clear_costmaps", False),
                "camera_stream_connected": camera_status.get("connected", False),
                "stack_start_reason": "" if self.stack_command_available("start_cmd") else "stack_start_not_available",
                "stack_stop_reason": "" if self.stack_command_available("stop_cmd") else "stack_stop_not_available",
                "mapping_start_reason": "" if mapping_start_available else "slam_start_unavailable",
                "mapping_stop_reason": "" if mapping_stop_available else "slam_stop_unavailable",
                "mapping_start_stop_reason": ""
                if mapping_start_stop_available
                else "slam_services_unavailable",
                "mapping_save_reason": ""
                if adapter_caps.get("slam_save", False)
                else "slam_save_unavailable",
                "switch_localization_reason": ""
                if switch_localization_cmd
                else "switch_to_localization_cmd_not_configured",
                "nav_goal_reason": ""
                if adapter_caps.get("nav_goal", False)
                else (
                    "nav2_msgs_not_available"
                    if not adapter_caps.get("nav_msgs_available", False)
                    else "navigate_to_pose_server_unavailable"
                ),
                "nav_cancel_reason": ""
                if adapter_caps.get("nav_cancel", False)
                else "no_active_goal",
                "clear_costmaps_reason": ""
                if adapter_caps.get("clear_costmaps", False)
                else "clear_costmap_services_unavailable",
            },
            "foxglove": {
                "port": self._config.get("foxglove", {}).get("port", 8765),
                "url_template": "ws://<jetson>:" + str(self._config.get("foxglove", {}).get("port", 8765)),
            },
        }

    def _watchdog_tick(self) -> None:
        if self.mode_manager.check_watchdog():
            self.adapters.publish_stop()
            self.get_logger().warn("Motion watchdog timeout. Stop command issued and robot disarmed.")
            self._queue_status_push()

    def _status_tick(self) -> None:
        self._queue_status_push()

    def _scan_tick(self) -> None:
        scan = self.adapters.latest_scan_snapshot()
        if not scan:
            return

        stamp = float(scan.get("stamp_unix", 0.0))
        if stamp <= 0.0 or stamp <= self._last_scan_push_stamp:
            return

        self._last_scan_push_stamp = stamp
        self._queue_scan_push(scan)

    def _queue_coro(self, coro: Any) -> None:
        if not self._loop:
            return
        try:
            asyncio.run_coroutine_threadsafe(coro, self._loop)
        except RuntimeError:
            return

    def _queue_status_push(self) -> None:
        if not self._ws_clients:
            return
        self._queue_coro(self.broadcast_event("status", self.snapshot()))

    def _queue_scan_push(self, payload: Dict[str, Any]) -> None:
        if not self._ws_clients:
            return
        self._queue_coro(self.broadcast_event("scan", payload))

    async def broadcast_event(self, event_type: str, payload: Dict[str, Any]) -> None:
        if not self._ws_clients:
            return

        message = json.dumps({"type": event_type, "data": payload})

        dead = []
        for ws in self._ws_clients:
            try:
                await ws.send_str(message)
            except (ConnectionResetError, RuntimeError):
                dead.append(ws)

        for ws in dead:
            self._ws_clients.discard(ws)

    def stop_all(self) -> Dict[str, Any]:
        self.adapters.publish_stop()
        nav_result = self.adapters.cancel_navigation_goal()
        arm_result = self.adapters.stop_arm()
        self.mode_manager.disarm(reason="stop_all")
        return {
            "ok": True,
            "navigation": nav_result,
            "arm": arm_result,
        }

    def run_shell_command(self, cmd: str, background: bool = False) -> Dict[str, Any]:
        cmd = (cmd or "").strip()
        if not cmd:
            return {"ok": False, "error": "command_not_configured"}

        if background:
            subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return {"ok": True, "cmd": cmd}

        try:
            result = subprocess.run(
                ["bash", "-lc", cmd],
                check=False,
                capture_output=True,
                text=True,
                timeout=20,
            )
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "command_timeout", "cmd": cmd}

        return {
            "ok": result.returncode == 0,
            "cmd": cmd,
            "returncode": result.returncode,
            "stdout": result.stdout[-400:],
            "stderr": result.stderr[-400:],
        }

    def run_stack_command(self, command_key: str, background: bool = False) -> Dict[str, Any]:
        cmd = self._config.get("stack", {}).get(command_key, "").strip() or self._default_stack_command(
            command_key
        )
        if not cmd:
            return {"ok": False, "error": f"stack_{command_key}_not_configured"}

        return self.run_shell_command(cmd, background=background)

    def stack_command_available(self, command_key: str) -> bool:
        cmd = self._config.get("stack", {}).get(command_key, "").strip() or self._default_stack_command(
            command_key
        )
        return bool(cmd)

    def run_mapping_command(self, command_key: str, background: bool = False) -> Dict[str, Any]:
        cmd = self._config.get("mapping", {}).get(command_key, "").strip() or self._default_mapping_command(
            command_key
        )
        if not cmd:
            return {"ok": False, "error": f"mapping_{command_key}_not_configured"}
        return self.run_shell_command(cmd, background=background)

    def mapping_command_available(self, command_key: str) -> bool:
        cmd = self._config.get("mapping", {}).get(command_key, "").strip() or self._default_mapping_command(
            command_key
        )
        return bool(cmd)

    def _default_stack_command(self, command_key: str) -> str:
        robot_root = os.environ.get("ROBOT_ROOT", str(Path.cwd()))
        root_q = shlex.quote(robot_root)
        # Match the top-level run_stack script process only, not child wrappers/log lines.
        stack_proc_pattern = "^bash ./jetson/scripts/run_stack.sh$"
        if command_key == "start_cmd":
            return (
                f"pgrep -f {shlex.quote(stack_proc_pattern)} >/dev/null "
                f"|| (cd {root_q} && ./jetson/scripts/run_stack.sh)"
            )
        if command_key == "stop_cmd":
            return f"pkill -f {shlex.quote(stack_proc_pattern)} || true"
        return ""

    def _default_mapping_command(self, command_key: str) -> str:
        robot_root = os.environ.get("ROBOT_ROOT", str(Path.cwd()))
        root_q = shlex.quote(robot_root)
        ros_distro = os.environ.get("ROS_DISTRO", "humble")
        ros_setup = shlex.quote(f"/opt/ros/{ros_distro}/setup.bash")
        ws_setup = shlex.quote(f"{robot_root}/ros_ws/install/setup.bash")

        if command_key == "start_cmd":
            return (
                f"source {ros_setup} >/dev/null 2>&1; "
                f"if [ -f {ws_setup} ]; then source {ws_setup}; fi; "
                "pgrep -f '[s]lam_toolbox' >/dev/null || "
                f"(cd {root_q} && ros2 launch slam_toolbox online_async_launch.py)"
            )
        if command_key == "stop_cmd":
            return "pkill -f '[s]lam_toolbox' || true"
        return ""

    def run_commissioning_check(self, check_id: str) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "check": check_id,
            "timestamp_unix": time.time(),
            "passed": False,
            "expected": "",
            "observed": "",
        }

        if check_id == "encoder_direction":
            age = self.adapters.topic_age_sec("odom")
            result["expected"] = "Odom/encoder updates observed while wheel spins"
            result["observed"] = f"odom_age_sec={age}"
            result["passed"] = age is not None and age < 2.0

        elif check_id == "motor_tick":
            result["expected"] = "Low-duty forward tick and stop"
            if self.mode_manager.arm():
                self.adapters.publish_cmd_vel(0.08, 0.0)
                time.sleep(0.4)
                self.adapters.publish_stop()
                self.mode_manager.disarm("motor_tick_complete")
                result["observed"] = "published cmd_vel(0.08,0.0) for 0.4s then stop"
                result["passed"] = True
            else:
                result["observed"] = "cannot arm while estop latched"

        elif check_id == "estop":
            result["expected"] = "Immediate stop and estop latch"
            self.mode_manager.estop()
            self.adapters.publish_stop()
            result["observed"] = "estop latched and stop command sent"
            result["passed"] = True

        elif check_id == "watchdog":
            result["expected"] = "Disarm after command timeout"
            timeout = self.mode_manager.command_timeout_sec()
            armed = self.mode_manager.arm()
            if not armed:
                result["observed"] = "arm failed (estop latched)"
            else:
                self.mode_manager.record_motion_command()
                time.sleep(timeout + 0.25)
                timed_out = self.mode_manager.check_watchdog()
                if timed_out:
                    self.adapters.publish_stop()
                result["observed"] = f"watchdog_timeout_triggered={timed_out}"
                result["passed"] = timed_out

        elif check_id == "tf_sanity":
            frames_ok = self.adapters.tf_has_required_frames(["map", "odom", "base_link"])
            result["expected"] = "map, odom, and base_link frames available"
            result["observed"] = f"frames_ok={frames_ok}"
            result["passed"] = frames_ok

        elif check_id == "mode_persistence_manual":
            result["expected"] = "Mode should persist across refresh + websocket reconnect"
            result["observed"] = "Operator marked mode persistence verified"
            result["passed"] = True

        else:
            result["expected"] = "Known check id"
            result["observed"] = "unknown check"

        with self._state_lock:
            self._commissioning_results[check_id] = result

        return result

    def set_mapping_state(self, **kwargs: Any) -> None:
        with self._state_lock:
            self._mapping_state.update(kwargs)

    def set_pick_place_state(self, **kwargs: Any) -> None:
        with self._state_lock:
            self._pick_place_state.update(kwargs)

    def start_demo(self, target_count: int) -> Dict[str, Any]:
        with self._state_lock:
            if self._demo_state["active"]:
                return {"ok": False, "error": "demo_already_running"}

            self._demo_state = {
                "active": True,
                "target_count": int(target_count),
                "completed_count": 0,
            }
            self._demo_cancel.clear()

        thread = threading.Thread(target=self._run_demo_worker, args=(int(target_count),), daemon=True)
        thread.start()
        return {"ok": True, "state": self._demo_state}

    def stop_demo(self) -> Dict[str, Any]:
        self._demo_cancel.set()
        with self._state_lock:
            self._demo_state["active"] = False
        return {"ok": True, "state": self._demo_state}

    def _run_demo_worker(self, target_count: int) -> None:
        for idx in range(max(0, target_count)):
            if self._demo_cancel.is_set():
                break
            time.sleep(1.0)
            with self._state_lock:
                self._demo_state["completed_count"] = idx + 1
            self._queue_status_push()

        with self._state_lock:
            self._demo_state["active"] = False

        self._queue_status_push()

    def shutdown(self) -> None:
        try:
            self.adapters.publish_stop()
        except Exception:
            pass

        try:
            self.recorder.stop()
        except Exception:
            pass


async def _json_request(request: web.Request) -> Dict[str, Any]:
    try:
        payload = await request.json()
        if isinstance(payload, dict):
            return payload
        return {}
    except Exception:
        return {}


def _app_response(ok: bool, **kwargs: Any) -> web.Response:
    body = {"ok": ok}
    body.update(kwargs)

    if not ok and "error" not in body:
        result = body.get("result")
        derived_error: Optional[str] = None
        if isinstance(result, dict):
            raw_error = result.get("error")
            if isinstance(raw_error, str) and raw_error:
                derived_error = raw_error
            else:
                nested_results = result.get("results")
                if isinstance(nested_results, list):
                    for item in nested_results:
                        if not isinstance(item, dict):
                            continue
                        item_error = item.get("error")
                        if isinstance(item_error, str) and item_error:
                            service = str(item.get("service", "")).strip()
                            derived_error = f"{service}:{item_error}" if service else item_error
                            break
        if derived_error:
            body["error"] = derived_error

    return web.json_response(body)


def _normalize_bank(value: Any) -> float:
    normalized = float(value)
    if abs(normalized) > 1.0:
        normalized = normalized / 100.0
    return max(-1.0, min(1.0, normalized))


def create_app(node: RobotConsoleApiNode) -> web.Application:
    app = web.Application()

    async def get_status(_request: web.Request) -> web.Response:
        return web.json_response(node.snapshot())

    async def get_config(_request: web.Request) -> web.Response:
        return web.json_response(node.config)

    async def get_scan(_request: web.Request) -> web.Response:
        payload = node.adapters.latest_scan_snapshot()
        if payload is None:
            return _app_response(False, error="scan_not_available")
        return _app_response(True, scan=payload)

    async def get_camera_topics(_request: web.Request) -> web.Response:
        status = node.adapters.camera_stream_status()
        return _app_response(
            True,
            selected_topic=status.get("selected_topic"),
            topics=status.get("available_topics", []),
        )

    async def get_streams(_request: web.Request) -> web.Response:
        scan_status = node.adapters.scan_stream_status()
        camera_status = node.adapters.camera_stream_status()
        return _app_response(
            True,
            scan=scan_status,
            camera=camera_status,
            selected_topic=camera_status.get("selected_topic"),
            topics=camera_status.get("available_topics", []),
        )

    async def set_camera_topic(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        topic = str(payload.get("topic", "")).strip()
        result = node.adapters.select_camera_stream_topic(topic)
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    def _manual_motion_guard() -> Optional[web.Response]:
        if node.mode_manager.current_mode() != RobotMode.MANUAL:
            return _app_response(False, error="manual_mode_required")
        if not node.mode_manager.should_allow_motion():
            return _app_response(False, error="robot_not_armed")
        return None

    async def set_mode(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        mode_value = payload.get("mode", "")
        try:
            mode = node.mode_manager.set_mode(mode_value)
        except ValueError as exc:
            return _app_response(False, error=str(exc))
        node._queue_status_push()
        return _app_response(True, mode=mode.value)

    async def arm(_request: web.Request) -> web.Response:
        if node.mode_manager.arm():
            node._queue_status_push()
            return _app_response(True, safety=node.mode_manager.safety_snapshot().to_dict())
        return _app_response(False, error="estop_latched")

    async def disarm(_request: web.Request) -> web.Response:
        node.mode_manager.disarm(reason="operator")
        node.adapters.publish_stop()
        node._queue_status_push()
        return _app_response(True, safety=node.mode_manager.safety_snapshot().to_dict())

    async def arm_state(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        desired = payload.get("armed")
        action = str(payload.get("action", "")).strip().lower()

        if isinstance(desired, bool):
            should_arm = desired
        elif action in {"arm", "disarm"}:
            should_arm = action == "arm"
        else:
            return _app_response(False, error="expected 'armed':bool or action arm/disarm")

        if should_arm:
            if node.mode_manager.arm():
                node._queue_status_push()
                return _app_response(True, safety=node.mode_manager.safety_snapshot().to_dict())
            return _app_response(False, error="estop_latched")

        node.mode_manager.disarm(reason="api_arm_endpoint")
        node.adapters.publish_stop()
        node._queue_status_push()
        return _app_response(True, safety=node.mode_manager.safety_snapshot().to_dict())

    async def estop(_request: web.Request) -> web.Response:
        node.mode_manager.estop()
        node.adapters.publish_stop()
        node.adapters.stop_arm()
        node.adapters.cancel_navigation_goal()
        node._queue_status_push()
        return _app_response(True, safety=node.mode_manager.safety_snapshot().to_dict())

    async def reset_estop(_request: web.Request) -> web.Response:
        node.mode_manager.reset_estop()
        node._queue_status_push()
        return _app_response(True, safety=node.mode_manager.safety_snapshot().to_dict())

    async def stop_all(_request: web.Request) -> web.Response:
        result = node.stop_all()
        node._queue_status_push()
        return _app_response(True, result=result)

    async def teleop_cmd_vel(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        guard = _manual_motion_guard()
        if guard:
            return guard

        linear = float(payload.get("linear_x", 0.0))
        angular = float(payload.get("angular_z", 0.0))

        node.adapters.publish_cmd_vel(linear, angular)
        node.mode_manager.record_motion_command()
        return _app_response(True, linear_x=linear, angular_z=angular)

    async def motor_bank(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        guard = _manual_motion_guard()
        if guard:
            return guard

        left = _normalize_bank(payload.get("left", 0.0))
        right = _normalize_bank(payload.get("right", 0.0))
        result = node.adapters.publish_motor_bank(left=left, right=right)
        node.mode_manager.record_motion_command()
        return _app_response(True, result=result)

    async def manual_base(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        command_type = str(payload.get("type", "")).strip().lower()

        if command_type == "stop":
            node.adapters.publish_stop()
            node.mode_manager.record_motion_command()
            return _app_response(True, result={"ok": True, "type": "stop"})

        guard = _manual_motion_guard()
        if guard:
            return guard

        if command_type == "motor_bank" or "left" in payload or "right" in payload:
            left = _normalize_bank(payload.get("left", 0.0))
            right = _normalize_bank(payload.get("right", 0.0))
            result = node.adapters.publish_motor_bank(left=left, right=right)
            node.mode_manager.record_motion_command()
            return _app_response(True, result={"ok": True, "type": "motor_bank", **result})

        linear = float(payload.get("linear_x", 0.0))
        angular = float(payload.get("angular_z", 0.0))
        node.adapters.publish_cmd_vel(linear, angular)
        node.mode_manager.record_motion_command()
        return _app_response(
            True,
            result={
                "ok": True,
                "type": "cmd_vel",
                "linear_x": linear,
                "angular_z": angular,
            },
        )

    async def manual_arm(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        action = str(payload.get("action", "")).strip().lower()

        if action == "stop" or bool(payload.get("stop")):
            result = node.adapters.stop_arm()
            return _app_response(result.get("ok", False), result=result)

        if "joints" in payload and isinstance(payload.get("joints"), dict):
            cleaned: Dict[str, float] = {}
            for name, value in payload.get("joints", {}).items():
                try:
                    cleaned[str(name)] = float(value)
                except (TypeError, ValueError):
                    continue
            if not cleaned:
                return _app_response(False, error="no_valid_joint_values")
            result = node.adapters.command_joints(cleaned)
            return _app_response(result.get("ok", False), result=result)

        if "gripper" in payload:
            result = node.adapters.command_gripper(str(payload.get("gripper", "")))
            return _app_response(result.get("ok", False), result=result)

        if "pose" in payload:
            result = node.adapters.command_named_pose(str(payload.get("pose", "")))
            return _app_response(result.get("ok", False), result=result)

        if "joint" in payload and "delta" in payload:
            joint = str(payload.get("joint", ""))
            delta = float(payload.get("delta", 0.0))
            result = node.adapters.jog_joint(joint=joint, delta=delta)
            return _app_response(result.get("ok", False), result=result)

        return _app_response(False, error="manual_arm_action_not_specified")

    async def set_initial_pose(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        yaw = float(payload.get("yaw", 0.0))
        frame_id = payload.get("frame_id", "map")

        node.adapters.set_initial_pose(x=x, y=y, yaw=yaw, frame_id=frame_id)
        return _app_response(True, pose={"x": x, "y": y, "yaw": yaw, "frame_id": frame_id})

    async def nav_goal(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        yaw = float(payload.get("yaw", 0.0))

        result = node.adapters.send_navigation_goal(x=x, y=y, yaw=yaw)
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def nav_cancel(_request: web.Request) -> web.Response:
        result = node.adapters.cancel_navigation_goal()
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def clear_costmaps(_request: web.Request) -> web.Response:
        result = node.adapters.clear_costmaps()
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def mapping_start(_request: web.Request) -> web.Response:
        result = node.adapters.start_slam()
        ok = result.get("ok", False)

        if not ok and result.get("error") in {"service_unavailable", "client_missing"}:
            launch_result = node.run_mapping_command("start_cmd", background=True)
            ok = launch_result.get("ok", False)
            result = {
                "ok": ok,
                "service_result": result,
                "launch_result": launch_result,
                "path": "launch_cmd",
            }

        node.set_mapping_state(slam_active=ok, localization_mode=False)
        node._queue_status_push()
        return _app_response(ok, result=result)

    async def mapping_stop(_request: web.Request) -> web.Response:
        result = node.adapters.stop_slam()
        ok = result.get("ok", False)

        if not ok and result.get("error") in {"service_unavailable", "client_missing"}:
            stop_result = node.run_mapping_command("stop_cmd", background=False)
            ok = stop_result.get("ok", False)
            result = {
                "ok": ok,
                "service_result": result,
                "stop_result": stop_result,
                "path": "stop_cmd",
            }

        node.set_mapping_state(slam_active=False)
        node._queue_status_push()
        return _app_response(ok, result=result)

    async def mapping_save(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        map_name = str(payload.get("filename", "map_snapshot"))
        result = node.adapters.save_map(map_name)
        if result.get("ok"):
            node.set_mapping_state(last_saved_map=map_name)
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def mapping_switch_localization(_request: web.Request) -> web.Response:
        cmd = node.config.get("mapping", {}).get("switch_to_localization_cmd", "").strip()
        if cmd:
            run_result = node.run_shell_command(cmd, background=False)
            ok = run_result.get("ok", False)
        else:
            run_result = {
                "ok": False,
                "error": "switch_to_localization_cmd_not_configured",
            }
            ok = False

        node.set_mapping_state(localization_mode=ok)
        node._queue_status_push()
        return _app_response(ok, result=run_result)

    async def checklist_run(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        check_id = str(payload.get("check", ""))
        result = await asyncio.to_thread(node.run_commissioning_check, check_id)
        node._queue_status_push()
        return _app_response(result.get("passed", False), result=result)

    async def arm_gripper(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        command = str(payload.get("command", "open"))
        result = node.adapters.command_gripper(command)
        return _app_response(result.get("ok", False), result=result)

    async def arm_pose(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        pose_name = str(payload.get("pose", "stow"))
        result = node.adapters.command_named_pose(pose_name)
        return _app_response(result.get("ok", False), result=result)

    async def arm_jog(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        joint = str(payload.get("joint", "j1"))
        delta = float(payload.get("delta", 0.0))
        result = node.adapters.jog_joint(joint=joint, delta=delta)
        return _app_response(result.get("ok", False), result=result)

    async def arm_joints(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        joints = payload.get("joints", {})
        if not isinstance(joints, dict):
            return _app_response(False, error="joints must be an object")

        cleaned: Dict[str, float] = {}
        for name, value in joints.items():
            try:
                cleaned[str(name)] = float(value)
            except (TypeError, ValueError):
                continue

        if not cleaned:
            return _app_response(False, error="no_valid_joint_values")

        result = node.adapters.command_joints(cleaned)
        return _app_response(result.get("ok", False), result=result)

    async def arm_stop(_request: web.Request) -> web.Response:
        result = node.adapters.stop_arm()
        return _app_response(result.get("ok", False), result=result)

    async def gripper_alias(request: web.Request) -> web.Response:
        return await arm_gripper(request)

    async def pick_place_start(_request: web.Request) -> web.Response:
        node.set_pick_place_state(active=True, stage="seeking", retry_count=0)
        node._queue_status_push()
        return _app_response(True, state=node.snapshot().get("pick_place"))

    async def pick_place_stop(_request: web.Request) -> web.Response:
        node.set_pick_place_state(active=False, stage="stopped")
        node._queue_status_push()
        return _app_response(True, state=node.snapshot().get("pick_place"))

    async def pick_place_skip(_request: web.Request) -> web.Response:
        node.set_pick_place_state(stage="skip_requested")
        node._queue_status_push()
        return _app_response(True, state=node.snapshot().get("pick_place"))

    async def pick_place_retry(_request: web.Request) -> web.Response:
        current = node.snapshot().get("pick_place", {})
        retry_count = int(current.get("retry_count", 0)) + 1
        node.set_pick_place_state(stage="retry_requested", retry_count=retry_count)
        node._queue_status_push()
        return _app_response(True, state=node.snapshot().get("pick_place"))

    async def pick_place_abort(_request: web.Request) -> web.Response:
        node.set_pick_place_state(active=False, stage="aborted")
        node._queue_status_push()
        return _app_response(True, state=node.snapshot().get("pick_place"))

    async def recording_start(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        tags = str(payload.get("tags", "untagged"))
        topics = payload.get("topics", [])
        if not isinstance(topics, list):
            topics = []

        result = node.recorder.start(tag=tags, topics=topics)
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def recording_stop(_request: web.Request) -> web.Response:
        result = node.recorder.stop()
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def recording_list(request: web.Request) -> web.Response:
        try:
            limit = int(request.query.get("limit", "10"))
        except ValueError:
            limit = 10
        entries = node.recorder.list_recent(limit=limit)
        return _app_response(True, recordings=entries)

    async def recording_replay_hint(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        bag_path = str(payload.get("path", ""))
        return _app_response(True, command=node.recorder.replay_hint(bag_path))

    async def demo_run(request: web.Request) -> web.Response:
        payload = await _json_request(request)
        count = int(payload.get("count", 1))
        result = node.start_demo(count)
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def demo_stop(_request: web.Request) -> web.Response:
        result = node.stop_demo()
        node._queue_status_push()
        return _app_response(result.get("ok", False), result=result)

    async def stack_start(_request: web.Request) -> web.Response:
        result = node.run_stack_command("start_cmd", background=True)
        return _app_response(result.get("ok", False), result=result)

    async def stack_stop(_request: web.Request) -> web.Response:
        result = node.run_stack_command("stop_cmd", background=True)
        return _app_response(result.get("ok", False), result=result)

    async def logs_sources(_request: web.Request) -> web.Response:
        return _app_response(True, sources=node.log_catalog.list_sources())

    async def logs_diagnostics(request: web.Request) -> web.Response:
        try:
            lines_per_source = int(request.query.get("lines", "60"))
        except ValueError:
            lines_per_source = 60
        lines_per_source = max(10, min(200, lines_per_source))

        text = node.log_catalog.diagnostics_text(node.snapshot(), lines_per_source=lines_per_source)
        as_attachment = request.query.get("download", "0") in {"1", "true", "yes"}

        headers = {}
        if as_attachment:
            headers["Content-Disposition"] = "attachment; filename=robot-console-diagnostics.txt"

        return web.Response(text=text, content_type="text/plain", headers=headers)

    async def logs_stream(request: web.Request) -> web.StreamResponse:
        source = str(request.query.get("source", "robot_console")).strip()
        try:
            tail_lines = int(request.query.get("tail", "120"))
        except ValueError:
            tail_lines = 120
        tail_lines = max(20, min(1000, tail_lines))

        source_path = node.log_catalog.resolve(source)
        if source_path is None:
            return _app_response(False, error="invalid_log_source")

        if not source_path.exists():
            if source in {"nav2", "slam"}:
                return _app_response(False, error="dynamic_log_source_unavailable")
            source_path.parent.mkdir(parents=True, exist_ok=True)
            source_path.touch()

        response = web.StreamResponse(
            status=200,
            reason="OK",
            headers={
                "Content-Type": "text/event-stream",
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Connection": "keep-alive",
            },
        )
        await response.prepare(request)

        proc = await asyncio.create_subprocess_exec(
            "tail",
            "-n",
            str(tail_lines),
            "-F",
            str(source_path),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )

        last_heartbeat = time.monotonic()
        try:
            while True:
                if request.transport is None or request.transport.is_closing():
                    break

                try:
                    assert proc.stdout is not None
                    raw_line = await asyncio.wait_for(proc.stdout.readline(), timeout=1.0)
                except asyncio.TimeoutError:
                    if time.monotonic() - last_heartbeat > 4.0:
                        await response.write(b": heartbeat\n\n")
                        last_heartbeat = time.monotonic()
                    continue

                if not raw_line:
                    break

                payload = {
                    "timestamp_unix": time.time(),
                    "source": source,
                    "line": raw_line.decode("utf-8", errors="replace").rstrip("\r\n"),
                }
                await response.write(f"data: {json.dumps(payload)}\n\n".encode("utf-8"))

        except (asyncio.CancelledError, ConnectionResetError, RuntimeError):
            pass
        finally:
            with contextlib.suppress(ProcessLookupError):
                proc.terminate()
            with contextlib.suppress(Exception):
                await asyncio.wait_for(proc.wait(), timeout=1.0)
            with contextlib.suppress(Exception):
                await response.write_eof()

        return response

    async def ws_handler(request: web.Request) -> web.StreamResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)

        node._ws_clients.add(ws)
        await ws.send_json({"type": "status", "data": node.snapshot()})

        scan = node.adapters.latest_scan_snapshot()
        if scan:
            await ws.send_json({"type": "scan", "data": scan})

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    if msg.data == "ping":
                        await ws.send_str("pong")
                    else:
                        with contextlib.suppress(Exception):
                            payload = json.loads(msg.data)
                            if isinstance(payload, dict) and payload.get("type") == "camera_select":
                                topic = str(payload.get("topic", "")).strip()
                                if topic:
                                    node.adapters.select_camera_stream_topic(topic)
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            node._ws_clients.discard(ws)
            if not node._ws_clients:
                node.stop_all()
                node._queue_status_push()

        return ws

    async def camera_mjpeg(request: web.Request) -> web.StreamResponse:
        topic = request.query.get("topic", "").strip()
        if topic:
            node.adapters.select_camera_stream_topic(topic)

        selected = node.adapters.camera_stream_topic()
        if not selected:
            return _app_response(False, error="no_camera_topic_selected")

        response = web.StreamResponse(
            status=200,
            reason="OK",
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Connection": "close",
            },
        )
        await response.prepare(request)

        last_stamp = 0.0
        fps = float(node.config.get("streaming", {}).get("camera_stream_hz", 12.0))
        sleep_sec = max(0.02, 1.0 / max(1.0, min(30.0, fps)))

        try:
            while True:
                if request.transport is None or request.transport.is_closing():
                    break

                frame = node.adapters.latest_camera_frame()
                if frame and frame.get("data") and frame.get("stamp_unix", 0.0) > last_stamp:
                    last_stamp = float(frame["stamp_unix"])
                    jpg = frame["data"]
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpg)}\r\n".encode("ascii")
                        + f"X-Timestamp: {last_stamp:.6f}\r\n\r\n".encode("ascii")
                    )
                    await response.write(header + jpg + b"\r\n")

                await asyncio.sleep(sleep_sec)

        except (asyncio.CancelledError, ConnectionResetError, RuntimeError):
            pass
        finally:
            with contextlib.suppress(Exception):
                await response.write_eof()

        return response

    web_root = Path(node.config.get("server", {}).get("web_root", "")).expanduser()
    layout_root = web_root.parent / "layouts"

    async def index(_request: web.Request) -> web.Response:
        return web.FileResponse(web_root / "index.html")

    async def static_files(request: web.Request) -> web.Response:
        path = request.match_info.get("path", "")
        candidate = (web_root / path).resolve()
        web_root_resolved = web_root.resolve()

        if not str(candidate).startswith(str(web_root_resolved)):
            raise web.HTTPForbidden()

        if candidate.is_file():
            return web.FileResponse(candidate)

        return web.FileResponse(web_root / "index.html")

    async def layout_files(request: web.Request) -> web.Response:
        path = request.match_info.get("path", "")
        candidate = (layout_root / path).resolve()
        layout_root_resolved = layout_root.resolve()

        if not str(candidate).startswith(str(layout_root_resolved)):
            raise web.HTTPForbidden()

        if candidate.is_file():
            return web.FileResponse(candidate)

        raise web.HTTPNotFound()

    app.router.add_get("/api/status", get_status)
    app.router.add_get("/api/config", get_config)
    app.router.add_get("/api/scan", get_scan)
    app.router.add_get("/api/streams", get_streams)
    app.router.add_get("/api/camera/topics", get_camera_topics)

    app.router.add_post("/api/camera/select", set_camera_topic)
    app.router.add_post("/api/mode", set_mode)

    app.router.add_post("/api/safety/arm", arm)
    app.router.add_post("/api/safety/disarm", disarm)
    app.router.add_post("/api/safety/estop", estop)
    app.router.add_post("/api/safety/reset_estop", reset_estop)
    app.router.add_post("/api/arm", arm_state)
    app.router.add_post("/api/stop_all", stop_all)

    app.router.add_post("/api/teleop/cmd_vel", teleop_cmd_vel)
    app.router.add_post("/api/cmd_vel", teleop_cmd_vel)
    app.router.add_post("/api/motor_bank", motor_bank)
    app.router.add_post("/api/manual/base", manual_base)
    app.router.add_post("/api/manual/arm", manual_arm)

    app.router.add_post("/api/localization/set_initial_pose", set_initial_pose)

    app.router.add_post("/api/nav/goal", nav_goal)
    app.router.add_post("/api/nav/cancel", nav_cancel)
    app.router.add_post("/api/nav/clear_costmaps", clear_costmaps)

    app.router.add_post("/api/mapping/start", mapping_start)
    app.router.add_post("/api/mapping/stop", mapping_stop)
    app.router.add_post("/api/mapping/save", mapping_save)
    app.router.add_post("/api/mapping/switch_localization", mapping_switch_localization)

    app.router.add_post("/api/checklist/run", checklist_run)

    app.router.add_post("/api/arm/gripper", arm_gripper)
    app.router.add_post("/api/gripper", gripper_alias)
    app.router.add_post("/api/arm/pose", arm_pose)
    app.router.add_post("/api/arm/jog", arm_jog)
    app.router.add_post("/api/arm/joints", arm_joints)
    app.router.add_post("/api/arm/stop", arm_stop)

    app.router.add_post("/api/tasks/pick_place/start", pick_place_start)
    app.router.add_post("/api/tasks/pick_place/stop", pick_place_stop)
    app.router.add_post("/api/tasks/pick_place/skip", pick_place_skip)
    app.router.add_post("/api/tasks/pick_place/retry", pick_place_retry)
    app.router.add_post("/api/tasks/pick_place/abort", pick_place_abort)

    app.router.add_post("/api/recording/start", recording_start)
    app.router.add_post("/api/recording/stop", recording_stop)
    app.router.add_get("/api/recording/list", recording_list)
    app.router.add_post("/api/recording/replay_hint", recording_replay_hint)

    app.router.add_post("/api/demo/run", demo_run)
    app.router.add_post("/api/demo/stop", demo_stop)

    app.router.add_post("/api/stack/start", stack_start)
    app.router.add_post("/api/stack/stop", stack_stop)

    app.router.add_get("/api/logs/sources", logs_sources)
    app.router.add_get("/api/logs/diagnostics", logs_diagnostics)
    app.router.add_get("/api/logs/stream", logs_stream)

    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/stream/camera.mjpeg", camera_mjpeg)

    app.router.add_get("/layouts/{path:.*}", layout_files)
    app.router.add_get("/", index)
    app.router.add_get("/{path:.*}", static_files)

    return app


async def run_web_server(node: RobotConsoleApiNode) -> None:
    app = create_app(node)
    runner = web.AppRunner(app)
    await runner.setup()

    host = node.config.get("server", {}).get("host", "0.0.0.0")
    port = int(node.config.get("server", {}).get("http_port", 8080))
    site = web.TCPSite(runner, host=host, port=port)
    await site.start()

    loop = asyncio.get_running_loop()
    node.attach_event_loop(loop)

    stop_event = asyncio.Event()

    def _signal_stop() -> None:
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _signal_stop)
        except NotImplementedError:
            pass

    node.get_logger().info(f"Robot Console listening on http://{host}:{port}")

    try:
        while not stop_event.is_set():
            await asyncio.sleep(0.5)
    finally:
        await runner.cleanup()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotConsoleApiNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        asyncio.run(run_web_server(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
