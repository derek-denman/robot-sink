import json
import math
import threading
import time
from collections import defaultdict, deque
from typing import Any, Callable, Dict, List, Optional, Sequence

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage

try:
    from action_msgs.msg import GoalStatus
except ImportError:  # pragma: no cover
    GoalStatus = None

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    NavigateToPose = None

try:
    from slam_toolbox.srv import SaveMap
except ImportError:  # pragma: no cover
    SaveMap = None


class RateTracker:
    def __init__(self, max_samples: int = 200) -> None:
        self._samples = deque(maxlen=max_samples)

    def tick(self) -> None:
        self._samples.append(time.monotonic())

    def hz(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        span = self._samples[-1] - self._samples[0]
        if span <= 0.0:
            return 0.0
        return round((len(self._samples) - 1) / span, 2)


class RosAdapters:
    """ROS topic/service/action wrapper helpers for robot console operations."""

    def __init__(self, node: Node, config: Dict[str, Any]) -> None:
        self._node = node
        self._config = config
        self._lock = threading.Lock()

        topics = config.get("topics", {})
        manual_cfg = config.get("manual_control", {})

        self._cmd_vel_pub = node.create_publisher(Twist, topics.get("cmd_vel", "/cmd_vel"), 10)
        self._initial_pose_pub = node.create_publisher(
            PoseWithCovarianceStamped,
            topics.get("initial_pose", "/initialpose"),
            10,
        )

        self._arm_gripper_pub = node.create_publisher(
            String, topics.get("arm_gripper", "/arm/gripper_cmd"), 10
        )
        self._arm_pose_pub = node.create_publisher(String, topics.get("arm_pose", "/arm/pose_cmd"), 10)
        self._arm_jog_pub = node.create_publisher(String, topics.get("arm_joint_jog", "/arm/joint_jog"), 10)
        self._arm_joints_pub = node.create_publisher(
            String, topics.get("arm_joints", "/arm/joints_cmd"), 10
        )
        self._arm_stop_pub = node.create_publisher(String, topics.get("arm_stop", "/arm/stop"), 10)

        self._rate = defaultdict(RateTracker)
        self._last_topic_unix: Dict[str, float] = {}
        self._tf_frames: set[str] = set()

        self._scan_topic = topics.get("scan", "/scan")
        self._odom_topic = topics.get("odom", "/odom")
        self._camera_topics = topics.get("camera_topics", []) or [topics.get("camera", "/oak/rgb/image_raw")]
        self._tf_topic = topics.get("tf", "/tf")
        self._tf_static_topic = topics.get("tf_static", "/tf_static")

        self._scan_sub = node.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._odom_sub = node.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._tf_sub = node.create_subscription(TFMessage, self._tf_topic, self._on_tf, 30)
        self._tf_static_sub = node.create_subscription(
            TFMessage, self._tf_static_topic, self._on_tf_static, 30
        )

        self._camera_subs = []
        for topic in self._camera_topics:
            self._camera_subs.append(
                node.create_subscription(
                    Image,
                    topic,
                    self._make_camera_callback(topic),
                    qos_profile_sensor_data,
                )
            )

        self._camera_stream_subs: Dict[str, Any] = {}
        self._camera_stream_frames: Dict[str, Dict[str, Any]] = {}
        self._camera_stream_lock = threading.Lock()

        configured_stream_topics = topics.get("camera_compressed_topics", []) or []
        configured_stream_topics = [
            topic for topic in configured_stream_topics if self._is_supported_mjpeg_topic(topic)
        ]
        derived_stream_topics = self._derived_compressed_topics(self._camera_topics)
        default_stream_topics = self._unique_topics(configured_stream_topics + derived_stream_topics)

        explicit_stream_topic = topics.get("camera_stream_topic", "")
        self._camera_stream_topic = explicit_stream_topic or (default_stream_topics[0] if default_stream_topics else "")

        for topic in default_stream_topics:
            self.ensure_camera_stream_subscription(topic)

        nav_goal_topic = topics.get("navigate_to_pose", "/navigate_to_pose")
        self._nav_client = None
        if NavigateToPose is not None:
            self._nav_client = ActionClient(node, NavigateToPose, nav_goal_topic)

        self._nav_goal_handle = None
        self._nav_state = "idle"
        self._nav_last_result = "none"
        self._nav_last_feedback: Dict[str, Any] = {}

        self._clear_costmap_clients = [
            node.create_client(
                Empty,
                topics.get("clear_costmap_local", "/local_costmap/clear_entirely_local_costmap"),
            ),
            node.create_client(
                Empty,
                topics.get("clear_costmap_global", "/global_costmap/clear_entirely_global_costmap"),
            ),
        ]

        self._slam_pause_client = node.create_client(
            Empty,
            topics.get("slam_pause", "/slam_toolbox/pause_new_measurements"),
        )
        self._slam_resume_client = node.create_client(
            Empty,
            topics.get("slam_resume", "/slam_toolbox/resume"),
        )
        self._slam_save_client = None
        if SaveMap is not None:
            self._slam_save_client = node.create_client(
                SaveMap,
                topics.get("slam_save_map", "/slam_toolbox/save_map"),
            )

        self._motion_watchdog_enabled = True

        self._scan_payload_lock = threading.Lock()
        self._latest_scan_payload: Optional[Dict[str, Any]] = None

        self._bank_linear_scale = float(manual_cfg.get("bank_linear_scale", 0.8))
        self._bank_angular_scale = float(manual_cfg.get("bank_angular_scale", 1.7))

    @staticmethod
    def _is_supported_mjpeg_topic(topic_name: str) -> bool:
        topic = (topic_name or "").strip()
        if not topic:
            return False
        return "compressedDepth" not in topic

    @staticmethod
    def _unique_topics(topics: Sequence[str]) -> List[str]:
        deduped: List[str] = []
        seen = set()
        for topic in topics:
            name = (topic or "").strip()
            if not name or name in seen:
                continue
            seen.add(name)
            deduped.append(name)
        return deduped

    @staticmethod
    def _derived_compressed_topics(raw_topics: Sequence[str]) -> List[str]:
        candidates: List[str] = []
        for topic in raw_topics:
            if not topic:
                continue
            if topic.endswith("/compressed"):
                candidates.append(topic)
                continue

            candidates.append(f"{topic}/compressed")
            if topic.endswith("/image_raw"):
                candidates.append(topic.replace("/image_raw", "/image_rect/compressed"))

        # Keep order stable and unique.
        deduped: List[str] = []
        seen = set()
        for topic in candidates:
            if topic in seen:
                continue
            seen.add(topic)
            deduped.append(topic)
        return [topic for topic in deduped if RosAdapters._is_supported_mjpeg_topic(topic)]

    def _mark_topic(self, key: str) -> None:
        self._rate[key].tick()
        self._last_topic_unix[key] = time.time()

    def _on_scan(self, msg: LaserScan) -> None:
        self._mark_topic("scan")
        payload = self._build_scan_payload(msg)
        with self._scan_payload_lock:
            self._latest_scan_payload = payload

    def _build_scan_payload(self, msg: LaserScan, max_points: int = 360) -> Dict[str, Any]:
        count = len(msg.ranges)
        step = max(1, count // max(1, max_points))

        points: List[List[float]] = []
        angle = msg.angle_min
        for idx, value in enumerate(msg.ranges):
            if idx % step != 0:
                angle += msg.angle_increment
                continue

            r = float(value)
            valid = math.isfinite(r) and (r >= msg.range_min) and (r <= msg.range_max)
            if valid:
                x = round(math.cos(angle) * r, 4)
                y = round(math.sin(angle) * r, 4)
                points.append([x, y])
                if len(points) >= max_points:
                    break

            angle += msg.angle_increment

        return {
            "stamp_unix": time.time(),
            "frame_id": msg.header.frame_id,
            "range_min": round(float(msg.range_min), 4),
            "range_max": round(float(msg.range_max), 4),
            "point_count": len(points),
            "points": points,
        }

    def _on_odom(self, _msg: Odometry) -> None:
        self._mark_topic("odom")

    def _on_tf(self, msg: TFMessage) -> None:
        self._mark_topic("tf")
        self._remember_frames(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._mark_topic("tf_static")
        self._remember_frames(msg)

    def _make_camera_callback(self, topic_name: str) -> Callable[[Image], None]:
        key = f"camera:{topic_name}"

        def _cb(_msg: Image) -> None:
            self._mark_topic("camera")
            self._mark_topic(key)

        return _cb

    def _make_camera_stream_callback(self, topic_name: str) -> Callable[[CompressedImage], None]:
        key = f"camera_stream:{topic_name}"

        def _cb(msg: CompressedImage) -> None:
            self._mark_topic("camera_stream")
            self._mark_topic(key)

            with self._camera_stream_lock:
                self._camera_stream_frames[topic_name] = {
                    "stamp_unix": time.time(),
                    "format": msg.format,
                    "data": bytes(msg.data),
                }

                # Select first active stream topic automatically.
                if not self._camera_stream_topic:
                    self._camera_stream_topic = topic_name

        return _cb

    def ensure_camera_stream_subscription(self, topic_name: str) -> None:
        if not topic_name:
            return
        if topic_name in self._camera_stream_subs:
            return

        self._camera_stream_subs[topic_name] = self._node.create_subscription(
            CompressedImage,
            topic_name,
            self._make_camera_stream_callback(topic_name),
            qos_profile_sensor_data,
        )

    def _remember_frames(self, msg: TFMessage) -> None:
        with self._lock:
            for transform in msg.transforms:
                parent = transform.header.frame_id.strip("/")
                child = transform.child_frame_id.strip("/")
                if parent:
                    self._tf_frames.add(parent)
                if child:
                    self._tf_frames.add(child)

    def topic_rates(self) -> Dict[str, float]:
        keys = ["scan", "odom", "camera", "camera_stream", "tf", "tf_static"]
        rates = {key: self._rate[key].hz() for key in keys}
        for topic in self._camera_topics:
            key = f"camera:{topic}"
            rates[key] = self._rate[key].hz()

        for topic in self.camera_stream_topics():
            key = f"camera_stream:{topic}"
            rates[key] = self._rate[key].hz()

        return rates

    def topic_age_sec(self, key: str) -> Optional[float]:
        timestamp = self._last_topic_unix.get(key)
        if timestamp is None:
            return None
        return max(0.0, time.time() - timestamp)

    def tf_has_required_frames(self, required: Sequence[str]) -> bool:
        with self._lock:
            known = set(self._tf_frames)
        for frame in required:
            if frame not in known:
                return False
        return True

    def latest_encoder_update_unix(self) -> Optional[float]:
        return self._last_topic_unix.get("odom")

    def motion_watchdog_enabled(self) -> bool:
        return self._motion_watchdog_enabled

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(msg)

    def publish_motor_bank(self, left: float, right: float) -> Dict[str, float]:
        left_clamped = max(-1.0, min(1.0, float(left)))
        right_clamped = max(-1.0, min(1.0, float(right)))

        linear = (left_clamped + right_clamped) * 0.5 * self._bank_linear_scale
        angular = (right_clamped - left_clamped) * self._bank_angular_scale
        self.publish_cmd_vel(linear, angular)

        return {
            "left": round(left_clamped, 4),
            "right": round(right_clamped, 4),
            "linear_x": round(linear, 4),
            "angular_z": round(angular, 4),
        }

    def publish_stop(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)

    def set_initial_pose(self, x: float, y: float, yaw: float, frame_id: str = "map") -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.068
        msg.pose.covariance = covariance

        self._initial_pose_pub.publish(msg)

    def command_gripper(self, command: str) -> Dict[str, Any]:
        msg = String()
        msg.data = command
        self._arm_gripper_pub.publish(msg)
        return {"ok": True, "command": command}

    def command_named_pose(self, pose_name: str) -> Dict[str, Any]:
        msg = String()
        msg.data = pose_name
        self._arm_pose_pub.publish(msg)
        return {"ok": True, "pose": pose_name}

    def jog_joint(self, joint: str, delta: float) -> Dict[str, Any]:
        msg = String()
        msg.data = f"{joint}:{delta:.4f}"
        self._arm_jog_pub.publish(msg)
        return {"ok": True, "joint": joint, "delta": float(delta)}

    def command_joints(self, joints: Dict[str, float]) -> Dict[str, Any]:
        payload = {str(name): float(value) for name, value in joints.items()}
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._arm_joints_pub.publish(msg)
        return {"ok": True, "joints": payload}

    def stop_arm(self) -> Dict[str, Any]:
        msg = String()
        msg.data = "stop"
        self._arm_stop_pub.publish(msg)
        return {"ok": True}

    def start_slam(self) -> Dict[str, Any]:
        return self._call_empty_service(self._slam_resume_client, "slam_resume")

    def stop_slam(self) -> Dict[str, Any]:
        return self._call_empty_service(self._slam_pause_client, "slam_pause")

    def save_map(self, map_name: str, timeout_sec: float = 5.0) -> Dict[str, Any]:
        if SaveMap is None or self._slam_save_client is None:
            return {
                "ok": False,
                "error": "slam_toolbox_python_interface_missing",
            }

        if not self._slam_save_client.wait_for_service(timeout_sec=1.0):
            return {
                "ok": False,
                "error": "slam_save_service_unavailable",
            }

        req = SaveMap.Request()
        # Support multiple versions of SaveMap request naming.
        for field, value in {
            "name": map_name,
            "map_name": map_name,
            "map_url": map_name,
            "image_format": "pgm",
        }.items():
            if hasattr(req, field):
                setattr(req, field, value)

        future = self._slam_save_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout_sec):
            return {"ok": False, "error": "slam_save_timeout"}

        exc = future.exception()
        if exc is not None:
            return {"ok": False, "error": str(exc)}

        response = future.result()
        return {
            "ok": True,
            "response": self._msg_to_dict(response),
        }

    def send_navigation_goal(self, x: float, y: float, yaw: float, frame_id: str = "map") -> Dict[str, Any]:
        if NavigateToPose is None or self._nav_client is None:
            return {"ok": False, "error": "nav2_msgs_not_available"}

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            return {"ok": False, "error": "navigate_to_pose_server_unavailable"}

        with self._lock:
            if self._nav_goal_handle is not None:
                return {"ok": False, "error": "goal_already_active"}
            self._nav_state = "pending"
            self._nav_last_result = "pending"

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.header.frame_id = frame_id
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        future = self._nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(self._on_nav_goal_response)

        return {
            "ok": True,
            "goal": {"x": float(x), "y": float(y), "yaw": float(yaw), "frame_id": frame_id},
        }

    def cancel_navigation_goal(self) -> Dict[str, Any]:
        with self._lock:
            goal_handle = self._nav_goal_handle
            if goal_handle is None:
                return {"ok": False, "error": "no_active_goal"}
            self._nav_state = "canceling"

        future = goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_nav_cancel_response)
        return {"ok": True}

    def clear_costmaps(self) -> Dict[str, Any]:
        results = []
        for idx, client in enumerate(self._clear_costmap_clients):
            results.append(self._call_empty_service(client, f"clear_costmap_{idx}"))

        return {
            "ok": all(result.get("ok") for result in results),
            "results": results,
        }

    def navigation_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "state": self._nav_state,
                "last_result": self._nav_last_result,
                "feedback": dict(self._nav_last_feedback),
                "active": self._nav_goal_handle is not None,
            }

    def latest_scan_snapshot(self) -> Optional[Dict[str, Any]]:
        with self._scan_payload_lock:
            if self._latest_scan_payload is None:
                return None
            return json.loads(json.dumps(self._latest_scan_payload))

    def scan_stream_status(self) -> Dict[str, Any]:
        rates = self.topic_rates()
        scan_age = self.topic_age_sec("scan")
        payload = self.latest_scan_snapshot()
        return {
            "topic": self._scan_topic,
            "fps": rates.get("scan", 0.0),
            "age_sec": scan_age,
            "point_count": (payload or {}).get("point_count", 0),
            "connected": scan_age is not None and scan_age < 2.0,
        }

    def camera_stream_topics(self) -> List[str]:
        discovered = []
        try:
            for topic_name, type_names in self._node.get_topic_names_and_types():
                if (
                    "sensor_msgs/msg/CompressedImage" in type_names
                    and self._is_supported_mjpeg_topic(topic_name)
                ):
                    discovered.append(topic_name)
        except Exception:
            pass

        for topic in discovered:
            self.ensure_camera_stream_subscription(topic)

        combined = (
            list(self._camera_stream_subs.keys())
            + self._derived_compressed_topics(self._camera_topics)
            + discovered
        )
        return sorted(self._unique_topics(combined))

    def camera_stream_topic(self) -> str:
        return self._camera_stream_topic

    def select_camera_stream_topic(self, topic_name: str) -> Dict[str, Any]:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}

        self.ensure_camera_stream_subscription(topic_name)
        self._camera_stream_topic = topic_name
        return {"ok": True, "topic": topic_name}

    def latest_camera_frame(self) -> Optional[Dict[str, Any]]:
        topic = self._camera_stream_topic
        if not topic:
            return None

        with self._camera_stream_lock:
            frame = self._camera_stream_frames.get(topic)
            if frame is None:
                return None
            return {
                "topic": topic,
                "stamp_unix": float(frame.get("stamp_unix", 0.0)),
                "format": frame.get("format", ""),
                "data": frame.get("data", b""),
            }

    def _camera_topic_metrics(self, topics: Sequence[str]) -> List[Dict[str, Any]]:
        now = time.time()
        rates = self.topic_rates()
        metrics: List[Dict[str, Any]] = []

        with self._camera_stream_lock:
            frames = dict(self._camera_stream_frames)

        for topic in topics:
            stamp = None
            frame = frames.get(topic)
            if frame is not None:
                stamp = float(frame.get("stamp_unix", 0.0))

            age = None
            if stamp is not None and stamp > 0.0:
                age = max(0.0, now - stamp)

            fps_key = f"camera_stream:{topic}"
            fps = rates.get(fps_key, 0.0)
            connected = age is not None and age < 2.0

            metrics.append(
                {
                    "topic": topic,
                    "fps": fps,
                    "age_sec": age,
                    "connected": connected,
                    "format": frame.get("format") if frame else None,
                }
            )

        return metrics

    def _auto_select_camera_topic(self, metrics: Sequence[Dict[str, Any]]) -> None:
        selected = self._camera_stream_topic
        if selected:
            for metric in metrics:
                if metric.get("topic") == selected:
                    return

        connected_topics = [metric for metric in metrics if metric.get("connected")]
        if not connected_topics:
            return

        # Prefer highest observed FPS among connected topics.
        connected_topics.sort(key=lambda item: float(item.get("fps", 0.0)), reverse=True)
        best_topic = str(connected_topics[0].get("topic", "")).strip()
        if best_topic:
            self._camera_stream_topic = best_topic

    def camera_stream_status(self) -> Dict[str, Any]:
        available_topics = self.camera_stream_topics()
        topic_metrics = self._camera_topic_metrics(available_topics)
        self._auto_select_camera_topic(topic_metrics)
        topic = self._camera_stream_topic

        selected_metric = None
        for metric in topic_metrics:
            if metric.get("topic") == topic:
                selected_metric = metric
                break

        if selected_metric is None:
            selected_metric = {
                "fps": 0.0,
                "age_sec": None,
                "connected": False,
                "format": None,
            }

        return {
            "selected_topic": topic,
            "available_topics": available_topics,
            "topic_metrics": topic_metrics,
            "fps": selected_metric.get("fps", 0.0),
            "age_sec": selected_metric.get("age_sec"),
            "connected": selected_metric.get("connected", False),
            "format": selected_metric.get("format"),
        }

    @staticmethod
    def _service_ready(client: Any) -> bool:
        if client is None:
            return False
        try:
            return bool(client.service_is_ready())
        except Exception:
            return False

    def capabilities(self) -> Dict[str, bool]:
        nav_msgs_available = NavigateToPose is not None and self._nav_client is not None
        nav_server_ready = False
        if nav_msgs_available:
            try:
                nav_server_ready = bool(self._nav_client.server_is_ready())
            except Exception:
                nav_server_ready = False

        with self._lock:
            nav_cancel = self._nav_goal_handle is not None

        slam_resume = self._service_ready(self._slam_resume_client)
        slam_pause = self._service_ready(self._slam_pause_client)
        slam_start_stop = slam_resume and slam_pause
        slam_save = SaveMap is not None and self._service_ready(self._slam_save_client)
        clear_costmaps = all(self._service_ready(client) for client in self._clear_costmap_clients)

        return {
            "nav_msgs_available": nav_msgs_available,
            "nav_server_ready": nav_server_ready,
            "nav_goal": nav_msgs_available and nav_server_ready,
            "nav_cancel": nav_cancel,
            "clear_costmaps": clear_costmaps,
            "slam_resume": slam_resume,
            "slam_pause": slam_pause,
            "slam_start_stop": slam_start_stop,
            "slam_save": slam_save,
        }

    def _on_nav_feedback(self, feedback_msg: Any) -> None:
        with self._lock:
            feedback = getattr(feedback_msg, "feedback", None)
            if feedback is None:
                self._nav_last_feedback = {}
                return

            distance = getattr(feedback, "distance_remaining", None)
            eta = getattr(feedback, "estimated_time_remaining", None)
            eta_sec = None
            if eta is not None and hasattr(eta, "sec"):
                eta_sec = float(eta.sec) + float(getattr(eta, "nanosec", 0)) / 1e9

            self._nav_last_feedback = {
                "distance_remaining": float(distance) if distance is not None else None,
                "eta_sec": eta_sec,
            }

    def _on_nav_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self._node.get_logger().error(f"NavigateToPose goal send failed: {exc}")
            with self._lock:
                self._nav_state = "idle"
                self._nav_last_result = "send_failed"
                self._nav_goal_handle = None
            return

        if not goal_handle.accepted:
            with self._lock:
                self._nav_state = "idle"
                self._nav_last_result = "rejected"
                self._nav_goal_handle = None
            return

        with self._lock:
            self._nav_state = "active"
            self._nav_last_result = "active"
            self._nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future: Any) -> None:
        status_name = "unknown"
        try:
            wrapped = future.result()
            status_name = self._goal_status_name(getattr(wrapped, "status", None))
        except Exception as exc:  # pragma: no cover
            self._node.get_logger().error(f"NavigateToPose result failed: {exc}")
            status_name = "result_failed"

        with self._lock:
            self._nav_last_result = status_name
            self._nav_state = "idle"
            self._nav_goal_handle = None

    def _on_nav_cancel_response(self, future: Any) -> None:
        try:
            _ = future.result()
            status = "canceled"
        except Exception:  # pragma: no cover
            status = "cancel_failed"

        with self._lock:
            self._nav_last_result = status
            self._nav_state = "idle"
            self._nav_goal_handle = None

    def _goal_status_name(self, status_code: Optional[int]) -> str:
        if status_code is None or GoalStatus is None:
            return "unknown"

        names = {
            GoalStatus.STATUS_UNKNOWN: "unknown",
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        return names.get(status_code, f"status_{status_code}")

    def _call_empty_service(
        self,
        client: Any,
        service_name: str,
        timeout_sec: float = 2.0,
    ) -> Dict[str, Any]:
        if client is None:
            return {"ok": False, "service": service_name, "error": "client_missing"}

        if not client.wait_for_service(timeout_sec=0.8):
            return {
                "ok": False,
                "service": service_name,
                "error": "service_unavailable",
            }

        future = client.call_async(Empty.Request())
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_sec):
            return {"ok": False, "service": service_name, "error": "service_timeout"}

        exc = future.exception()
        if exc is not None:
            return {
                "ok": False,
                "service": service_name,
                "error": str(exc),
            }

        return {"ok": True, "service": service_name}

    @staticmethod
    def _msg_to_dict(msg: Any) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        if msg is None:
            return result

        slots = getattr(msg, "__slots__", [])
        for slot in slots:
            key = slot.lstrip("_")
            result[key] = getattr(msg, key, None)
        return result
