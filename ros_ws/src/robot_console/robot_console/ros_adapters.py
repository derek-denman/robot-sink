import contextlib
import json
import math
import struct
import threading
import time
from collections import defaultdict, deque
from typing import Any, Callable, Dict, List, Optional, Sequence

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan, PointCloud2
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

try:
    from vision_msgs.msg import Detection3DArray
except ImportError:  # pragma: no cover
    Detection3DArray = None


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
        self._tf_graph: Dict[str, Dict[str, Dict[str, float]]] = defaultdict(dict)

        self._scan_topic = str(topics.get("scan", "/scan")).strip() or "/scan"
        self._odom_topic = str(topics.get("odom", "/odom")).strip() or "/odom"
        self._map_topic = str(topics.get("map", "/map")).strip() or "/map"
        self._costmap_topic = str(topics.get("costmap", "/local_costmap/costmap")).strip() or "/local_costmap/costmap"
        self._map_frame = str(topics.get("map_frame", "map")).strip().strip("/") or "map"
        self._base_frame = str(topics.get("base_frame", "base_link")).strip().strip("/") or "base_link"
        self._rgb_topic = str(
            topics.get("rgb", topics.get("camera", "/oak/rgb/image_raw"))
        ).strip() or "/oak/rgb/image_raw"
        self._depth_topic = str(topics.get("depth", "/oak/stereo/image_raw")).strip() or "/oak/stereo/image_raw"
        self._detection_topic = str(topics.get("detections", "/oak/nn/spatial_detections")).strip()
        self._camera_info_topic = str(topics.get("camera_info", "/oak/rgb/camera_info")).strip()
        configured_camera_topics = topics.get("camera_topics", []) or []
        self._camera_topics = self._unique_topics(
            [self._rgb_topic, self._depth_topic] + list(configured_camera_topics)
        )
        self._tf_topic = topics.get("tf", "/tf")
        self._tf_static_topic = topics.get("tf_static", "/tf_static")
        self._fixed_frame_candidates = ["map", "odom", "base_link"]
        self._selected_fixed_frame = str(topics.get("fixed_frame", "")).strip().strip("/")
        configured_path_topics = topics.get("path_topics", [])
        if not configured_path_topics:
            configured_path_topics = [topics.get("path", "/plan"), "/plan", "/global_plan", "/local_plan"]
        self._path_topics = self._unique_topics(configured_path_topics)
        self._selected_path_topic = topics.get("path_topic", "") or (
            self._path_topics[0] if self._path_topics else ""
        )

        self._pointcloud_topic = str(topics.get("pointcloud", "")).strip()
        self._pointcloud_max_points = int(config.get("streaming", {}).get("pointcloud_max_points", 4500))
        self._scan_overlay_points = int(config.get("streaming", {}).get("scan_overlay_points", 520))
        obstacle_cfg = config.get("obstacle_map", {})
        self._obstacle_source_mode = str(obstacle_cfg.get("source_mode", "auto")).strip().lower() or "auto"
        self._obstacle_resolution = max(0.02, min(0.25, float(obstacle_cfg.get("resolution_m", 0.05))))
        self._obstacle_window_m = max(4.0, min(30.0, float(obstacle_cfg.get("window_size_m", 16.0))))
        self._obstacle_max_range_m = max(1.0, min(30.0, float(obstacle_cfg.get("max_range_m", 8.0))))
        self._obstacle_outlier_threshold_m = max(
            0.05, min(2.0, float(obstacle_cfg.get("outlier_threshold_m", 0.4)))
        )
        self._obstacle_inflation_radius_m = max(
            0.0, min(3.0, float(obstacle_cfg.get("inflation_radius_m", 0.35)))
        )
        self._obstacle_decay_time_sec = max(
            0.5, min(20.0, float(obstacle_cfg.get("decay_time_sec", 2.5)))
        )
        self._obstacle_decay_start_sec = max(
            0.0, min(10.0, float(obstacle_cfg.get("decay_start_sec", 0.6)))
        )
        self._obstacle_depth_fusion_enabled = bool(obstacle_cfg.get("depth_fusion_enabled", False))
        self._obstacle_compute_hz = max(1.0, min(20.0, float(obstacle_cfg.get("compute_hz", 2.0))))
        self._obstacle_compute_period = 1.0 / self._obstacle_compute_hz
        self._last_obstacle_compute_unix = 0.0
        self._obstacle_scan_max_points = int(obstacle_cfg.get("scan_max_points", 240))
        self._obstacle_scan_max_points = max(120, min(1200, self._obstacle_scan_max_points))
        self._obstacle_free_value = 0
        self._obstacle_occupied_value = 253
        self._obstacle_unknown_value = 255
        self._obstacle_inflation_min = 1
        self._obstacle_inflation_max = 252
        self._obstacle_width = max(64, int(round(self._obstacle_window_m / self._obstacle_resolution)))
        self._obstacle_height = self._obstacle_width
        self._obstacle_cell_count = self._obstacle_width * self._obstacle_height
        self._obstacle_cells: List[int] = [self._obstacle_unknown_value for _ in range(self._obstacle_cell_count)]
        self._obstacle_last_seen: List[float] = [0.0 for _ in range(self._obstacle_cell_count)]
        self._obstacle_origin = {"x": -self._obstacle_window_m * 0.5, "y": -self._obstacle_window_m * 0.5, "yaw": 0.0}
        configured_costmap_topics = topics.get("costmap_topics", []) or []
        self._costmap_topics = self._unique_topics(
            [self._costmap_topic, "/local_costmap/costmap", "/global_costmap/costmap"] + list(configured_costmap_topics)
        )

        self._scan_sub: Any = None
        self.ensure_scan_subscription(self._scan_topic)
        self._odom_sub = node.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = None
        self._set_map_subscription(self._map_topic)
        self._costmap_sub: Any = None
        self.ensure_costmap_subscription(self._costmap_topic)
        self._tf_sub = node.create_subscription(TFMessage, self._tf_topic, self._on_tf, 30)
        self._tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._tf_static_sub = node.create_subscription(
            TFMessage, self._tf_static_topic, self._on_tf_static, self._tf_static_qos
        )

        self._path_subs: Dict[str, Any] = {}
        self._path_payloads: Dict[str, Dict[str, Any]] = {}
        for topic in self._path_topics:
            self.ensure_path_subscription(topic)

        self._pointcloud_sub: Any = None
        self._pointcloud_payload_lock = threading.Lock()
        self._latest_pointcloud_payload: Optional[Dict[str, Any]] = None
        if self._pointcloud_topic:
            self.ensure_pointcloud_subscription(self._pointcloud_topic)

        self._camera_raw_subs: Dict[str, Any] = {}
        for topic in self._camera_topics:
            self.ensure_camera_raw_subscription(topic)

        self._camera_stream_subs: Dict[str, Any] = {}
        self._camera_stream_frames: Dict[str, Dict[str, Any]] = {}
        self._camera_stream_lock = threading.Lock()
        # Some camera stacks publish CompressedImage with RELIABLE QoS while others
        # use BEST_EFFORT. We subscribe with both to avoid refresh stalls caused by
        # QoS-specific delivery behavior on certain DDS/camera combinations.
        self._camera_stream_qos_profiles = [
            qos_profile_sensor_data,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        ]

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

        self._camera_info_sub: Any = None
        self._set_camera_info_subscription(self._camera_info_topic)

        self._detection_sub: Any = None
        if Detection3DArray is not None and self._detection_topic:
            self.ensure_detection_subscription(self._detection_topic)

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
        self._map_payload_lock = threading.Lock()
        self._latest_map_payload: Optional[Dict[str, Any]] = None
        self._obstacle_payload_lock = threading.Lock()
        self._latest_obstacle_payload: Optional[Dict[str, Any]] = None
        self._latest_nav2_obstacle_payload: Optional[Dict[str, Any]] = None
        self._latest_computed_obstacle_payload: Optional[Dict[str, Any]] = None
        self._camera_raw_lock = threading.Lock()
        self._camera_raw_state: Dict[str, Dict[str, Any]] = {}
        self._latest_camera_info_lock = threading.Lock()
        self._latest_camera_info: Optional[Dict[str, Any]] = None
        self._latest_odom_lock = threading.Lock()
        self._latest_odom_pose: Optional[Dict[str, Any]] = None
        self._latest_detection_payload_lock = threading.Lock()
        self._latest_detection_payload: Optional[Dict[str, Any]] = None
        self._latest_depth_payload_lock = threading.Lock()
        self._latest_depth_payload: Optional[Dict[str, Any]] = None

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

    def _discover_topics(self, ros_type: str) -> List[str]:
        topics: List[str] = []
        try:
            for topic_name, type_names in self._node.get_topic_names_and_types():
                if ros_type in type_names:
                    topics.append(topic_name)
        except Exception:
            return []
        return sorted(self._unique_topics(topics))

    def _publisher_count(self, topic_name: str) -> int:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return 0
        try:
            return len(self._node.get_publishers_info_by_topic(topic_name))
        except Exception:
            return 0

    def _topic_options(self, ros_type: str, selected: str = "", defaults: Optional[Sequence[str]] = None) -> List[Dict[str, Any]]:
        candidates = []
        if defaults:
            candidates.extend(list(defaults))
        if selected:
            candidates.append(selected)
        candidates.extend(self._discover_topics(ros_type))
        topics = self._unique_topics(candidates)
        return [
            {
                "topic": topic,
                "publisher_count": self._publisher_count(topic),
            }
            for topic in topics
        ]

    def ensure_scan_subscription(self, topic_name: str) -> Dict[str, Any]:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}
        if self._scan_sub is not None and self._scan_topic == topic_name:
            return {"ok": True, "topic": topic_name}

        if self._scan_sub is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_subscription(self._scan_sub)
            self._scan_sub = None

        self._scan_sub = self._node.create_subscription(
            LaserScan,
            topic_name,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._scan_topic = topic_name
        return {"ok": True, "topic": topic_name}

    def ensure_camera_raw_subscription(self, topic_name: str) -> Dict[str, Any]:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}
        if topic_name in self._camera_raw_subs:
            return {"ok": True, "topic": topic_name}

        self._camera_raw_subs[topic_name] = self._node.create_subscription(
            Image,
            topic_name,
            self._make_camera_callback(topic_name),
            qos_profile_sensor_data,
        )
        if topic_name not in self._camera_topics:
            self._camera_topics.append(topic_name)
        return {"ok": True, "topic": topic_name}

    def _derive_camera_info_topic(self, rgb_topic: str) -> str:
        topic = (rgb_topic or "").strip()
        if not topic:
            return ""
        if topic.endswith("/image_raw"):
            return topic.replace("/image_raw", "/camera_info")
        if topic.endswith("/image_rect"):
            return topic.replace("/image_rect", "/camera_info")
        if topic.endswith("/compressed"):
            return topic.replace("/compressed", "/camera_info")
        return topic.rstrip("/") + "/camera_info"

    def _set_camera_info_subscription(self, topic_name: str) -> None:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return
        if self._camera_info_sub is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_subscription(self._camera_info_sub)
            self._camera_info_sub = None
        self._camera_info_topic = topic_name
        self._camera_info_sub = self._node.create_subscription(
            CameraInfo,
            topic_name,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

    def ensure_detection_subscription(self, topic_name: str) -> Dict[str, Any]:
        if Detection3DArray is None:
            return {"ok": False, "error": "vision_msgs_unavailable"}

        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}
        if self._detection_sub is not None and self._detection_topic == topic_name:
            return {"ok": True, "topic": topic_name}

        if self._detection_sub is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_subscription(self._detection_sub)
            self._detection_sub = None

        self._detection_sub = self._node.create_subscription(
            Detection3DArray,
            topic_name,
            self._on_detections,
            qos_profile_sensor_data,
        )
        self._detection_topic = topic_name
        return {"ok": True, "topic": topic_name}

    def _set_map_subscription(self, topic_name: str) -> None:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return
        if self._map_sub is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_subscription(self._map_sub)
        self._map_topic = topic_name
        self._map_sub = self._node.create_subscription(
            OccupancyGrid,
            topic_name,
            self._on_map,
            self._map_qos,
        )

    def ensure_costmap_subscription(self, topic_name: str) -> Dict[str, Any]:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}
        if self._costmap_sub is not None and self._costmap_topic == topic_name:
            return {"ok": True, "topic": topic_name}

        if self._costmap_sub is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_subscription(self._costmap_sub)
            self._costmap_sub = None

        self._costmap_sub = self._node.create_subscription(
            OccupancyGrid,
            topic_name,
            self._on_costmap,
            self._map_qos,
        )
        self._costmap_topic = topic_name
        if topic_name not in self._costmap_topics:
            self._costmap_topics.append(topic_name)
        return {"ok": True, "topic": topic_name}

    def _make_path_callback(self, topic_name: str) -> Callable[[Path], None]:
        key = f"path:{topic_name}"

        def _cb(msg: Path) -> None:
            self._mark_topic("path")
            self._mark_topic(key)
            payload = self._build_path_payload(topic_name, msg)
            with self._lock:
                self._path_payloads[topic_name] = payload
                if not self._selected_path_topic:
                    self._selected_path_topic = topic_name

        return _cb

    def ensure_path_subscription(self, topic_name: str) -> None:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return
        if topic_name in self._path_subs:
            return
        self._path_subs[topic_name] = self._node.create_subscription(
            Path,
            topic_name,
            self._make_path_callback(topic_name),
            10,
        )
        if topic_name not in self._path_topics:
            self._path_topics.append(topic_name)

    def ensure_pointcloud_subscription(self, topic_name: str) -> Dict[str, Any]:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}

        if self._pointcloud_sub is not None and self._pointcloud_topic == topic_name:
            return {"ok": True, "topic": topic_name}

        if self._pointcloud_sub is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_subscription(self._pointcloud_sub)
            self._pointcloud_sub = None

        self._pointcloud_sub = self._node.create_subscription(
            PointCloud2,
            topic_name,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self._pointcloud_topic = topic_name
        return {"ok": True, "topic": topic_name}

    def _mark_topic(self, key: str) -> None:
        self._rate[key].tick()
        self._last_topic_unix[key] = time.time()

    def _on_scan(self, msg: LaserScan) -> None:
        self._mark_topic("scan")
        payload = self._build_scan_payload(msg)
        with self._scan_payload_lock:
            self._latest_scan_payload = payload
        now = time.time()
        if (now - self._last_obstacle_compute_unix) >= self._obstacle_compute_period:
            self._last_obstacle_compute_unix = now
            self._update_obstacle_map_from_scan(msg)

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

        map_points, _ = self._scan_points_in_frame(
            self._map_frame,
            msg.header.frame_id,
            points,
            max_points=self._scan_overlay_points,
        )

        return {
            "stamp_unix": time.time(),
            "frame_id": msg.header.frame_id,
            "sample_count": count,
            "angle_min": round(float(msg.angle_min), 6),
            "angle_max": round(float(msg.angle_max), 6),
            "angle_increment": round(float(msg.angle_increment), 6),
            "range_min": round(float(msg.range_min), 4),
            "range_max": round(float(msg.range_max), 4),
            "point_count": len(points),
            "points": points,
            "ranges_preview": [round(float(v), 4) for v in list(msg.ranges[:24])],
            "map_frame": self._map_frame,
            "point_count_map": len(map_points),
            "points_map": map_points,
        }

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._mark_topic("map")
        payload = self._build_map_payload(msg)
        with self._map_payload_lock:
            self._latest_map_payload = payload

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._mark_topic("costmap")
        self._mark_topic("obstacle_map")
        if self._costmap_topic:
            self._mark_topic(f"costmap:{self._costmap_topic}")
            self._mark_topic(f"obstacle_map:{self._costmap_topic}")
        payload = self._build_nav2_obstacle_payload(msg, self._costmap_topic or msg.header.frame_id)
        with self._obstacle_payload_lock:
            self._latest_nav2_obstacle_payload = payload
            if self._obstacle_source_mode in {"auto", "nav2"}:
                self._latest_obstacle_payload = payload

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._mark_topic("pointcloud")
        if self._pointcloud_topic:
            self._mark_topic(f"pointcloud:{self._pointcloud_topic}")
        payload = self._build_pointcloud_payload(msg, self._pointcloud_topic or msg.header.frame_id)
        with self._pointcloud_payload_lock:
            self._latest_pointcloud_payload = payload

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        value = float(angle)
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    @staticmethod
    def _compose_transform(parent_to_mid: Dict[str, float], mid_to_child: Dict[str, float]) -> Dict[str, float]:
        yaw = float(parent_to_mid.get("yaw", 0.0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x = float(parent_to_mid.get("x", 0.0)) + cos_yaw * float(mid_to_child.get("x", 0.0)) - sin_yaw * float(mid_to_child.get("y", 0.0))
        y = float(parent_to_mid.get("y", 0.0)) + sin_yaw * float(mid_to_child.get("x", 0.0)) + cos_yaw * float(mid_to_child.get("y", 0.0))
        return {
            "x": x,
            "y": y,
            "yaw": RosAdapters._normalize_angle(yaw + float(mid_to_child.get("yaw", 0.0))),
            "stamp_unix": min(
                float(parent_to_mid.get("stamp_unix", time.time())),
                float(mid_to_child.get("stamp_unix", time.time())),
            ),
        }

    @staticmethod
    def _invert_transform(transform: Dict[str, float]) -> Dict[str, float]:
        yaw = float(transform.get("yaw", 0.0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = float(transform.get("x", 0.0))
        ty = float(transform.get("y", 0.0))
        inv_yaw = RosAdapters._normalize_angle(-yaw)
        return {
            "x": -(cos_yaw * tx + sin_yaw * ty),
            "y": sin_yaw * tx - cos_yaw * ty,
            "yaw": inv_yaw,
            "stamp_unix": float(transform.get("stamp_unix", time.time())),
        }

    @staticmethod
    def _map_cell_value(value: int) -> int:
        cell = int(value)
        if cell < 0:
            return 255
        return max(0, min(100, cell))

    def _encode_map_rle(self, values: Sequence[int]) -> List[List[int]]:
        if not values:
            return []
        out: List[List[int]] = []
        run_val = self._map_cell_value(values[0])
        run_count = 1
        for raw in values[1:]:
            value = self._map_cell_value(raw)
            if value == run_val and run_count < 65535:
                run_count += 1
                continue
            out.append([run_val, run_count])
            run_val = value
            run_count = 1
        out.append([run_val, run_count])
        return out

    @staticmethod
    def _encode_u8_rle(values: Sequence[int]) -> List[List[int]]:
        if not values:
            return []
        out: List[List[int]] = []
        run_val = max(0, min(255, int(values[0])))
        run_count = 1
        for raw in values[1:]:
            value = max(0, min(255, int(raw)))
            if value == run_val and run_count < 65535:
                run_count += 1
                continue
            out.append([run_val, run_count])
            run_val = value
            run_count = 1
        out.append([run_val, run_count])
        return out

    def _obstacle_value_mapping(self) -> Dict[str, int]:
        return {
            "free": self._obstacle_free_value,
            "occupied": self._obstacle_occupied_value,
            "unknown": self._obstacle_unknown_value,
            "inflation_min": self._obstacle_inflation_min,
            "inflation_max": self._obstacle_inflation_max,
        }

    def _nav_cost_to_u8(self, value: int) -> int:
        if value < 0:
            return self._obstacle_unknown_value
        if value == 0:
            return self._obstacle_free_value
        if value >= 100:
            return self._obstacle_occupied_value
        scaled = int(round((float(value) / 100.0) * self._obstacle_occupied_value))
        return max(self._obstacle_inflation_min, min(self._obstacle_occupied_value, scaled))

    def _build_map_payload(self, msg: OccupancyGrid) -> Dict[str, Any]:
        origin = msg.info.origin
        yaw = self._quat_to_yaw(
            float(origin.orientation.x),
            float(origin.orientation.y),
            float(origin.orientation.z),
            float(origin.orientation.w),
        )
        rle = self._encode_map_rle(msg.data)
        return {
            "stamp_unix": time.time(),
            "frame_id": msg.header.frame_id or self._map_frame,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin": {
                "x": float(origin.position.x),
                "y": float(origin.position.y),
                "yaw": yaw,
            },
            "encoding": "rle_u8",
            "cell_count": len(msg.data),
            "data_rle": rle,
        }

    def _build_nav2_obstacle_payload(self, msg: OccupancyGrid, topic_name: str) -> Dict[str, Any]:
        origin = msg.info.origin
        yaw = self._quat_to_yaw(
            float(origin.orientation.x),
            float(origin.orientation.y),
            float(origin.orientation.z),
            float(origin.orientation.w),
        )
        values = [self._nav_cost_to_u8(int(raw)) for raw in msg.data]
        return {
            "stamp_unix": time.time(),
            "topic": topic_name,
            "source": "nav2",
            "frame_id": msg.header.frame_id or self._map_frame,
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin": {
                "x": float(origin.position.x),
                "y": float(origin.position.y),
                "yaw": yaw,
            },
            "encoding": "rle_u8",
            "cell_count": len(values),
            "data_rle": self._encode_u8_rle(values),
            "value_mapping": self._obstacle_value_mapping(),
            "depth_fusion_enabled": self._obstacle_depth_fusion_enabled,
            "warnings": [],
        }

    def _build_path_payload(self, topic_name: str, msg: Path, max_points: int = 1200) -> Dict[str, Any]:
        points: List[List[float]] = []
        total = len(msg.poses)
        step = max(1, total // max(1, max_points))
        for idx, pose_stamped in enumerate(msg.poses):
            if idx % step != 0:
                continue
            pose = pose_stamped.pose
            points.append([round(float(pose.position.x), 4), round(float(pose.position.y), 4)])
            if len(points) >= max_points:
                break

        return {
            "stamp_unix": time.time(),
            "topic": topic_name,
            "frame_id": msg.header.frame_id or self._map_frame,
            "point_count": len(points),
            "points": points,
        }

    @staticmethod
    def _median3(a: float, b: float, c: float) -> float:
        vals = [a, b, c]
        vals.sort()
        return vals[1]

    def _scan_points_for_obstacle(self, msg: LaserScan) -> List[List[float]]:
        points: List[List[float]] = []
        values = list(msg.ranges)
        if not values:
            return points

        max_range = min(float(msg.range_max), self._obstacle_max_range_m)
        step = max(1, len(values) // max(1, self._obstacle_scan_max_points))
        angle = float(msg.angle_min)
        for idx, raw in enumerate(values):
            if idx % step != 0:
                angle += float(msg.angle_increment)
                continue
            r = float(raw)
            valid = math.isfinite(r) and (r >= float(msg.range_min)) and (r <= max_range)
            if not valid:
                angle += float(msg.angle_increment)
                continue

            if idx > 0 and idx < len(values) - 1:
                prev = float(values[idx - 1])
                nxt = float(values[idx + 1])
                neighbors_ok = (
                    math.isfinite(prev)
                    and math.isfinite(nxt)
                    and prev >= float(msg.range_min)
                    and nxt >= float(msg.range_min)
                )
                if neighbors_ok:
                    med = self._median3(prev, r, nxt)
                    if abs(r - med) > self._obstacle_outlier_threshold_m:
                        angle += float(msg.angle_increment)
                        continue

            points.append([math.cos(angle) * r, math.sin(angle) * r])
            angle += float(msg.angle_increment)

        return points

    def _transform_scan_points(
        self,
        target_frame: str,
        source_frame: str,
        points: Sequence[Sequence[float]],
    ) -> tuple[List[List[float]], tuple[float, float], bool]:
        source_frame = (source_frame or "").strip("/")
        target_frame = (target_frame or "").strip("/")
        if not source_frame or not target_frame:
            return [[float(p[0]), float(p[1])] for p in points if len(p) >= 2], (0.0, 0.0), False
        if source_frame == target_frame:
            return [[float(p[0]), float(p[1])] for p in points if len(p) >= 2], (0.0, 0.0), True

        tf = self.lookup_transform(target_frame, source_frame)
        if tf is None:
            return [[float(p[0]), float(p[1])] for p in points if len(p) >= 2], (0.0, 0.0), False

        yaw = float(tf.get("yaw", 0.0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = float(tf.get("x", 0.0))
        ty = float(tf.get("y", 0.0))
        transformed: List[List[float]] = []
        for point in points:
            if len(point) < 2:
                continue
            x = float(point[0])
            y = float(point[1])
            transformed.append([tx + cos_yaw * x - sin_yaw * y, ty + sin_yaw * x + cos_yaw * y])
        return transformed, (tx, ty), True

    def _cell_of(
        self,
        x: float,
        y: float,
        origin_x: float,
        origin_y: float,
    ) -> tuple[int, int]:
        col = int((x - origin_x) / self._obstacle_resolution)
        row = int((y - origin_y) / self._obstacle_resolution)
        return row, col

    def _cell_index(self, row: int, col: int) -> int:
        if row < 0 or col < 0 or row >= self._obstacle_height or col >= self._obstacle_width:
            return -1
        return row * self._obstacle_width + col

    def _ray_cells(self, row0: int, col0: int, row1: int, col1: int) -> List[tuple[int, int]]:
        cells: List[tuple[int, int]] = []
        dr = abs(row1 - row0)
        dc = abs(col1 - col0)
        sr = 1 if row0 < row1 else -1
        sc = 1 if col0 < col1 else -1
        err = dc - dr
        row = row0
        col = col0
        while True:
            cells.append((row, col))
            if row == row1 and col == col1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                col += sc
            if e2 < dc:
                err += dc
                row += sr
            if len(cells) > (self._obstacle_width + self._obstacle_height):
                break
        return cells

    def _inflate_obstacle_cells(self, observed: List[int], occupied_indices: Sequence[int]) -> None:
        radius_cells = int(round(self._obstacle_inflation_radius_m / self._obstacle_resolution))
        if radius_cells <= 0:
            return
        offsets: List[tuple[int, int, float]] = []
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                dist = math.hypot(float(dr), float(dc))
                if dist > radius_cells:
                    continue
                offsets.append((dr, dc, dist))

        for index in occupied_indices:
            row = index // self._obstacle_width
            col = index % self._obstacle_width
            for dr, dc, dist in offsets:
                nr = row + dr
                nc = col + dc
                nidx = self._cell_index(nr, nc)
                if nidx < 0:
                    continue
                if observed[nidx] == self._obstacle_occupied_value:
                    continue
                ratio = 1.0 - (dist / max(1.0, float(radius_cells)))
                if ratio <= 0.0:
                    continue
                cost = int(
                    round(
                        self._obstacle_inflation_min
                        + ratio * (self._obstacle_inflation_max - self._obstacle_inflation_min)
                    )
                )
                current = observed[nidx]
                if current == self._obstacle_unknown_value or cost > current:
                    observed[nidx] = max(self._obstacle_inflation_min, min(self._obstacle_inflation_max, cost))

    def _build_computed_obstacle_payload(self, msg: LaserScan) -> Dict[str, Any]:
        now = time.time()
        warnings: List[str] = []
        fixed_frame, fixed_warnings = self._resolve_fixed_frame()
        warnings.extend(fixed_warnings)
        scan_frame = (msg.header.frame_id or "").strip("/")
        sensor_points = self._scan_points_for_obstacle(msg)
        points, sensor_origin, resolved = self._transform_scan_points(fixed_frame, scan_frame, sensor_points)

        frame_id = fixed_frame if resolved else (scan_frame or "sensor")
        if sensor_points and not resolved:
            warnings.append(f"Missing TF: cannot transform {scan_frame or 'scan'} to {fixed_frame}")

        center_x = sensor_origin[0]
        center_y = sensor_origin[1]
        pose = self.robot_pose_in_frame(frame_id)
        if pose is not None:
            center_x = float(pose.get("x", center_x))
            center_y = float(pose.get("y", center_y))

        origin_x = center_x - (self._obstacle_width * self._obstacle_resolution) * 0.5
        origin_y = center_y - (self._obstacle_height * self._obstacle_resolution) * 0.5
        self._obstacle_origin = {"x": origin_x, "y": origin_y, "yaw": 0.0}

        observed = [self._obstacle_unknown_value for _ in range(self._obstacle_cell_count)]
        occupied_indices: List[int] = []

        sensor_row, sensor_col = self._cell_of(sensor_origin[0], sensor_origin[1], origin_x, origin_y)
        for point in points:
            px = float(point[0])
            py = float(point[1])
            end_row, end_col = self._cell_of(px, py, origin_x, origin_y)
            end_idx = self._cell_index(end_row, end_col)
            if end_idx < 0:
                continue
            ray = self._ray_cells(sensor_row, sensor_col, end_row, end_col)
            for rr, cc in ray[:-1]:
                idx = self._cell_index(rr, cc)
                if idx < 0:
                    continue
                observed[idx] = self._obstacle_free_value

            observed[end_idx] = self._obstacle_occupied_value
            occupied_indices.append(end_idx)

        self._inflate_obstacle_cells(observed, occupied_indices)

        for idx, value in enumerate(observed):
            if value != self._obstacle_unknown_value:
                self._obstacle_cells[idx] = value
                self._obstacle_last_seen[idx] = now
                continue

            age = now - float(self._obstacle_last_seen[idx])
            if age > self._obstacle_decay_time_sec:
                self._obstacle_cells[idx] = self._obstacle_unknown_value
                continue
            if age > self._obstacle_decay_start_sec:
                current = self._obstacle_cells[idx]
                if current == self._obstacle_unknown_value:
                    continue
                if current <= self._obstacle_free_value:
                    continue
                if current < self._obstacle_occupied_value:
                    self._obstacle_cells[idx] = max(
                        self._obstacle_free_value,
                        int(round(current * 0.88)),
                    )

        return {
            "stamp_unix": now,
            "topic": self._costmap_topic,
            "source": "computed",
            "frame_id": frame_id,
            "width": self._obstacle_width,
            "height": self._obstacle_height,
            "resolution": self._obstacle_resolution,
            "origin": dict(self._obstacle_origin),
            "encoding": "rle_u8",
            "cell_count": self._obstacle_cell_count,
            "data_rle": self._encode_u8_rle(self._obstacle_cells),
            "value_mapping": self._obstacle_value_mapping(),
            "depth_fusion_enabled": self._obstacle_depth_fusion_enabled,
            "warnings": warnings,
        }

    def _update_obstacle_map_from_scan(self, msg: LaserScan) -> None:
        self._mark_topic("obstacle_map")
        if self._costmap_topic:
            self._mark_topic(f"obstacle_map:{self._costmap_topic}")
        computed = self._build_computed_obstacle_payload(msg)
        with self._obstacle_payload_lock:
            self._latest_computed_obstacle_payload = computed
            nav2_payload = self._latest_nav2_obstacle_payload

            nav2_age = None
            if nav2_payload is not None:
                nav2_age = max(0.0, time.time() - float(nav2_payload.get("stamp_unix", 0.0)))
            nav2_fresh = nav2_payload is not None and nav2_age is not None and nav2_age < 1.5

            use_nav2 = self._obstacle_source_mode == "nav2" or (
                self._obstacle_source_mode == "auto" and nav2_fresh
            )
            if use_nav2 and nav2_payload is not None:
                self._latest_obstacle_payload = json.loads(json.dumps(nav2_payload))
            else:
                self._latest_obstacle_payload = json.loads(json.dumps(computed))

    def _build_pointcloud_payload(self, msg: PointCloud2, topic_name: str) -> Dict[str, Any]:
        total = int(msg.width) * int(msg.height)
        if total <= 0 or not msg.data:
            return {
                "stamp_unix": time.time(),
                "topic": topic_name,
                "frame_id": msg.header.frame_id,
                "point_count": 0,
                "points": [],
            }

        fields = {field.name: field for field in msg.fields}
        field_x = fields.get("x")
        field_y = fields.get("y")
        field_z = fields.get("z")
        if field_x is None or field_y is None or field_z is None:
            return {
                "stamp_unix": time.time(),
                "topic": topic_name,
                "frame_id": msg.header.frame_id,
                "point_count": 0,
                "points": [],
                "error": "missing_xyz_fields",
            }

        endian = ">" if msg.is_bigendian else "<"
        unpack_f = f"{endian}f"
        max_offset = max(int(field_x.offset), int(field_y.offset), int(field_z.offset))
        point_step = int(msg.point_step)
        data = bytes(msg.data)
        sample_step = max(1, total // max(1, self._pointcloud_max_points))

        points: List[List[float]] = []
        for idx in range(0, total, sample_step):
            base = idx * point_step
            if base + max_offset + 4 > len(data):
                break
            x = struct.unpack_from(unpack_f, data, base + int(field_x.offset))[0]
            y = struct.unpack_from(unpack_f, data, base + int(field_y.offset))[0]
            z = struct.unpack_from(unpack_f, data, base + int(field_z.offset))[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            points.append([round(float(x), 4), round(float(y), 4), round(float(z), 4)])
            if len(points) >= self._pointcloud_max_points:
                break

        return {
            "stamp_unix": time.time(),
            "topic": topic_name,
            "frame_id": msg.header.frame_id,
            "point_count": len(points),
            "points": points,
            "source_points": total,
        }

    def _camera_projection_info(self) -> Optional[Dict[str, Any]]:
        with self._latest_camera_info_lock:
            if self._latest_camera_info is None:
                return None
            return dict(self._latest_camera_info)

    def _project_detection_marker(
        self,
        camera_info: Optional[Dict[str, Any]],
        x: float,
        y: float,
        z: float,
        size_x: float,
        size_y: float,
    ) -> Optional[Dict[str, float]]:
        if camera_info is None:
            return None
        if z <= 0.02:
            return None
        fx = float(camera_info.get("fx", 0.0))
        fy = float(camera_info.get("fy", 0.0))
        cx = float(camera_info.get("cx", 0.0))
        cy = float(camera_info.get("cy", 0.0))
        width = float(camera_info.get("width", 0.0))
        height = float(camera_info.get("height", 0.0))
        if fx <= 0.0 or fy <= 0.0 or width <= 0.0 or height <= 0.0:
            return None

        u = fx * (x / z) + cx
        v = fy * (y / z) + cy
        rect_w = abs(fx * max(0.001, size_x) / z)
        rect_h = abs(fy * max(0.001, size_y) / z)
        return {
            "u": round(float(u), 2),
            "v": round(float(v), 2),
            "u_norm": round(float(u / width), 5),
            "v_norm": round(float(v / height), 5),
            "rect_w": round(float(rect_w), 2),
            "rect_h": round(float(rect_h), 2),
            "frame_width": int(width),
            "frame_height": int(height),
        }

    def _extract_detection_label(self, detection: Any, fallback_idx: int) -> tuple[str, float]:
        class_id = ""
        score = 0.0
        results = list(getattr(detection, "results", []) or [])
        if results:
            hypothesis = getattr(results[0], "hypothesis", results[0])
            class_id = str(getattr(hypothesis, "class_id", "") or "")
            try:
                score = float(getattr(hypothesis, "score", 0.0) or 0.0)
            except (TypeError, ValueError):
                score = 0.0
        if not class_id:
            class_id = str(getattr(detection, "id", "") or f"det_{fallback_idx}")
        return class_id, score

    def _build_detection_payload(self, msg: Any, max_items: int = 120) -> Dict[str, Any]:
        frame_id = (msg.header.frame_id or "").strip("/")
        fixed_frame, warnings = self._resolve_fixed_frame()
        tf_to_fixed = self.lookup_transform(fixed_frame, frame_id) if frame_id else None
        resolved_in_fixed = frame_id == fixed_frame or tf_to_fixed is not None

        if not resolved_in_fixed and frame_id and fixed_frame:
            warnings = list(warnings) + [f"Missing TF: cannot transform {frame_id} to {fixed_frame}"]

        camera_info = self._camera_projection_info()
        detections_payload: List[Dict[str, Any]] = []

        for idx, detection in enumerate(list(msg.detections)[:max_items]):
            bbox = getattr(detection, "bbox", None)
            center = getattr(bbox, "center", None)
            pos = getattr(center, "position", None)
            orient = getattr(center, "orientation", None)
            size = getattr(bbox, "size", None)
            if pos is None or size is None:
                continue

            x = float(getattr(pos, "x", 0.0))
            y = float(getattr(pos, "y", 0.0))
            z = float(getattr(pos, "z", 0.0))
            size_x = max(0.01, float(getattr(size, "x", 0.0)))
            size_y = max(0.01, float(getattr(size, "y", 0.0)))
            size_z = max(0.01, float(getattr(size, "z", 0.0)))
            yaw = 0.0
            if orient is not None:
                yaw = self._quat_to_yaw(
                    float(getattr(orient, "x", 0.0)),
                    float(getattr(orient, "y", 0.0)),
                    float(getattr(orient, "z", 0.0)),
                    float(getattr(orient, "w", 1.0)),
                )
            class_id, score = self._extract_detection_label(detection, idx)

            if resolved_in_fixed:
                tf = tf_to_fixed or {"x": 0.0, "y": 0.0, "yaw": 0.0}
                cos_yaw = math.cos(float(tf.get("yaw", 0.0)))
                sin_yaw = math.sin(float(tf.get("yaw", 0.0)))
                fx = float(tf.get("x", 0.0)) + cos_yaw * x - sin_yaw * y
                fy = float(tf.get("y", 0.0)) + sin_yaw * x + cos_yaw * y
                fyaw = self._normalize_angle(yaw + float(tf.get("yaw", 0.0)))
                render_frame = fixed_frame
            else:
                fx = x
                fy = y
                fyaw = yaw
                render_frame = frame_id or "sensor"

            marker = self._project_detection_marker(camera_info, x, y, z, size_x, size_y)
            detections_payload.append(
                {
                    "id": str(getattr(detection, "id", "") or f"d{idx}"),
                    "class_id": class_id,
                    "score": round(score, 4),
                    "distance_m": round(math.sqrt(x * x + y * y + z * z), 3),
                    "position": {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4)},
                    "size": {"x": round(size_x, 4), "y": round(size_y, 4), "z": round(size_z, 4)},
                    "yaw": round(yaw, 4),
                    # BEV rendering ignores z / pitch / roll by design.
                    "bev": {
                        "x": round(fx, 4),
                        "y": round(fy, 4),
                        "yaw": round(fyaw, 4),
                        "length": round(size_x, 4),
                        "width": round(size_y, 4),
                    },
                    "render_frame": render_frame,
                    "camera_marker": marker,
                }
            )

        return {
            "stamp_unix": time.time(),
            "topic": self._detection_topic,
            "frame_id": frame_id,
            "fixed_frame": fixed_frame,
            "resolved_in_fixed": resolved_in_fixed,
            "warnings": warnings,
            "detection_count": len(detections_payload),
            "detections": detections_payload,
        }

    @staticmethod
    def _stamp_to_unix(stamp_msg: Any) -> float:
        sec = float(getattr(stamp_msg, "sec", 0.0))
        nsec = float(getattr(stamp_msg, "nanosec", 0.0))
        value = sec + (nsec / 1e9)
        if value <= 0.0:
            return time.time()
        return value

    def _on_odom(self, msg: Odometry) -> None:
        self._mark_topic("odom")
        pose = msg.pose.pose
        yaw = self._quat_to_yaw(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        payload = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw,
            "frame_id": (msg.header.frame_id or "odom").strip("/") or "odom",
            "child_frame_id": (msg.child_frame_id or self._base_frame).strip("/") or self._base_frame,
            "stamp_unix": self._stamp_to_unix(msg.header.stamp),
            "twist_linear_x": float(msg.twist.twist.linear.x),
            "twist_angular_z": float(msg.twist.twist.angular.z),
        }
        with self._latest_odom_lock:
            self._latest_odom_pose = payload

    def _on_tf(self, msg: TFMessage) -> None:
        self._mark_topic("tf")
        self._remember_frames(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._mark_topic("tf_static")
        self._remember_frames(msg)

    def _make_camera_callback(self, topic_name: str) -> Callable[[Image], None]:
        key = f"camera:{topic_name}"

        def _cb(msg: Image) -> None:
            self._mark_topic("camera")
            self._mark_topic(key)
            if topic_name == self._depth_topic:
                self._mark_topic("depth")
                self._mark_topic(f"depth:{topic_name}")
                depth_payload = self._build_depth_payload(msg)
                with self._latest_depth_payload_lock:
                    self._latest_depth_payload = depth_payload
            elif topic_name == self._rgb_topic:
                self._mark_topic("rgb")
                self._mark_topic(f"rgb:{topic_name}")

            with self._camera_raw_lock:
                self._camera_raw_state[topic_name] = {
                    "stamp_unix": self._stamp_to_unix(msg.header.stamp),
                    "frame_id": msg.header.frame_id,
                    "encoding": msg.encoding,
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "step": int(msg.step),
                    "topic": topic_name,
                }

        return _cb

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._mark_topic("camera_info")
        if len(msg.k) < 9:
            return
        with self._latest_camera_info_lock:
            self._latest_camera_info = {
                "frame_id": msg.header.frame_id,
                "stamp_unix": self._stamp_to_unix(msg.header.stamp),
                "width": int(msg.width),
                "height": int(msg.height),
                "fx": float(msg.k[0]),
                "fy": float(msg.k[4]),
                "cx": float(msg.k[2]),
                "cy": float(msg.k[5]),
            }

    def _build_depth_payload(self, msg: Image) -> Dict[str, Any]:
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        data = bytes(msg.data)
        if width <= 0 or height <= 0 or step <= 0 or len(data) < 2:
            return {
                "stamp_unix": time.time(),
                "topic": self._depth_topic,
                "frame_id": msg.header.frame_id,
                "encoding": msg.encoding,
                "width": width,
                "height": height,
                "rows": 0,
                "cols": 0,
                "min_mm": None,
                "max_mm": None,
                "data": [],
            }

        step_x = max(1, width // 96)
        step_y = max(1, height // 64)
        byteorder = "big" if bool(msg.is_bigendian) else "little"
        sampled_mm: List[int] = []
        min_mm: Optional[int] = None
        max_mm: Optional[int] = None

        for y in range(0, height, step_y):
            row_base = y * step
            for x in range(0, width, step_x):
                offset = row_base + x * 2
                if offset + 2 > len(data):
                    sampled_mm.append(0)
                    continue
                raw = int.from_bytes(data[offset : offset + 2], byteorder=byteorder, signed=False)
                sampled_mm.append(raw)
                if raw <= 0:
                    continue
                if min_mm is None or raw < min_mm:
                    min_mm = raw
                if max_mm is None or raw > max_mm:
                    max_mm = raw

        if min_mm is None or max_mm is None or max_mm <= min_mm:
            normalized = [0 for _ in sampled_mm]
        else:
            span = float(max_mm - min_mm)
            normalized = [
                0
                if value <= 0
                else int(max(0.0, min(255.0, ((value - min_mm) / span) * 255.0)))
                for value in sampled_mm
            ]

        return {
            "stamp_unix": time.time(),
            "topic": self._depth_topic,
            "frame_id": msg.header.frame_id,
            "encoding": msg.encoding,
            "width": width,
            "height": height,
            "rows": max(1, (height + step_y - 1) // step_y),
            "cols": max(1, (width + step_x - 1) // step_x),
            "step_x": step_x,
            "step_y": step_y,
            "min_mm": min_mm,
            "max_mm": max_mm,
            "data": normalized,
        }

    def _on_detections(self, msg: Any) -> None:
        self._mark_topic("detections")
        if self._detection_topic:
            self._mark_topic(f"detections:{self._detection_topic}")
        payload = self._build_detection_payload(msg)
        with self._latest_detection_payload_lock:
            self._latest_detection_payload = payload

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

        callback = self._make_camera_stream_callback(topic_name)
        subscriptions = [
            self._node.create_subscription(
                CompressedImage,
                topic_name,
                callback,
                qos,
            )
            for qos in self._camera_stream_qos_profiles
        ]
        self._camera_stream_subs[topic_name] = subscriptions

    def _remember_frames(self, msg: TFMessage) -> None:
        with self._lock:
            for transform in msg.transforms:
                parent = transform.header.frame_id.strip("/")
                child = transform.child_frame_id.strip("/")
                if parent:
                    self._tf_frames.add(parent)
                if child:
                    self._tf_frames.add(child)
                if not parent or not child:
                    continue

                trans = transform.transform.translation
                rot = transform.transform.rotation
                stamp_unix = self._stamp_to_unix(transform.header.stamp)

                edge = {
                    "x": float(trans.x),
                    "y": float(trans.y),
                    "yaw": self._quat_to_yaw(float(rot.x), float(rot.y), float(rot.z), float(rot.w)),
                    "stamp_unix": stamp_unix,
                }
                self._tf_graph[parent][child] = edge
                self._tf_graph[child][parent] = self._invert_transform(edge)

    def lookup_transform(
        self,
        parent_frame: str,
        child_frame: str,
        max_depth: int = 12,
    ) -> Optional[Dict[str, float]]:
        parent_frame = (parent_frame or "").strip("/")
        child_frame = (child_frame or "").strip("/")
        if not parent_frame or not child_frame:
            return None
        if parent_frame == child_frame:
            return {"x": 0.0, "y": 0.0, "yaw": 0.0, "stamp_unix": time.time()}

        with self._lock:
            graph = {src: dict(edges) for src, edges in self._tf_graph.items()}

        visited = {parent_frame}
        queue: deque[tuple[str, Dict[str, float], int]] = deque(
            [(parent_frame, {"x": 0.0, "y": 0.0, "yaw": 0.0, "stamp_unix": time.time()}, 0)]
        )
        while queue:
            frame, accum, depth = queue.popleft()
            if depth >= max_depth:
                continue
            for next_frame, edge in graph.get(frame, {}).items():
                if next_frame in visited:
                    continue
                next_tf = self._compose_transform(accum, edge)
                if next_frame == child_frame:
                    return next_tf
                visited.add(next_frame)
                queue.append((next_frame, next_tf, depth + 1))

        return None

    def available_frames(self) -> List[str]:
        with self._lock:
            known = set(self._tf_frames)
        map_payload = self.latest_map_snapshot() or {}
        map_frame = str(map_payload.get("frame_id", self._map_frame)).strip("/")
        if map_frame:
            known.add(map_frame)

        scan_payload = self.latest_scan_snapshot() or {}
        scan_frame = str(scan_payload.get("frame_id", "")).strip("/")
        if scan_frame:
            known.add(scan_frame)

        det_payload = self.latest_detection_snapshot() or {}
        det_frame = str(det_payload.get("frame_id", "")).strip("/")
        if det_frame:
            known.add(det_frame)

        odom = self._latest_odom_snapshot() or {}
        odom_frame = str(odom.get("frame_id", "")).strip("/")
        child_frame = str(odom.get("child_frame_id", "")).strip("/")
        if odom_frame:
            known.add(odom_frame)
        if child_frame:
            known.add(child_frame)

        return sorted([frame for frame in known if frame])

    def set_fixed_frame(self, frame_name: str) -> Dict[str, Any]:
        name = (frame_name or "").strip().strip("/")
        if name and name not in self._fixed_frame_candidates:
            return {"ok": False, "error": "unsupported_fixed_frame"}
        self._selected_fixed_frame = name
        return {"ok": True, "selected_fixed_frame": self._selected_fixed_frame or "auto"}

    def fixed_frame_state(self) -> Dict[str, Any]:
        resolved, warnings = self._resolve_fixed_frame()
        selected = self._selected_fixed_frame or "auto"
        return {
            "selected": selected,
            "resolved": resolved,
            "available": self.available_frames(),
            "options": list(self._fixed_frame_candidates),
            "warnings": warnings,
        }

    def tf_health_status(self) -> Dict[str, Any]:
        fixed = self.fixed_frame_state()
        fixed_frame = str(fixed.get("resolved", ""))
        scan = self.latest_scan_snapshot() or {}
        scan_frame = str(scan.get("frame_id", "")).strip("/")
        scan_ok = bool(scan_frame and (scan_frame == fixed_frame or self.lookup_transform(fixed_frame, scan_frame)))
        det = self.latest_detection_snapshot() or {}
        det_frame = str(det.get("frame_id", "")).strip("/")
        det_ok = bool(det_frame and (det_frame == fixed_frame or self.lookup_transform(fixed_frame, det_frame)))

        missing_pairs: List[str] = []
        if scan_frame and not scan_ok:
            missing_pairs.append(f"{scan_frame}->{fixed_frame}")
        if det_frame and not det_ok:
            missing_pairs.append(f"{det_frame}->{fixed_frame}")

        has_base_link = self._base_frame in set(self.available_frames())
        has_map_odom = self.lookup_transform("map", "odom") is not None or self._map_frame == "odom"
        has_odom_base = self.lookup_transform("odom", self._base_frame) is not None
        has_base_laser = bool(scan_frame and self.lookup_transform(self._base_frame, scan_frame))
        has_base_oak = self.lookup_transform(self._base_frame, "oak-d-base-frame") is not None
        has_base_oak = has_base_oak or self.lookup_transform(
            self._base_frame, "oak_rgb_camera_optical_frame"
        ) is not None

        if not has_odom_base:
            missing_pairs.append(f"odom->{self._base_frame}")
        if scan_frame and not has_base_laser:
            missing_pairs.append(f"{self._base_frame}->{scan_frame}")
        if not has_base_oak:
            missing_pairs.append(f"{self._base_frame}->oak-d-base-frame")

        return {
            "tf_age_sec": self.topic_age_sec("tf"),
            "tf_static_age_sec": self.topic_age_sec("tf_static"),
            "fixed_frame": fixed,
            "scan_to_fixed_ok": scan_ok,
            "detections_to_fixed_ok": det_ok,
            "missing_pairs": missing_pairs,
            "has_map_odom_tf": has_map_odom,
            "has_base_link": has_base_link,
            "has_odom_base_tf": has_odom_base,
            "has_base_laser_tf": has_base_laser,
            "has_base_oak_tf": has_base_oak,
        }

    def _resolve_fixed_frame(self) -> tuple[str, List[str]]:
        warnings: List[str] = []
        available = set(self.available_frames())
        selected = self._selected_fixed_frame.strip("/") if self._selected_fixed_frame else ""
        if selected:
            if selected in available:
                return selected, warnings
            warnings.append(f"Requested fixed frame '{selected}' unavailable")

        for frame in self._fixed_frame_candidates:
            if frame in available:
                return frame, warnings

        fallback = self._base_frame if self._base_frame in available else "base_link"
        warnings.append(f"No preferred fixed frame available; falling back to {fallback}")
        return fallback, warnings

    def _transform_xy_points(
        self,
        points: Sequence[Sequence[float]],
        target_frame: str,
        source_frame: str,
        max_points: int = 500,
    ) -> tuple[List[List[float]], bool]:
        source_frame = (source_frame or "").strip("/")
        target_frame = (target_frame or "").strip("/")
        if not source_frame or not target_frame:
            return [], False

        if target_frame == source_frame:
            return [
                [round(float(point[0]), 4), round(float(point[1]), 4)]
                for point in points[:max_points]
                if len(point) >= 2
            ], True

        tf = self.lookup_transform(target_frame, source_frame)
        if tf is None:
            return [], False

        step = max(1, len(points) // max(1, max_points))
        yaw = float(tf.get("yaw", 0.0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = float(tf.get("x", 0.0))
        ty = float(tf.get("y", 0.0))

        transformed: List[List[float]] = []
        for idx, point in enumerate(points):
            if idx % step != 0:
                continue
            if len(point) < 2:
                continue
            sx = float(point[0])
            sy = float(point[1])
            mx = tx + cos_yaw * sx - sin_yaw * sy
            my = ty + sin_yaw * sx + cos_yaw * sy
            transformed.append([round(mx, 4), round(my, 4)])
            if len(transformed) >= max_points:
                break
        return transformed, True

    def _scan_points_in_frame(
        self,
        target_frame: str,
        scan_frame: str,
        points: Sequence[Sequence[float]],
        max_points: int = 500,
    ) -> tuple[List[List[float]], bool]:
        return self._transform_xy_points(points, target_frame, scan_frame, max_points=max_points)

    def _latest_odom_snapshot(self) -> Optional[Dict[str, Any]]:
        with self._latest_odom_lock:
            if self._latest_odom_pose is None:
                return None
            return dict(self._latest_odom_pose)

    def robot_pose_in_frame(self, frame_name: str) -> Optional[Dict[str, float]]:
        frame_name = (frame_name or "").strip("/")
        if not frame_name:
            return None

        tf = self.lookup_transform(frame_name, self._base_frame)
        if tf is not None:
            return {
                "x": round(float(tf.get("x", 0.0)), 4),
                "y": round(float(tf.get("y", 0.0)), 4),
                "yaw": round(float(tf.get("yaw", 0.0)), 4),
                "stamp_unix": float(tf.get("stamp_unix", time.time())),
                "frame_id": frame_name,
                "base_frame": self._base_frame,
                "source": "tf",
            }

        odom = self._latest_odom_snapshot()
        if odom and str(odom.get("frame_id", "")) == frame_name:
            return {
                "x": round(float(odom.get("x", 0.0)), 4),
                "y": round(float(odom.get("y", 0.0)), 4),
                "yaw": round(float(odom.get("yaw", 0.0)), 4),
                "stamp_unix": float(odom.get("stamp_unix", time.time())),
                "frame_id": frame_name,
                "base_frame": str(odom.get("child_frame_id", self._base_frame)),
                "source": "odom",
            }
        return None

    def robot_pose_in_map(self) -> Optional[Dict[str, float]]:
        return self.robot_pose_in_frame(self._map_frame)

    def topic_rates(self) -> Dict[str, float]:
        keys = [
            "scan",
            "odom",
            "camera",
            "rgb",
            "depth",
            "camera_info",
            "detections",
            "camera_stream",
            "tf",
            "tf_static",
            "map",
            "costmap",
            "obstacle_map",
            "path",
            "pointcloud",
        ]
        rates = {key: self._rate[key].hz() for key in keys}
        for topic in self._camera_topics:
            key = f"camera:{topic}"
            rates[key] = self._rate[key].hz()
            rates[f"rgb:{topic}"] = self._rate[f"rgb:{topic}"].hz()
            rates[f"depth:{topic}"] = self._rate[f"depth:{topic}"].hz()

        if self._detection_topic:
            rates[f"detections:{self._detection_topic}"] = self._rate[
                f"detections:{self._detection_topic}"
            ].hz()

        for topic in self.camera_stream_topics():
            key = f"camera_stream:{topic}"
            rates[key] = self._rate[key].hz()

        for topic in self.path_topics():
            key = f"path:{topic}"
            rates[key] = self._rate[key].hz()

        for topic in self.costmap_topics():
            key = f"costmap:{topic}"
            rates[key] = self._rate[key].hz()
            rates[f"obstacle_map:{topic}"] = self._rate[f"obstacle_map:{topic}"].hz()

        for topic in self.pointcloud_topics():
            key = f"pointcloud:{topic}"
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

    def latest_map_snapshot(self) -> Optional[Dict[str, Any]]:
        with self._map_payload_lock:
            if self._latest_map_payload is None:
                return None
            return json.loads(json.dumps(self._latest_map_payload))

    def latest_obstacle_map_snapshot(self) -> Optional[Dict[str, Any]]:
        with self._obstacle_payload_lock:
            if self._latest_obstacle_payload is None:
                return None
            return json.loads(json.dumps(self._latest_obstacle_payload))

    def latest_pointcloud_snapshot(self) -> Optional[Dict[str, Any]]:
        with self._pointcloud_payload_lock:
            if self._latest_pointcloud_payload is None:
                return None
            return json.loads(json.dumps(self._latest_pointcloud_payload))

    def path_topics(self) -> List[str]:
        discovered = self._discover_topics("nav_msgs/msg/Path")
        for topic in discovered:
            self.ensure_path_subscription(topic)

        combined = list(self._path_subs.keys()) + self._path_topics + discovered
        topics = sorted(self._unique_topics(combined))
        active = [topic for topic in topics if self._publisher_count(topic) > 0]
        if active and (not self._selected_path_topic or self._publisher_count(self._selected_path_topic) == 0):
            self._selected_path_topic = active[0]
        return topics

    def selected_path_payload(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            topic = self._selected_path_topic
            payload = self._path_payloads.get(topic)
            if payload is not None:
                return json.loads(json.dumps(payload))

            if not self._path_payloads:
                return None
            # Fallback to latest path payload.
            latest = max(
                self._path_payloads.values(),
                key=lambda item: float(item.get("stamp_unix", 0.0)),
            )
            return json.loads(json.dumps(latest))

    def select_path_topic(self, topic_name: str) -> Dict[str, Any]:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}
        self.ensure_path_subscription(topic_name)
        with self._lock:
            self._selected_path_topic = topic_name
        return {"ok": True, "topic": topic_name}

    def map_topics(self) -> Dict[str, Any]:
        map_topics = self._discover_topics("nav_msgs/msg/OccupancyGrid")
        scan_topics = self._discover_topics("sensor_msgs/msg/LaserScan")
        path_topics = self.path_topics()
        costmap_topics = self.costmap_topics()
        fixed_state = self.fixed_frame_state()
        return {
            "selected": {
                "map_topic": self._map_topic,
                "costmap_topic": self._costmap_topic,
                "scan_topic": self._scan_topic,
                "path_topic": self._selected_path_topic,
                "map_frame": self._map_frame,
                "base_frame": self._base_frame,
                "fixed_frame": fixed_state.get("selected"),
                "resolved_fixed_frame": fixed_state.get("resolved"),
            },
            "available": {
                "map_topics": map_topics,
                "costmap_topics": costmap_topics,
                "scan_topics": scan_topics,
                "path_topics": path_topics,
                "fixed_frames": fixed_state.get("options", []),
            },
        }

    def costmap_topics(self) -> List[str]:
        discovered = self._discover_topics("nav_msgs/msg/OccupancyGrid")
        candidates = [topic for topic in discovered if "costmap" in topic.lower()]
        candidates.extend(self._costmap_topics)
        combined = self._unique_topics(candidates)
        return sorted([topic for topic in combined if topic])

    def obstacle_map_stream_status(self) -> Dict[str, Any]:
        rates = self.topic_rates()
        payload = self.latest_obstacle_map_snapshot()
        age = self.topic_age_sec("obstacle_map")
        topic = str((payload or {}).get("topic", self._costmap_topic))
        topic_key = f"obstacle_map:{topic}" if topic else "obstacle_map"
        return {
            "selected_topic": self._costmap_topic,
            "available_topics": self.costmap_topics(),
            "source": (payload or {}).get("source", "none"),
            "fps": rates.get(topic_key, rates.get("obstacle_map", 0.0)),
            "age_sec": age,
            "connected": payload is not None and age is not None and age < 2.5,
            "frame_id": (payload or {}).get("frame_id", ""),
            "width": (payload or {}).get("width", 0),
            "height": (payload or {}).get("height", 0),
            "resolution": (payload or {}).get("resolution", 0.0),
            "publisher_count": self._publisher_count(self._costmap_topic),
            "depth_fusion_enabled": bool((payload or {}).get("depth_fusion_enabled", False)),
            "value_mapping": (payload or {}).get("value_mapping", self._obstacle_value_mapping()),
            "warnings": (payload or {}).get("warnings", []),
        }

    def configure_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        changed: Dict[str, Any] = {}

        map_topic = str(payload.get("map_topic", "")).strip()
        if map_topic and map_topic != self._map_topic:
            self._set_map_subscription(map_topic)
            changed["map_topic"] = map_topic

        costmap_topic = str(payload.get("costmap_topic", "")).strip()
        if costmap_topic and costmap_topic != self._costmap_topic:
            result = self.ensure_costmap_subscription(costmap_topic)
            if not result.get("ok", False):
                return result
            changed["costmap_topic"] = costmap_topic

        path_topic = str(payload.get("path_topic", "")).strip()
        if path_topic:
            result = self.select_path_topic(path_topic)
            if not result.get("ok", False):
                return result
            changed["path_topic"] = path_topic

        map_frame = str(payload.get("map_frame", "")).strip().strip("/")
        if map_frame:
            self._map_frame = map_frame
            changed["map_frame"] = map_frame

        base_frame = str(payload.get("base_frame", "")).strip().strip("/")
        if base_frame:
            self._base_frame = base_frame
            changed["base_frame"] = base_frame

        fixed_frame = str(payload.get("fixed_frame", "")).strip().strip("/")
        if fixed_frame or "fixed_frame" in payload:
            result = self.set_fixed_frame(fixed_frame)
            if not result.get("ok", False):
                return result
            changed["fixed_frame"] = result.get("selected_fixed_frame")

        return {
            "ok": True,
            "changed": changed,
            "topics": self.map_topics(),
            "status": self.map_stream_status(),
        }

    def map_overlay_snapshot(self) -> Dict[str, Any]:
        scan = self.latest_scan_snapshot() or {}
        fixed_frame, frame_warnings = self._resolve_fixed_frame()
        pose = self.robot_pose_in_frame(fixed_frame)
        if pose is None:
            pose = {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "stamp_unix": time.time(),
                "frame_id": fixed_frame,
                "base_frame": self._base_frame,
                "source": "origin",
            }
        path = self.selected_path_payload()
        warnings = list(frame_warnings)
        wedge_points: List[List[float]] = []
        if pose:
            px = float(pose.get("x", 0.0))
            py = float(pose.get("y", 0.0))
            pyaw = float(pose.get("yaw", 0.0))
            steps = 24
            radius = 4.2
            half_fov = 0.62
            wedge_points.append([round(px, 4), round(py, 4)])
            for idx in range(steps + 1):
                offset = -half_fov + (idx / steps) * half_fov * 2.0
                wx = px + math.cos(pyaw + offset) * radius
                wy = py + math.sin(pyaw + offset) * radius
                wedge_points.append([round(wx, 4), round(wy, 4)])
        scan_points_sensor = list(scan.get("points", []))
        scan_frame = str(scan.get("frame_id", "")).strip("/")
        scan_points_fixed, scan_resolved = self._scan_points_in_frame(
            fixed_frame,
            scan_frame,
            scan_points_sensor,
            max_points=self._scan_overlay_points,
        )
        if scan_points_sensor and not scan_resolved:
            warnings.append(f"Missing TF: cannot transform {scan_frame or 'scan'} to {fixed_frame}")

        path_points: List[List[float]] = []
        path_frame = ""
        path_resolved = False
        if path:
            path_frame = str(path.get("frame_id", "")).strip("/")
            path_points, path_resolved = self._transform_xy_points(
                path.get("points", []),
                fixed_frame,
                path_frame,
                max_points=1200,
            )
            if not path_points and path.get("points") and not path_resolved:
                warnings.append(f"Missing TF: cannot transform {path_frame or 'path'} to {fixed_frame}")

        detections = self.latest_detection_snapshot() or {}
        detection_items = detections.get("detections", []) if isinstance(detections, dict) else []
        detection_bev = [
            {
                "id": item.get("id", ""),
                "class_id": item.get("class_id", ""),
                "score": item.get("score", 0.0),
                "distance_m": item.get("distance_m"),
                "bev": item.get("bev", {}),
                "camera_marker": item.get("camera_marker"),
            }
            for item in detection_items
            if isinstance(item, dict)
        ]
        if isinstance(detections, dict):
            for warning in detections.get("warnings", []) or []:
                if warning not in warnings:
                    warnings.append(str(warning))

        active_path_topics = [topic for topic in self.path_topics() if self._publisher_count(topic) > 0]
        return {
            "stamp_unix": time.time(),
            "map_frame": self._map_frame,
            "fixed_frame": fixed_frame,
            "selected_fixed_frame": self._selected_fixed_frame or "auto",
            "fixed_frame_warnings": frame_warnings,
            "scan_topic": self._scan_topic,
            "scan_frame": scan_frame,
            "scan_points": scan_points_fixed,
            "scan_points_sensor": scan_points_sensor,
            "scan_point_count": len(scan_points_fixed),
            "scan_sensor_point_count": len(scan_points_sensor),
            "scan_resolved": scan_resolved,
            "scan_age_sec": self.topic_age_sec("scan"),
            "robot_pose": pose,
            "path_topic": path.get("topic") if path else self._selected_path_topic,
            "path_frame": path_frame,
            "path_points": path_points,
            "path_point_count": len(path_points),
            "path_resolved": path_resolved,
            "planner_active": len(active_path_topics) > 0,
            "active_plan_topics": active_path_topics,
            "detection_topic": detections.get("topic", self._detection_topic)
            if isinstance(detections, dict)
            else self._detection_topic,
            "detection_frame": detections.get("frame_id", "")
            if isinstance(detections, dict)
            else "",
            "detection_count": len(detection_bev),
            "detection_resolved": bool(detections.get("resolved_in_fixed", False))
            if isinstance(detections, dict)
            else False,
            "detections": detection_bev,
            "drivable_wedge": wedge_points,
            "warnings": warnings,
            "tf_age_sec": self.topic_age_sec("tf"),
        }

    def map_stream_status(self) -> Dict[str, Any]:
        rates = self.topic_rates()
        age = self.topic_age_sec("map")
        payload = self.latest_map_snapshot()
        fixed = self.fixed_frame_state()
        pose = self.robot_pose_in_frame(str(fixed.get("resolved", self._map_frame)))
        available_map_topics = self._discover_topics("nav_msgs/msg/OccupancyGrid")
        available_path_topics = self.path_topics()
        active_path_topics = [topic for topic in available_path_topics if self._publisher_count(topic) > 0]
        return {
            "topic": self._map_topic,
            "fps": rates.get("map", 0.0),
            "age_sec": age,
            "connected": age is not None and age < 5.0,
            "map_available": payload is not None,
            "frame_id": (payload or {}).get("frame_id", self._map_frame),
            "width": (payload or {}).get("width", 0),
            "height": (payload or {}).get("height", 0),
            "resolution": (payload or {}).get("resolution", 0.0),
            "tf_age_sec": self.topic_age_sec("tf"),
            "scan_rate_hz": rates.get("scan", 0.0),
            "pose": pose,
            "selected_path_topic": self._selected_path_topic,
            "available_map_topics": available_map_topics,
            "available_path_topics": available_path_topics,
            "active_plan_topics": active_path_topics,
            "planner_active": len(active_path_topics) > 0,
            "fixed_frame": fixed,
            "path_rate_hz": rates.get(f"path:{self._selected_path_topic}", 0.0)
            if self._selected_path_topic
            else 0.0,
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
            "sample_count": (payload or {}).get("sample_count", 0),
            "frame_id": (payload or {}).get("frame_id"),
            "range_min": (payload or {}).get("range_min"),
            "range_max": (payload or {}).get("range_max"),
            "angle_min": (payload or {}).get("angle_min"),
            "angle_max": (payload or {}).get("angle_max"),
            "publisher_count": self._publisher_count(self._scan_topic),
            "connected": scan_age is not None and scan_age < 2.0,
        }

    def pointcloud_topics(self) -> List[str]:
        discovered = self._discover_topics("sensor_msgs/msg/PointCloud2")
        combined = self._unique_topics(discovered + [self._pointcloud_topic])
        return sorted([topic for topic in combined if topic])

    def select_pointcloud_topic(self, topic_name: str) -> Dict[str, Any]:
        topic_name = (topic_name or "").strip()
        if not topic_name:
            return {"ok": False, "error": "missing_topic"}
        return self.ensure_pointcloud_subscription(topic_name)

    def pointcloud_stream_status(self) -> Dict[str, Any]:
        topics = self.pointcloud_topics()
        payload = self.latest_pointcloud_snapshot()
        selected = self._pointcloud_topic
        if not selected and topics:
            selected = topics[0]
            self.ensure_pointcloud_subscription(selected)

        topic_key = f"pointcloud:{selected}" if selected else "pointcloud"
        rates = self.topic_rates()
        age = self.topic_age_sec("pointcloud")
        return {
            "selected_topic": selected,
            "available_topics": topics,
            "fps": rates.get(topic_key, rates.get("pointcloud", 0.0)),
            "age_sec": age,
            "connected": age is not None and age < 2.5,
            "publisher_count": self._publisher_count(selected) if selected else 0,
            "frame_id": (payload or {}).get("frame_id"),
            "point_count": (payload or {}).get("point_count", 0),
        }

    def latest_depth_snapshot(self) -> Optional[Dict[str, Any]]:
        with self._latest_depth_payload_lock:
            if self._latest_depth_payload is None:
                return None
            return json.loads(json.dumps(self._latest_depth_payload))

    def latest_detection_snapshot(self) -> Optional[Dict[str, Any]]:
        with self._latest_detection_payload_lock:
            if self._latest_detection_payload is None:
                return None
            return json.loads(json.dumps(self._latest_detection_payload))

    def rgb_raw_status(self) -> Dict[str, Any]:
        rates = self.topic_rates()
        with self._camera_raw_lock:
            sample = dict(self._camera_raw_state.get(self._rgb_topic, {}))
        age = None
        if sample.get("stamp_unix"):
            age = max(0.0, time.time() - float(sample["stamp_unix"]))
        return {
            "topic": self._rgb_topic,
            "fps": rates.get(f"rgb:{self._rgb_topic}", rates.get(f"camera:{self._rgb_topic}", 0.0)),
            "age_sec": age,
            "connected": age is not None and age < 2.5,
            "publisher_count": self._publisher_count(self._rgb_topic),
            "frame_id": sample.get("frame_id"),
            "encoding": sample.get("encoding"),
            "width": sample.get("width"),
            "height": sample.get("height"),
            "camera_info_topic": self._camera_info_topic,
            "camera_info_age_sec": self.topic_age_sec("camera_info"),
        }

    def depth_stream_status(self) -> Dict[str, Any]:
        rates = self.topic_rates()
        payload = self.latest_depth_snapshot()
        age = self.topic_age_sec("depth")
        return {
            "topic": self._depth_topic,
            "fps": rates.get(f"depth:{self._depth_topic}", rates.get("depth", 0.0)),
            "age_sec": age,
            "connected": age is not None and age < 2.5,
            "publisher_count": self._publisher_count(self._depth_topic),
            "frame_id": (payload or {}).get("frame_id"),
            "encoding": (payload or {}).get("encoding"),
            "width": (payload or {}).get("width"),
            "height": (payload or {}).get("height"),
            "min_mm": (payload or {}).get("min_mm"),
            "max_mm": (payload or {}).get("max_mm"),
            "rows": (payload or {}).get("rows", 0),
            "cols": (payload or {}).get("cols", 0),
        }

    def detection_topics(self) -> List[str]:
        if Detection3DArray is None:
            return [self._detection_topic] if self._detection_topic else []
        discovered = self._discover_topics("vision_msgs/msg/Detection3DArray")
        combined = self._unique_topics(discovered + [self._detection_topic])
        return sorted([topic for topic in combined if topic])

    def detection_stream_status(self) -> Dict[str, Any]:
        rates = self.topic_rates()
        payload = self.latest_detection_snapshot()
        age = self.topic_age_sec("detections")
        topic_key = f"detections:{self._detection_topic}" if self._detection_topic else "detections"
        return {
            "topic": self._detection_topic,
            "available_topics": self.detection_topics(),
            "fps": rates.get(topic_key, rates.get("detections", 0.0)),
            "age_sec": age,
            "connected": age is not None and age < 2.0,
            "publisher_count": self._publisher_count(self._detection_topic),
            "frame_id": (payload or {}).get("frame_id"),
            "detection_count": (payload or {}).get("detection_count", 0),
            "resolved_in_fixed": (payload or {}).get("resolved_in_fixed", False),
            "warnings": (payload or {}).get("warnings", []),
            "vision_msgs_available": Detection3DArray is not None,
        }

    def topic_catalog(self) -> Dict[str, Any]:
        return {
            "scan": {
                "selected_topic": self._scan_topic,
                "options": self._topic_options(
                    "sensor_msgs/msg/LaserScan",
                    selected=self._scan_topic,
                    defaults=["/scan"],
                ),
            },
            "rgb": {
                "selected_topic": self._rgb_topic,
                "options": self._topic_options(
                    "sensor_msgs/msg/Image",
                    selected=self._rgb_topic,
                    defaults=["/oak/rgb/image_raw"],
                ),
            },
            "depth": {
                "selected_topic": self._depth_topic,
                "options": self._topic_options(
                    "sensor_msgs/msg/Image",
                    selected=self._depth_topic,
                    defaults=["/oak/stereo/image_raw"],
                ),
            },
            "detections": {
                "selected_topic": self._detection_topic,
                "options": [
                    {
                        "topic": topic,
                        "publisher_count": self._publisher_count(topic),
                    }
                    for topic in self.detection_topics()
                ],
            },
            "map": {
                "selected_topic": self._map_topic,
                "options": self._topic_options(
                    "nav_msgs/msg/OccupancyGrid",
                    selected=self._map_topic,
                    defaults=["/map"],
                ),
            },
            "costmap": {
                "selected_topic": self._costmap_topic,
                "options": [
                    {
                        "topic": topic,
                        "publisher_count": self._publisher_count(topic),
                    }
                    for topic in self.costmap_topics()
                ],
            },
            "pointcloud": {
                "selected_topic": self._pointcloud_topic,
                "options": self._topic_options(
                    "sensor_msgs/msg/PointCloud2",
                    selected=self._pointcloud_topic,
                    defaults=["/cloud_map"],
                ),
            },
            "plans": {
                "selected_topic": self._selected_path_topic,
                "options": [
                    {
                        "topic": topic,
                        "publisher_count": self._publisher_count(topic),
                    }
                    for topic in self.path_topics()
                ],
            },
            "fixed_frame": self.fixed_frame_state(),
        }

    def configure_visualizer(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        changed: Dict[str, Any] = {}

        scan_topic = str(payload.get("scan_topic", "")).strip()
        if scan_topic and scan_topic != self._scan_topic:
            result = self.ensure_scan_subscription(scan_topic)
            if not result.get("ok", False):
                return result
            changed["scan_topic"] = scan_topic

        rgb_topic = str(payload.get("rgb_topic", "")).strip()
        if rgb_topic and rgb_topic != self._rgb_topic:
            self.ensure_camera_raw_subscription(rgb_topic)
            self._rgb_topic = rgb_topic
            changed["rgb_topic"] = rgb_topic
            guessed_info = self._derive_camera_info_topic(rgb_topic)
            if guessed_info:
                self._set_camera_info_subscription(guessed_info)
                changed["camera_info_topic"] = guessed_info
            derived_compressed = self._derived_compressed_topics([rgb_topic])
            if derived_compressed:
                self.ensure_camera_stream_subscription(derived_compressed[0])
                self._camera_stream_topic = derived_compressed[0]
                changed["camera_stream_topic"] = derived_compressed[0]

        depth_topic = str(payload.get("depth_topic", "")).strip()
        if depth_topic and depth_topic != self._depth_topic:
            self.ensure_camera_raw_subscription(depth_topic)
            self._depth_topic = depth_topic
            changed["depth_topic"] = depth_topic

        detection_topic = str(payload.get("detection_topic", "")).strip()
        if detection_topic and detection_topic != self._detection_topic:
            result = self.ensure_detection_subscription(detection_topic)
            if not result.get("ok", False):
                return result
            changed["detection_topic"] = detection_topic

        camera_stream_topic = str(payload.get("camera_stream_topic", "")).strip()
        if camera_stream_topic and camera_stream_topic != self._camera_stream_topic:
            self.ensure_camera_stream_subscription(camera_stream_topic)
            self._camera_stream_topic = camera_stream_topic
            changed["camera_stream_topic"] = camera_stream_topic

        camera_info_topic = str(payload.get("camera_info_topic", "")).strip()
        if camera_info_topic and camera_info_topic != self._camera_info_topic:
            self._set_camera_info_subscription(camera_info_topic)
            changed["camera_info_topic"] = camera_info_topic

        pointcloud_topic = str(payload.get("pointcloud_topic", "")).strip()
        if pointcloud_topic and pointcloud_topic != self._pointcloud_topic:
            result = self.ensure_pointcloud_subscription(pointcloud_topic)
            if not result.get("ok", False):
                return result
            changed["pointcloud_topic"] = pointcloud_topic

        map_topic = str(payload.get("map_topic", "")).strip()
        if map_topic and map_topic != self._map_topic:
            self._set_map_subscription(map_topic)
            changed["map_topic"] = map_topic

        costmap_topic = str(payload.get("costmap_topic", "")).strip()
        if costmap_topic and costmap_topic != self._costmap_topic:
            result = self.ensure_costmap_subscription(costmap_topic)
            if not result.get("ok", False):
                return result
            changed["costmap_topic"] = costmap_topic

        path_topic = str(payload.get("path_topic", "")).strip()
        if path_topic and path_topic != self._selected_path_topic:
            result = self.select_path_topic(path_topic)
            if not result.get("ok", False):
                return result
            changed["path_topic"] = path_topic

        fixed_frame = str(payload.get("fixed_frame", "")).strip().strip("/")
        if fixed_frame or "fixed_frame" in payload:
            result = self.set_fixed_frame(fixed_frame)
            if not result.get("ok", False):
                return result
            changed["fixed_frame"] = result.get("selected_fixed_frame")

        return {
            "ok": True,
            "changed": changed,
            "topic_catalog": self.topic_catalog(),
            "map_status": self.map_stream_status(),
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
            "publisher_count": self._publisher_count(topic) if topic else 0,
            "selected_rgb_topic": self._rgb_topic,
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
            "vision_msgs_available": Detection3DArray is not None,
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
