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
from sensor_msgs.msg import CompressedImage, Image, LaserScan, PointCloud2
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
        self._tf_graph: Dict[str, Dict[str, Dict[str, float]]] = defaultdict(dict)

        self._scan_topic = topics.get("scan", "/scan")
        self._odom_topic = topics.get("odom", "/odom")
        self._map_topic = topics.get("map", "/map")
        self._map_frame = topics.get("map_frame", "map")
        self._base_frame = topics.get("base_frame", "base_link")
        self._camera_topics = topics.get("camera_topics", []) or [topics.get("camera", "/oak/rgb/image_raw")]
        self._tf_topic = topics.get("tf", "/tf")
        self._tf_static_topic = topics.get("tf_static", "/tf_static")
        configured_path_topics = topics.get("path_topics", [])
        if not configured_path_topics:
            configured_path_topics = [topics.get("path", "/plan"), "/plan", "/global_plan"]
        self._path_topics = self._unique_topics(configured_path_topics)
        self._selected_path_topic = topics.get("path_topic", "") or (
            self._path_topics[0] if self._path_topics else ""
        )

        self._pointcloud_topic = str(topics.get("pointcloud", "")).strip()
        self._pointcloud_max_points = int(config.get("streaming", {}).get("pointcloud_max_points", 4500))
        self._scan_overlay_points = int(config.get("streaming", {}).get("scan_overlay_points", 520))

        self._scan_sub = node.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._odom_sub = node.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = None
        self._set_map_subscription(self._map_topic)
        self._tf_sub = node.create_subscription(TFMessage, self._tf_topic, self._on_tf, 30)
        self._tf_static_sub = node.create_subscription(
            TFMessage, self._tf_static_topic, self._on_tf_static, 30
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
        self._map_payload_lock = threading.Lock()
        self._latest_map_payload: Optional[Dict[str, Any]] = None

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

        map_points = self._scan_points_in_map(msg.header.frame_id, points, max_points=self._scan_overlay_points)

        return {
            "stamp_unix": time.time(),
            "frame_id": msg.header.frame_id,
            "range_min": round(float(msg.range_min), 4),
            "range_max": round(float(msg.range_max), 4),
            "point_count": len(points),
            "points": points,
            "map_frame": self._map_frame,
            "point_count_map": len(map_points),
            "points_map": map_points,
        }

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._mark_topic("map")
        payload = self._build_map_payload(msg)
        with self._map_payload_lock:
            self._latest_map_payload = payload

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
                if not parent or not child:
                    continue

                trans = transform.transform.translation
                rot = transform.transform.rotation
                stamp_msg = transform.header.stamp
                stamp_unix = float(stamp_msg.sec) + float(getattr(stamp_msg, "nanosec", 0)) / 1e9
                if stamp_unix <= 0.0:
                    stamp_unix = time.time()

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

    def _scan_points_in_map(
        self,
        scan_frame: str,
        points: Sequence[Sequence[float]],
        max_points: int = 500,
    ) -> List[List[float]]:
        tf = self.lookup_transform(self._map_frame, scan_frame)
        if tf is None:
            return []

        step = max(1, len(points) // max(1, max_points))
        yaw = float(tf.get("yaw", 0.0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = float(tf.get("x", 0.0))
        ty = float(tf.get("y", 0.0))

        mapped: List[List[float]] = []
        for idx, point in enumerate(points):
            if idx % step != 0:
                continue
            if len(point) < 2:
                continue
            sx = float(point[0])
            sy = float(point[1])
            mx = tx + cos_yaw * sx - sin_yaw * sy
            my = ty + sin_yaw * sx + cos_yaw * sy
            mapped.append([round(mx, 4), round(my, 4)])
            if len(mapped) >= max_points:
                break
        return mapped

    def robot_pose_in_map(self) -> Optional[Dict[str, float]]:
        tf = self.lookup_transform(self._map_frame, self._base_frame)
        if tf is None:
            return None
        return {
            "x": round(float(tf.get("x", 0.0)), 4),
            "y": round(float(tf.get("y", 0.0)), 4),
            "yaw": round(float(tf.get("yaw", 0.0)), 4),
            "stamp_unix": float(tf.get("stamp_unix", time.time())),
            "frame_id": self._map_frame,
            "base_frame": self._base_frame,
        }

    def topic_rates(self) -> Dict[str, float]:
        keys = ["scan", "odom", "camera", "camera_stream", "tf", "tf_static", "map", "path", "pointcloud"]
        rates = {key: self._rate[key].hz() for key in keys}
        for topic in self._camera_topics:
            key = f"camera:{topic}"
            rates[key] = self._rate[key].hz()

        for topic in self.camera_stream_topics():
            key = f"camera_stream:{topic}"
            rates[key] = self._rate[key].hz()

        for topic in self.path_topics():
            key = f"path:{topic}"
            rates[key] = self._rate[key].hz()

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
        return sorted(self._unique_topics(combined))

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
        return {
            "selected": {
                "map_topic": self._map_topic,
                "scan_topic": self._scan_topic,
                "path_topic": self._selected_path_topic,
                "map_frame": self._map_frame,
                "base_frame": self._base_frame,
            },
            "available": {
                "map_topics": map_topics,
                "scan_topics": scan_topics,
                "path_topics": path_topics,
            },
        }

    def configure_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        changed: Dict[str, Any] = {}

        map_topic = str(payload.get("map_topic", "")).strip()
        if map_topic and map_topic != self._map_topic:
            self._set_map_subscription(map_topic)
            changed["map_topic"] = map_topic

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

        return {
            "ok": True,
            "changed": changed,
            "topics": self.map_topics(),
            "status": self.map_stream_status(),
        }

    def map_overlay_snapshot(self) -> Dict[str, Any]:
        scan = self.latest_scan_snapshot() or {}
        pose = self.robot_pose_in_map()
        path = self.selected_path_payload()
        return {
            "stamp_unix": time.time(),
            "map_frame": self._map_frame,
            "scan_topic": self._scan_topic,
            "scan_points": scan.get("points_map", []),
            "scan_point_count": scan.get("point_count_map", 0),
            "scan_age_sec": self.topic_age_sec("scan"),
            "robot_pose": pose,
            "path_topic": path.get("topic") if path else self._selected_path_topic,
            "path_points": path.get("points", []) if path else [],
            "path_point_count": path.get("point_count", 0) if path else 0,
            "tf_age_sec": self.topic_age_sec("tf"),
        }

    def map_stream_status(self) -> Dict[str, Any]:
        rates = self.topic_rates()
        age = self.topic_age_sec("map")
        payload = self.latest_map_snapshot()
        pose = self.robot_pose_in_map()
        available_map_topics = self._discover_topics("nav_msgs/msg/OccupancyGrid")
        available_path_topics = self.path_topics()
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
            "frame_id": (payload or {}).get("frame_id"),
            "point_count": (payload or {}).get("point_count", 0),
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
