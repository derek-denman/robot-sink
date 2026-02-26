#!/usr/bin/env python3
"""Safety gate for toy pickup: blocks on shoe detections in pickup zone."""

from __future__ import annotations

import time
from typing import Any, List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _as_zone(value: Sequence[float]) -> Tuple[float, float, float, float]:
    if len(value) != 4:
        raise ValueError("pickup_zone_xyxy_norm must have 4 values")
    x1, y1, x2, y2 = [float(v) for v in value]
    if x2 <= x1 or y2 <= y1:
        raise ValueError("pickup zone must satisfy x2>x1 and y2>y1")
    return (_clamp01(x1), _clamp01(y1), _clamp01(x2), _clamp01(y2))


class SafetyGateNode(Node):
    """Derive blocked state and toy candidate stream from Detection2DArray."""

    def __init__(self) -> None:
        super().__init__("toy_shoe_safety_gate")

        self.declare_parameter("detections_topic", "/perception/detections_2d")
        self.declare_parameter("toy_candidates_topic", "/perception/toy_candidates")
        self.declare_parameter("shoe_blocked_topic", "/perception/shoe_blocked")
        self.declare_parameter("pickup_zone_xyxy_norm", [0.30, 0.20, 0.95, 0.98])
        self.declare_parameter("input_width", 640)
        self.declare_parameter("input_height", 640)
        self.declare_parameter("toy_class_name", "toy")
        self.declare_parameter("shoe_class_name", "shoe")
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("shoe_block_debounce_hits", 2)
        self.declare_parameter("shoe_block_hold_sec", 0.30)

        self._detections_topic = str(self.get_parameter("detections_topic").value)
        self._toy_candidates_topic = str(self.get_parameter("toy_candidates_topic").value)
        self._shoe_blocked_topic = str(self.get_parameter("shoe_blocked_topic").value)
        self._pickup_zone = _as_zone(self.get_parameter("pickup_zone_xyxy_norm").value)
        self._input_width = max(1, int(self.get_parameter("input_width").value))
        self._input_height = max(1, int(self.get_parameter("input_height").value))
        self._toy_class_name = str(self.get_parameter("toy_class_name").value).strip() or "toy"
        self._shoe_class_name = str(self.get_parameter("shoe_class_name").value).strip() or "shoe"
        self._min_score = float(self.get_parameter("min_score").value)
        self._debounce_hits = max(1, int(self.get_parameter("shoe_block_debounce_hits").value))
        self._hold_sec = max(0.0, float(self.get_parameter("shoe_block_hold_sec").value))

        self._consecutive_shoe_hits = 0
        self._blocked_until = 0.0

        self._toy_pub = self.create_publisher(Detection2DArray, self._toy_candidates_topic, 10)
        self._blocked_pub = self.create_publisher(Bool, self._shoe_blocked_topic, 10)
        self._sub = self.create_subscription(Detection2DArray, self._detections_topic, self._on_detections, 10)

        self.get_logger().info(
            "safety_gate_node started "
            f"detections_topic={self._detections_topic} toy_topic={self._toy_candidates_topic} "
            f"blocked_topic={self._shoe_blocked_topic} debounce_hits={self._debounce_hits} hold_sec={self._hold_sec}"
        )

    def _class_and_score(self, detection: Detection2D) -> Tuple[str, float]:
        if not detection.results:
            return (str(getattr(detection, "id", "")), 0.0)

        hyp = detection.results[0]
        if hasattr(hyp, "hypothesis"):
            return (str(hyp.hypothesis.class_id), float(hyp.hypothesis.score))
        return (str(getattr(hyp, "id", "")), float(getattr(hyp, "score", 0.0)))

    def _center_xy(self, detection: Detection2D) -> Tuple[float, float]:
        center = detection.bbox.center
        if hasattr(center, "x"):
            return float(center.x), float(center.y)
        return float(center.position.x), float(center.position.y)

    def _in_pickup_zone(self, detection: Detection2D) -> bool:
        cx, cy = self._center_xy(detection)
        nx = cx / float(self._input_width)
        ny = cy / float(self._input_height)
        x1, y1, x2, y2 = self._pickup_zone
        return x1 <= nx <= x2 and y1 <= ny <= y2

    def _compute_blocked(self, shoes_in_zone: bool, now: float) -> bool:
        if shoes_in_zone:
            self._consecutive_shoe_hits += 1
            if self._consecutive_shoe_hits >= self._debounce_hits:
                self._blocked_until = max(self._blocked_until, now + self._hold_sec)
        else:
            self._consecutive_shoe_hits = 0

        return (self._consecutive_shoe_hits >= self._debounce_hits) or (now < self._blocked_until)

    def _publish(self, header: Any, blocked: bool, toy_candidates: Sequence[Detection2D]) -> None:
        blocked_msg = Bool()
        blocked_msg.data = bool(blocked)
        self._blocked_pub.publish(blocked_msg)

        candidates = Detection2DArray()
        candidates.header = header
        if not blocked:
            candidates.detections = list(toy_candidates)
        self._toy_pub.publish(candidates)

    def _on_detections(self, msg: Detection2DArray) -> None:
        now = time.monotonic()

        shoes_in_zone = False
        toy_candidates: List[Detection2D] = []

        for det in msg.detections:
            if not self._in_pickup_zone(det):
                continue

            class_name, score = self._class_and_score(det)
            if score < self._min_score:
                continue

            if class_name == self._shoe_class_name:
                shoes_in_zone = True
            elif class_name == self._toy_class_name:
                toy_candidates.append(det)

        blocked = self._compute_blocked(shoes_in_zone=shoes_in_zone, now=now)
        self._publish(header=msg.header, blocked=blocked, toy_candidates=toy_candidates)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
