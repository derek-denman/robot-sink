#!/usr/bin/env python3
"""Bridge BBB encoder telemetry to ROS2 odometry + odom->base_link TF."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
from typing import Any, Iterable, Optional
import urllib.error
import urllib.request

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    value = float(angle)
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def mean_at(values: Iterable[float], indices: list[int]) -> float:
    selected = [float(values[idx]) for idx in indices if idx >= 0]
    if not selected:
        return 0.0
    return sum(selected) / float(len(selected))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


@dataclass
class TelemetrySample:
    timestamp_us: int
    counts: list[int]
    velocity_tps: list[float]
    encoder_online: bool


class BBBOdomTFBridge(Node):
    """Poll BBB status API, integrate planar odom, publish /odom + TF."""

    def __init__(self) -> None:
        super().__init__("bbb_odom_tf_bridge")

        self._status_url = self._declare_string_param(
            "status_url",
            "http://192.168.7.2:8080/api/status",
        )
        self._poll_hz = clamp(self._declare_float_param("poll_hz", 50.0), 2.0, 200.0)
        self._http_timeout_sec = clamp(
            self._declare_float_param("http_timeout_ms", 120.0) / 1000.0,
            0.02,
            2.0,
        )
        self._stale_timeout_sec = clamp(
            self._declare_float_param("telemetry_stale_ms", 350.0) / 1000.0,
            0.05,
            10.0,
        )
        self._wheel_base_m = max(0.01, self._declare_float_param("wheel_base_m", 0.32))
        self._wheel_radius_m = max(0.001, self._declare_float_param("wheel_radius_m", 0.085))
        self._ticks_per_rev = max(1.0, self._declare_float_param("ticks_per_revolution", 4096.0))
        self._max_linear_mps = max(0.05, self._declare_float_param("max_linear_mps", 2.0))
        self._max_angular_rps = max(0.05, self._declare_float_param("max_angular_rps", 6.0))
        self._left_sign = -1.0 if self._declare_float_param("left_sign", 1.0) < 0.0 else 1.0
        self._right_sign = -1.0 if self._declare_float_param("right_sign", 1.0) < 0.0 else 1.0
        self._use_velocity_tps = bool(self._declare_bool_param("use_velocity_tps", True))
        self._max_dt_sec = clamp(self._declare_float_param("max_integration_dt_sec", 0.1), 0.01, 1.0)
        self._odom_frame = self._declare_string_param("odom_frame", "odom").strip("/") or "odom"
        self._base_frame = self._declare_string_param("base_frame", "base_link").strip("/") or "base_link"
        self._publish_tf = bool(self._declare_bool_param("publish_tf", True))
        self._left_indices = self._declare_int_array_param("left_indices", [0, 2])
        self._right_indices = self._declare_int_array_param("right_indices", [1, 3])

        self._odom_pub = self.create_publisher(Odometry, "/odom", 50)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._linear_mps = 0.0
        self._angular_rps = 0.0

        self._last_counts: Optional[list[int]] = None
        self._last_sample_mono: Optional[float] = None
        self._last_bbb_timestamp_us = 0
        self._last_fresh_rx_mono: Optional[float] = None
        self._last_warn: dict[str, float] = {}
        self._stale_active = False

        self._timer = self.create_timer(1.0 / self._poll_hz, self._poll_once)
        self.get_logger().info(
            "bbb_odom_tf_bridge started: "
            f"url={self._status_url} poll_hz={self._poll_hz:.1f} stale_ms={self._stale_timeout_sec * 1000.0:.0f}"
        )
        self.get_logger().info(
            "Only run one instance of bbb_odom_tf_bridge/base_tf.launch.py to avoid competing odom->base_link TF."
        )

    def _declare_string_param(self, name: str, default: str) -> str:
        self.declare_parameter(name, default)
        return str(self.get_parameter(name).value)

    def _declare_float_param(self, name: str, default: float) -> float:
        self.declare_parameter(name, float(default))
        value = self.get_parameter(name).value
        return float(value)

    def _declare_bool_param(self, name: str, default: bool) -> bool:
        self.declare_parameter(name, bool(default))
        return bool(self.get_parameter(name).value)

    def _declare_int_array_param(self, name: str, default: list[int]) -> list[int]:
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        if not isinstance(value, list):
            return list(default)
        cleaned: list[int] = []
        for item in value:
            try:
                cleaned.append(int(item))
            except (TypeError, ValueError):
                continue
        return cleaned or list(default)

    def _warn_throttled(self, key: str, message: str, period_sec: float = 2.5) -> None:
        now = time.monotonic()
        last = self._last_warn.get(key, 0.0)
        if (now - last) < period_sec:
            return
        self._last_warn[key] = now
        self.get_logger().warn(message)

    def _fetch_sample(self) -> Optional[TelemetrySample]:
        req = urllib.request.Request(
            self._status_url,
            headers={"Accept": "application/json"},
            method="GET",
        )
        try:
            with urllib.request.urlopen(req, timeout=self._http_timeout_sec) as response:
                raw = response.read()
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self._warn_throttled("status_fetch", f"BBB status fetch failed: {exc}")
            return None

        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._warn_throttled("status_json", f"BBB status parse failed: {exc}")
            return None

        encoder = payload.get("encoder", {})
        pru = payload.get("pru", {})
        try:
            timestamp_us = int(encoder.get("timestamp_us", 0))
        except (TypeError, ValueError):
            timestamp_us = 0

        counts = encoder.get("counts", [0, 0, 0, 0])
        velocities = encoder.get("velocity_tps", [0, 0, 0, 0])
        if not isinstance(counts, list) or not isinstance(velocities, list):
            return None
        if len(counts) < 2 or len(velocities) < 2:
            return None

        parsed_counts: list[int] = []
        parsed_vel: list[float] = []
        for value in counts:
            try:
                parsed_counts.append(int(value))
            except (TypeError, ValueError):
                parsed_counts.append(0)
        for value in velocities:
            try:
                parsed_vel.append(float(value))
            except (TypeError, ValueError):
                parsed_vel.append(0.0)

        return TelemetrySample(
            timestamp_us=timestamp_us,
            counts=parsed_counts,
            velocity_tps=parsed_vel,
            encoder_online=bool(pru.get("encoder_online", False)),
        )

    def _poll_once(self) -> None:
        now_mono = time.monotonic()
        sample = self._fetch_sample()

        if sample is not None and sample.encoder_online and sample.timestamp_us > 0:
            is_fresh = sample.timestamp_us != self._last_bbb_timestamp_us
            if is_fresh:
                self._integrate_sample(sample, now_mono)
                self._last_bbb_timestamp_us = sample.timestamp_us
                self._last_fresh_rx_mono = now_mono

        stale = self._is_stale(now_mono)
        if stale:
            self._linear_mps = 0.0
            self._angular_rps = 0.0
            if not self._stale_active:
                self._stale_active = True
                self._warn_throttled(
                    "telemetry_stale_start",
                    "BBB telemetry stale; pose integration paused and zero twist published.",
                    period_sec=0.0,
                )
        elif self._stale_active:
            self._stale_active = False
            self.get_logger().info("BBB telemetry fresh again; pose integration resumed.")

        self._publish_odom_and_tf()

    def _is_stale(self, now_mono: float) -> bool:
        if self._last_fresh_rx_mono is None:
            return True
        return (now_mono - self._last_fresh_rx_mono) > self._stale_timeout_sec

    def _integrate_sample(self, sample: TelemetrySample, now_mono: float) -> None:
        if self._last_sample_mono is None:
            self._last_sample_mono = now_mono
            self._last_counts = list(sample.counts)
            return

        dt = now_mono - self._last_sample_mono
        self._last_sample_mono = now_mono
        dt = clamp(dt, 0.0, self._max_dt_sec)
        if dt <= 0.0:
            self._last_counts = list(sample.counts)
            return

        left_tps = 0.0
        right_tps = 0.0

        if self._use_velocity_tps:
            left_tps = mean_at(sample.velocity_tps, self._left_indices)
            right_tps = mean_at(sample.velocity_tps, self._right_indices)

        if (not self._use_velocity_tps) or (
            abs(left_tps) < 1e-5 and abs(right_tps) < 1e-5 and self._last_counts is not None
        ):
            if self._last_counts is not None:
                left_delta = mean_at(
                    [sample.counts[idx] - self._last_counts[idx] for idx in range(len(sample.counts))],
                    self._left_indices,
                )
                right_delta = mean_at(
                    [sample.counts[idx] - self._last_counts[idx] for idx in range(len(sample.counts))],
                    self._right_indices,
                )
                left_tps = left_delta / dt
                right_tps = right_delta / dt

        self._last_counts = list(sample.counts)

        meters_per_tick = (2.0 * math.pi * self._wheel_radius_m) / self._ticks_per_rev
        left_mps = left_tps * meters_per_tick * self._left_sign
        right_mps = right_tps * meters_per_tick * self._right_sign

        linear = 0.5 * (left_mps + right_mps)
        angular = (right_mps - left_mps) / self._wheel_base_m

        if abs(linear) > self._max_linear_mps:
            self._warn_throttled(
                "linear_clamp",
                f"Linear velocity {linear:.3f} m/s exceeds limit; clamping to ±{self._max_linear_mps:.2f}.",
            )
        if abs(angular) > self._max_angular_rps:
            self._warn_throttled(
                "angular_clamp",
                f"Angular velocity {angular:.3f} rad/s exceeds limit; clamping to ±{self._max_angular_rps:.2f}.",
            )
        linear = clamp(linear, -self._max_linear_mps, self._max_linear_mps)
        angular = clamp(angular, -self._max_angular_rps, self._max_angular_rps)

        yaw_mid = self._yaw + 0.5 * angular * dt
        self._x += linear * math.cos(yaw_mid) * dt
        self._y += linear * math.sin(yaw_mid) * dt
        self._yaw = normalize_angle(self._yaw + angular * dt)

        self._linear_mps = linear
        self._angular_rps = angular

    def _publish_odom_and_tf(self) -> None:
        stamp = self.get_clock().now().to_msg()
        quat = yaw_to_quaternion(self._yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = float(self._x)
        odom.pose.pose.position.y = float(self._y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        odom.twist.twist.linear.x = float(self._linear_mps)
        odom.twist.twist.angular.z = float(self._angular_rps)

        # Moderate covariance defaults for wheel-odom.
        odom.pose.covariance = [
            0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.04, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.08,
        ]
        odom.twist.covariance = [
            0.06, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.06, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.12,
        ]

        self._odom_pub.publish(odom)

        if not self._publish_tf:
            return

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = float(self._x)
        tf.transform.translation.y = float(self._y)
        tf.transform.translation.z = 0.0
        tf.transform.rotation = quat
        self._tf_broadcaster.sendTransform(tf)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BBBOdomTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
