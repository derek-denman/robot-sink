import glob
import os
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List


class DiagnosticsProvider:
    def __init__(self, adapters: Any, recorder: Any, bag_path: Path, temp_paths: List[str]) -> None:
        self._adapters = adapters
        self._recorder = recorder
        self._bag_path = bag_path
        self._temp_paths = temp_paths

    def health_summary(self) -> Dict[str, Any]:
        rates = self._adapters.topic_rates()
        tf_health = self._adapters.tf_health_status()
        tf_ok = bool(tf_health.get("scan_to_fixed_ok", False))
        odom_age = self._adapters.topic_age_sec("odom")
        scan_age = self._adapters.topic_age_sec("scan")
        camera_age = self._adapters.topic_age_sec("camera")
        depth_age = self._adapters.topic_age_sec("depth")
        detections_age = self._adapters.topic_age_sec("detections")
        camera_stream = self._adapters.camera_stream_status()
        scan_stream = self._adapters.scan_stream_status()

        return {
            "base_connected": odom_age is not None and odom_age < 2.0,
            "tf_ok": tf_ok,
            "watchdog_status": "ok" if self._adapters.motion_watchdog_enabled() else "idle",
            "bag_active": self._recorder.active() is not None,
            "lidar_rate_hz": rates.get("scan", 0.0),
            "odom_rate_hz": rates.get("odom", 0.0),
            "oak_rate_hz": rates.get("camera", 0.0),
            "depth_rate_hz": rates.get("depth", 0.0),
            "detections_rate_hz": rates.get("detections", 0.0),
            "camera_stream_rate_hz": rates.get(
                f"camera_stream:{camera_stream.get('selected_topic', '')}", 0.0
            ),
            "scan_age_sec": scan_age,
            "odom_age_sec": odom_age,
            "camera_age_sec": camera_age,
            "depth_age_sec": depth_age,
            "detections_age_sec": detections_age,
            "camera_stream_age_sec": camera_stream.get("age_sec"),
            "scan_stream_connected": scan_stream.get("connected", False),
            "camera_stream_connected": camera_stream.get("connected", False),
            "tf_health": tf_health,
            "nav": self._adapters.navigation_status(),
        }

    def reliability_snapshot(self) -> Dict[str, Any]:
        temps = self._read_temperatures_c()
        disk = shutil.disk_usage(self._bag_path)

        return {
            "topic_rates": self._adapters.topic_rates(),
            "usb_disconnect_count": 0,
            "temperatures_c": temps,
            "cpu_temp_c": temps[0] if temps else None,
            "max_temp_c": max(temps) if temps else None,
            "disk_total_bytes": disk.total,
            "disk_used_bytes": disk.used,
            "disk_free_bytes": disk.free,
            "timestamp_unix": time.time(),
        }

    def _read_temperatures_c(self) -> List[float]:
        values: List[float] = []
        paths: List[str] = []
        if self._temp_paths:
            paths.extend(self._temp_paths)
        else:
            paths.extend(glob.glob("/sys/class/thermal/thermal_zone*/temp"))

        for path in paths:
            try:
                raw = Path(path).read_text(encoding="utf-8").strip()
                value = float(raw)
                if value > 250.0:
                    value = value / 1000.0
                values.append(round(value, 2))
            except (OSError, ValueError):
                continue

        return values
