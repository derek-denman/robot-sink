import datetime as dt
import os
import re
import shutil
import signal
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional


def _slugify(value: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9_-]+", "-", value.strip())
    cleaned = re.sub(r"-+", "-", cleaned).strip("-")
    return cleaned[:48] or "session"


def compute_guided_capture_duration_sec(
    *,
    target_images: int,
    extraction_fps: float,
    duration_override_sec: float,
    safety_buffer_sec: float,
) -> float:
    """Compute planned bag duration for extraction-targeted capture.

    If duration_override_sec is positive, it is used directly.
    Otherwise duration is derived from target_images / extraction_fps plus buffer.
    """
    override = float(duration_override_sec)
    if override > 0.0:
        return override

    if target_images <= 0:
        raise ValueError("target_images must be > 0 when no duration override is provided")
    if extraction_fps <= 0.0:
        raise ValueError("extraction_fps must be > 0")

    duration = (float(target_images) / float(extraction_fps)) + max(0.0, float(safety_buffer_sec))
    return max(1.0, duration)


@dataclass
class BagSession:
    output_dir: Path
    tag: str
    pid: int
    cmd: List[str]
    started_unix: float
    log_file: Path

    def to_dict(self) -> Dict[str, Any]:
        return {
            "output_dir": str(self.output_dir),
            "tag": self.tag,
            "pid": self.pid,
            "cmd": list(self.cmd),
            "started_unix": self.started_unix,
            "log_file": str(self.log_file),
        }


class RosbagRecorder:
    def __init__(self, storage_path: Path, default_topics: Optional[List[str]] = None) -> None:
        self._storage_path = storage_path.expanduser().resolve()
        self._storage_path.mkdir(parents=True, exist_ok=True)
        self._default_topics = default_topics or []

        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._active: Optional[BagSession] = None
        self._ros2_bag_stop_supported: Optional[bool] = None

    @property
    def storage_path(self) -> Path:
        return self._storage_path

    def start(self, tag: str, topics: Optional[List[str]] = None) -> Dict[str, Any]:
        with self._lock:
            if self._proc and self._proc.poll() is None:
                return {"ok": False, "error": "recording_already_active", "active": self._active.to_dict()}

            now = dt.datetime.utcnow()
            safe_tag = _slugify(tag or "untagged")
            session_name = f"{now.strftime('%Y%m%d_%H%M%S')}_{safe_tag}"
            output_dir = self._storage_path / session_name
            log_file = self._storage_path / f"{session_name}.log"

            selected_topics = [t for t in (topics or self._default_topics) if t]
            cmd = ["ros2", "bag", "record", "-o", str(output_dir)]
            if selected_topics:
                cmd.extend(selected_topics)
            else:
                cmd.append("-a")

            log_handle = open(log_file, "a", encoding="utf-8")
            proc = subprocess.Popen(
                cmd,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )

            self._proc = proc
            self._active = BagSession(
                output_dir=output_dir,
                tag=safe_tag,
                pid=proc.pid,
                cmd=cmd,
                started_unix=now.timestamp(),
                log_file=log_file,
            )

            return {"ok": True, "active": self._active.to_dict()}

    def stop(self, timeout_sec: float = 12.0) -> Dict[str, Any]:
        with self._lock:
            if not self._proc or not self._active:
                return {"ok": False, "error": "recording_not_active"}

            proc = self._proc
            session = self._active

            stop_meta: Dict[str, Any] = {}
            stop_cmd = ["ros2", "bag", "stop"]
            if self._bag_stop_supported():
                try:
                    stop_result = subprocess.run(
                        stop_cmd,
                        check=False,
                        capture_output=True,
                        text=True,
                        timeout=4.0,
                    )
                    stop_meta = {
                        "stop_command": " ".join(stop_cmd),
                        "stop_supported": True,
                        "stop_returncode": stop_result.returncode,
                        "stop_stdout": stop_result.stdout[-200:],
                        "stop_stderr": stop_result.stderr[-200:],
                    }
                except (OSError, subprocess.SubprocessError):
                    stop_meta = {
                        "stop_command": " ".join(stop_cmd),
                        "stop_supported": True,
                        "stop_error": "ros2_bag_stop_failed_to_execute",
                    }
            else:
                stop_meta = {
                    "stop_command": " ".join(stop_cmd),
                    "stop_supported": False,
                }

            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

            try:
                proc.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=4.0)
                except subprocess.TimeoutExpired:
                    proc.kill()

            self._proc = None
            self._active = None

            return {
                "ok": True,
                "stopped": session.to_dict(),
                "returncode": proc.returncode,
                **stop_meta,
            }

    def _bag_stop_supported(self) -> bool:
        cached = self._ros2_bag_stop_supported
        if cached is not None:
            return cached

        try:
            result = subprocess.run(
                ["ros2", "bag", "--help"],
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
            help_text = f"{result.stdout}\n{result.stderr}".lower()
            self._ros2_bag_stop_supported = " stop" in help_text or "\nstop" in help_text
        except (OSError, subprocess.SubprocessError):
            self._ros2_bag_stop_supported = False

        return bool(self._ros2_bag_stop_supported)

    def active(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._proc and self._proc.poll() is None and self._active:
                return self._active.to_dict()
            return None

    def list_recent(self, limit: int = 10) -> List[Dict[str, Any]]:
        entries: List[Dict[str, Any]] = []
        for candidate in self._storage_path.glob("*"):
            if not candidate.is_dir():
                continue

            meta = {
                "path": str(candidate),
                "name": candidate.name,
                "mtime_unix": candidate.stat().st_mtime,
                "size_bytes": self._dir_size(candidate),
            }
            entries.append(meta)

        entries.sort(key=lambda x: x["mtime_unix"], reverse=True)
        return entries[: max(1, limit)]

    def status(self) -> Dict[str, Any]:
        return {
            "active": self.active(),
            "storage_path": str(self._storage_path),
        }

    @staticmethod
    def _dir_size(path: Path) -> int:
        total = 0
        for root, _, files in os.walk(path):
            for file_name in files:
                full_path = Path(root) / file_name
                try:
                    total += full_path.stat().st_size
                except OSError:
                    continue
        return total

    def replay_hint(self, bag_path: str) -> str:
        return f"ros2 bag play {bag_path}"

    def storage_usage(self) -> Dict[str, Any]:
        usage = shutil.disk_usage(self._storage_path)
        return {
            "total_bytes": usage.total,
            "used_bytes": usage.used,
            "free_bytes": usage.free,
        }
