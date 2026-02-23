import os
from pathlib import Path
from typing import Any, Dict, List, Optional


def _resolve_path(path_text: str, robot_root: Path) -> Path:
    expanded = Path(path_text).expanduser()
    if expanded.is_absolute():
        return expanded
    return (robot_root / expanded).resolve()


def _tail_lines(path: Path, max_lines: int = 80, max_bytes: int = 128 * 1024) -> List[str]:
    if not path.exists() or not path.is_file():
        return []

    size = path.stat().st_size
    with path.open("rb") as f:
        if size > max_bytes:
            f.seek(-max_bytes, os.SEEK_END)
        data = f.read()

    text = data.decode("utf-8", errors="replace")
    lines = text.splitlines()
    return lines[-max(1, int(max_lines)) :]


class ConsoleLogCatalog:
    """Whitelisted log source resolver for console-safe streaming."""

    DEFAULT_STATIC_SOURCES: Dict[str, str] = {
        "robot_console": "jetson/logs/robot-console.log",
        "run_stack": "/tmp/robot_stack_manual.log",
        "foxglove": "jetson/logs/foxglove.log",
        "oakd": "jetson/logs/oakd.log",
        "rplidar": "jetson/logs/rplidar.log",
        "roarm": "jetson/logs/roarm.log",
        "task": "jetson/logs/task.log",
        "bb_bridge": "jetson/logs/bb-bridge.log",
    }

    DYNAMIC_SOURCES = {"nav2", "slam"}

    def __init__(self, robot_root: Path, source_overrides: Optional[Dict[str, str]] = None) -> None:
        self._robot_root = robot_root.resolve()
        self._source_overrides = source_overrides or {}

    def _static_source_map(self) -> Dict[str, Path]:
        merged: Dict[str, str] = dict(self.DEFAULT_STATIC_SOURCES)
        for key, value in self._source_overrides.items():
            if isinstance(key, str) and isinstance(value, str):
                merged[key.strip()] = value.strip()

        return {
            source_id: _resolve_path(path_text, self._robot_root)
            for source_id, path_text in merged.items()
            if source_id
        }

    def _find_latest_ros_log(self, patterns: List[str]) -> Optional[Path]:
        latest_dir = Path.home() / ".ros" / "log" / "latest"
        if not latest_dir.exists():
            return None

        candidates: List[Path] = []
        for item in latest_dir.glob("*.log"):
            lowered = item.name.lower()
            if any(pattern in lowered for pattern in patterns):
                candidates.append(item)

        if not candidates:
            return None

        candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
        return candidates[0]

    def resolve(self, source_id: str) -> Optional[Path]:
        source = (source_id or "").strip()
        if not source:
            return None

        static_map = self._static_source_map()
        if source in static_map:
            return static_map[source]

        if source == "nav2":
            return self._find_latest_ros_log(
                [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "lifecycle_manager_navigation",
                    "amcl",
                    "map_server",
                ]
            )

        if source == "slam":
            return self._find_latest_ros_log(["slam_toolbox"])

        return None

    def list_sources(self) -> List[Dict[str, Any]]:
        static_map = self._static_source_map()
        rows: List[Dict[str, Any]] = []

        for source_id in sorted(static_map.keys()):
            path = static_map[source_id]
            rows.append(
                {
                    "id": source_id,
                    "path": str(path),
                    "exists": path.exists(),
                    "dynamic": False,
                }
            )

        for source_id in sorted(self.DYNAMIC_SOURCES):
            path = self.resolve(source_id)
            rows.append(
                {
                    "id": source_id,
                    "path": str(path) if path else None,
                    "exists": bool(path and path.exists()),
                    "dynamic": True,
                }
            )

        return rows

    def diagnostics_text(self, snapshot: Dict[str, Any], lines_per_source: int = 60) -> str:
        lines: List[str] = []
        lines.append("=== Robot Console Diagnostics ===")
        lines.append("")
        lines.append("Status Snapshot:")
        lines.append(str(snapshot))
        lines.append("")

        for source in self.list_sources():
            source_id = source.get("id", "unknown")
            path_text = source.get("path")
            exists = source.get("exists", False)
            lines.append(f"--- {source_id} ---")
            lines.append(f"path: {path_text}")
            lines.append(f"exists: {exists}")

            if exists and path_text:
                tail = _tail_lines(Path(path_text), max_lines=lines_per_source)
                if tail:
                    lines.extend(tail)
                else:
                    lines.append("(no lines)")
            else:
                lines.append("(source unavailable)")
            lines.append("")

        return "\n".join(lines).rstrip() + "\n"
