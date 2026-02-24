from __future__ import annotations

import math
from pathlib import Path
import re
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def _safe_name(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]", "_", value)


def _build_nodes(context: Any) -> list[Any]:
    config_path = Path(LaunchConfiguration("config_file").perform(context)).expanduser()
    raw = _load_yaml(config_path)
    bridge_params = raw.get("bridge", {}) if isinstance(raw.get("bridge"), dict) else {}
    static_transforms = raw.get("static_transforms", {})
    if not isinstance(static_transforms, dict):
        static_transforms = {}

    nodes: list[Any] = [
        Node(
            package="base_bringup",
            executable="bbb_odom_tf_bridge",
            name="bbb_odom_tf_bridge",
            output="screen",
            parameters=[bridge_params],
        )
    ]

    for key, spec_raw in static_transforms.items():
        if not isinstance(spec_raw, dict):
            continue
        parent = str(spec_raw.get("parent", "")).strip()
        child = str(spec_raw.get("child", "")).strip()
        if not parent or not child:
            continue

        x = float(spec_raw.get("x", 0.0))
        y = float(spec_raw.get("y", 0.0))
        z = float(spec_raw.get("z", 0.0))
        roll = float(spec_raw.get("roll", 0.0))
        pitch = float(spec_raw.get("pitch", 0.0))
        yaw = float(spec_raw.get("yaw", 0.0))
        qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)

        args = [
            f"{x:.6f}",
            f"{y:.6f}",
            f"{z:.6f}",
            f"{qx:.6f}",
            f"{qy:.6f}",
            f"{qz:.6f}",
            f"{qw:.6f}",
            parent,
            child,
        ]
        node_name = _safe_name(f"static_tf_{key}")
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=node_name,
                output="screen",
                arguments=args,
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    default_config = (
        Path(get_package_share_directory("base_bringup")) / "config" / "base_bringup.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(default_config),
                description="base_bringup YAML config (bridge params + static transforms)",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
