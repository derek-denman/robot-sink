#!/usr/bin/env python3
"""Deterministic rosbag frame extractor for toy-vs-shoe I0 dataset creation."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple


DEFAULT_TOPICS = [
    "/oak/rgb/image_raw/compressed",
    "/oak/rgb/image_raw",
    "/oak/stereo/image_raw/compressed",
    "/oak/stereo/image_raw",
]

SUPPORTED_TOPIC_TYPES = {
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/Image",
}


@dataclass(frozen=True)
class FrameDescriptor:
    topic: str
    stamp_ns: int
    source_index: int


class ExtractionError(RuntimeError):
    """Raised when extraction cannot continue."""


def deterministic_phase_ns(seed: int, session_id: str, topic: str, interval_ns: int) -> int:
    if interval_ns <= 0:
        raise ValueError("interval_ns must be positive")
    token = f"{seed}:{session_id}:{topic}".encode("utf-8")
    digest = hashlib.sha256(token).digest()
    return int.from_bytes(digest[:8], "big") % interval_ns


def select_stable_frames(
    records: Sequence[FrameDescriptor],
    fps: float,
    seed: int,
    session_id: str,
    topic: str,
) -> List[FrameDescriptor]:
    if fps <= 0:
        raise ValueError("fps must be > 0")
    if not records:
        return []

    interval_ns = max(1, int(round(1_000_000_000 / fps)))
    phase_ns = deterministic_phase_ns(seed=seed, session_id=session_id, topic=topic, interval_ns=interval_ns)

    ordered = sorted(records, key=lambda r: (r.stamp_ns, r.source_index))
    selected: List[FrameDescriptor] = []
    seen_buckets = set()

    for rec in ordered:
        bucket_id = (rec.stamp_ns - phase_ns) // interval_ns
        if bucket_id in seen_buckets:
            continue
        seen_buckets.add(bucket_id)
        selected.append(rec)

    return selected


def compute_frame_list_checksum(rows: Sequence[Mapping[str, Any]]) -> str:
    payload = [
        {
            "topic": str(row["topic"]),
            "stamp_ns": int(row["stamp_ns"]),
            "image_relpath": str(row["image_relpath"]),
        }
        for row in rows
    ]
    encoded = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def parse_topics_arg(topics: str | None) -> List[str]:
    if topics is None:
        return list(DEFAULT_TOPICS)
    parsed = [t.strip() for t in topics.split(",") if t.strip()]
    if not parsed:
        raise ValueError("--topics resolved to an empty list")
    return sorted(set(parsed))


def sanitize_topic(topic: str) -> str:
    return topic.strip("/").replace("/", "_").replace(":", "_") or "topic"


def discover_bag_sessions(bags_dir: Path) -> List[Path]:
    sessions = [p for p in sorted(bags_dir.iterdir()) if p.is_dir() and (p / "metadata.yaml").exists()]
    return sessions


def infer_storage_id(bag_dir: Path) -> str:
    if any(bag_dir.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def decode_ros_image(msg: Any, msg_type: str) -> Any:
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError as exc:  # pragma: no cover - environment specific
        raise ExtractionError("OpenCV and numpy are required for image decoding") from exc

    if msg_type == "sensor_msgs/msg/CompressedImage":
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if image is None:
            raise ExtractionError("Failed to decode CompressedImage payload")
        return image

    if msg_type != "sensor_msgs/msg/Image":
        raise ExtractionError(f"Unsupported ROS message type for decoding: {msg_type}")

    encoding = str(getattr(msg, "encoding", "")).lower()
    height = int(getattr(msg, "height", 0))
    width = int(getattr(msg, "width", 0))
    step = int(getattr(msg, "step", 0))
    data = bytes(getattr(msg, "data", b""))

    if height <= 0 or width <= 0:
        raise ExtractionError("Invalid sensor_msgs/Image dimensions")

    if encoding in {"bgr8", "rgb8"}:
        channels = 3
    elif encoding in {"mono8", "8uc1"}:
        channels = 1
    else:
        raise ExtractionError(f"Unsupported image encoding: {encoding}")

    min_step = width * channels
    row_step = max(step, min_step)
    expected = height * row_step
    if len(data) < expected:
        raise ExtractionError("Image payload is smaller than expected from dimensions/step")

    arr = np.frombuffer(data[:expected], dtype=np.uint8).reshape((height, row_step))
    arr = arr[:, :min_step]

    if channels == 1:
        mono = arr.reshape((height, width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    color = arr.reshape((height, width, channels))
    if encoding == "rgb8":
        return cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    return color


def iterate_rosbag_images(bag_dir: Path, topics: Sequence[str]) -> Iterable[Tuple[str, int, Any]]:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as exc:  # pragma: no cover - environment specific
        raise ExtractionError(
            "ROS 2 Python APIs are unavailable. Source ROS environment and install rosbag2_py."
        ) from exc

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=infer_storage_id(bag_dir))
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    allowed_topics = set(topics)

    while reader.has_next():
        topic_name, serialized_msg, stamp_ns = reader.read_next()
        if topic_name not in allowed_topics:
            continue

        msg_type = topics_and_types.get(topic_name, "")
        if msg_type not in SUPPORTED_TOPIC_TYPES:
            continue

        msg_cls = get_message(msg_type)
        msg = deserialize_message(serialized_msg, msg_cls)
        image = decode_ros_image(msg, msg_type)
        yield topic_name, int(stamp_ns), image


def extract_session(
    bag_dir: Path,
    out_dir: Path,
    fps: float,
    seed: int,
    topics: Sequence[str],
    jpeg_quality: int,
) -> Dict[str, Any]:
    try:
        import cv2  # type: ignore
    except ImportError as exc:  # pragma: no cover - environment specific
        raise ExtractionError("OpenCV is required to save extracted images") from exc

    session_id = bag_dir.name
    session_root = out_dir / "sessions" / session_id
    images_dir = session_root / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    by_topic: MutableMapping[str, List[Dict[str, Any]]] = defaultdict(list)

    for source_idx, (topic, stamp_ns, image) in enumerate(iterate_rosbag_images(bag_dir, topics)):
        by_topic[topic].append(
            {
                "topic": topic,
                "stamp_ns": int(stamp_ns),
                "source_index": source_idx,
                "image": image,
            }
        )

    selected_rows: List[Dict[str, Any]] = []
    available_by_topic: Dict[str, int] = {}
    selected_by_topic: Dict[str, int] = {}

    for topic in sorted(by_topic):
        topic_frames = sorted(by_topic[topic], key=lambda row: (row["stamp_ns"], row["source_index"]))
        available_by_topic[topic] = len(topic_frames)

        descriptors = [
            FrameDescriptor(topic=topic, stamp_ns=row["stamp_ns"], source_index=index)
            for index, row in enumerate(topic_frames)
        ]
        selected = select_stable_frames(
            records=descriptors,
            fps=fps,
            seed=seed,
            session_id=session_id,
            topic=topic,
        )
        selected_by_topic[topic] = len(selected)

        topic_key = sanitize_topic(topic)
        for local_rank, desc in enumerate(selected):
            frame = topic_frames[desc.source_index]
            out_name = f"{session_id}_{topic_key}_{desc.stamp_ns:019d}_{local_rank:05d}.jpg"
            image_relpath = f"sessions/{session_id}/images/{out_name}"
            image_path = out_dir / image_relpath

            ok = cv2.imwrite(
                str(image_path),
                frame["image"],
                [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)],
            )
            if not ok:
                raise ExtractionError(f"Failed to write image: {image_path}")

            selected_rows.append(
                {
                    "session_id": session_id,
                    "topic": desc.topic,
                    "stamp_ns": int(desc.stamp_ns),
                    "image_relpath": image_relpath,
                }
            )

    selected_rows.sort(key=lambda row: (row["topic"], row["stamp_ns"], row["image_relpath"]))
    checksum = compute_frame_list_checksum(selected_rows)

    meta = {
        "session_id": session_id,
        "bag_uri": str(bag_dir),
        "seed": int(seed),
        "fps": float(fps),
        "topics": sorted(set(topics)),
        "available_count_by_topic": available_by_topic,
        "selected_count_by_topic": selected_by_topic,
        "selected_frames": selected_rows,
        "selected_frame_checksum_sha256": checksum,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }

    (session_root / "meta.json").write_text(json.dumps(meta, indent=2, sort_keys=True), encoding="utf-8")
    return meta


def build_global_index(metas: Sequence[Mapping[str, Any]], out_dir: Path, topics: Sequence[str], fps: float, seed: int) -> None:
    sessions = []
    frames = []

    for meta in metas:
        sessions.append(
            {
                "session_id": meta["session_id"],
                "selected_frame_checksum_sha256": meta["selected_frame_checksum_sha256"],
                "selected_frame_count": len(meta.get("selected_frames", [])),
            }
        )
        frames.extend(meta.get("selected_frames", []))

    frames.sort(key=lambda row: (row["session_id"], row["topic"], row["stamp_ns"], row["image_relpath"]))
    index = {
        "seed": int(seed),
        "fps": float(fps),
        "topics": sorted(set(topics)),
        "sessions": sorted(sessions, key=lambda row: row["session_id"]),
        "frames": frames,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    (out_dir / "session_index.json").write_text(json.dumps(index, indent=2, sort_keys=True), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Deterministically extract images from rosbag sessions")
    parser.add_argument("--bags-dir", type=Path, required=True, help="Directory containing rosbag2 session folders")
    parser.add_argument("--out-dir", type=Path, required=True, help="Output root for extracted frames and metadata")
    parser.add_argument("--fps", type=float, default=2.0, help="Target deterministic sample rate per topic")
    parser.add_argument("--seed", type=int, default=42, help="Seed for deterministic phase offset")
    parser.add_argument(
        "--topics",
        type=str,
        default=None,
        help="Comma-separated image topics (default: oak RGB + stereo raw/compressed)",
    )
    parser.add_argument("--jpeg-quality", type=int, default=95, help="JPEG quality for output images")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.fps <= 0:
        raise SystemExit("--fps must be > 0")
    if args.jpeg_quality < 1 or args.jpeg_quality > 100:
        raise SystemExit("--jpeg-quality must be in [1, 100]")
    if not args.bags_dir.exists() or not args.bags_dir.is_dir():
        raise SystemExit(f"--bags-dir not found or not a directory: {args.bags_dir}")

    topics = parse_topics_arg(args.topics)
    sessions = discover_bag_sessions(args.bags_dir)
    if not sessions:
        raise SystemExit(f"No rosbag session folders with metadata.yaml found in {args.bags_dir}")

    args.out_dir.mkdir(parents=True, exist_ok=True)

    metas: List[Mapping[str, Any]] = []
    for bag_dir in sessions:
        meta = extract_session(
            bag_dir=bag_dir,
            out_dir=args.out_dir,
            fps=float(args.fps),
            seed=int(args.seed),
            topics=topics,
            jpeg_quality=int(args.jpeg_quality),
        )
        metas.append(meta)

    build_global_index(metas=metas, out_dir=args.out_dir, topics=topics, fps=float(args.fps), seed=int(args.seed))

    total_frames = sum(len(meta.get("selected_frames", [])) for meta in metas)
    print(f"Processed {len(metas)} sessions -> {total_frames} extracted frames")
    print(f"Session index: {args.out_dir / 'session_index.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
