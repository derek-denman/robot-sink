from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


def _load_extract_module():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "extract_rosbag_images.py"
    )
    spec = importlib.util.spec_from_file_location("extract_rosbag_images", script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_stable_selection_is_reproducible_with_reordered_input() -> None:
    mod = _load_extract_module()

    records = [
        mod.FrameDescriptor(topic="/oak/rgb/image_raw/compressed", stamp_ns=100_000_000, source_index=0),
        mod.FrameDescriptor(topic="/oak/rgb/image_raw/compressed", stamp_ns=400_000_000, source_index=1),
        mod.FrameDescriptor(topic="/oak/rgb/image_raw/compressed", stamp_ns=610_000_000, source_index=2),
        mod.FrameDescriptor(topic="/oak/rgb/image_raw/compressed", stamp_ns=1_020_000_000, source_index=3),
        mod.FrameDescriptor(topic="/oak/rgb/image_raw/compressed", stamp_ns=1_100_000_000, source_index=4),
        mod.FrameDescriptor(topic="/oak/rgb/image_raw/compressed", stamp_ns=1_500_000_000, source_index=5),
    ]

    selected_first = mod.select_stable_frames(
        records=records,
        fps=2.0,
        seed=42,
        session_id="s07",
        topic="/oak/rgb/image_raw/compressed",
    )
    selected_second = mod.select_stable_frames(
        records=list(reversed(records)),
        fps=2.0,
        seed=42,
        session_id="s07",
        topic="/oak/rgb/image_raw/compressed",
    )

    assert selected_first == selected_second

    rows_first = [
        {
            "topic": rec.topic,
            "stamp_ns": rec.stamp_ns,
            "image_relpath": f"sessions/s07/images/{index:03d}.jpg",
        }
        for index, rec in enumerate(selected_first)
    ]
    rows_second = [
        {
            "topic": rec.topic,
            "stamp_ns": rec.stamp_ns,
            "image_relpath": f"sessions/s07/images/{index:03d}.jpg",
        }
        for index, rec in enumerate(selected_second)
    ]

    assert mod.compute_frame_list_checksum(rows_first) == mod.compute_frame_list_checksum(rows_second)


def test_deterministic_phase_is_stable_for_same_inputs() -> None:
    mod = _load_extract_module()

    phase_a = mod.deterministic_phase_ns(
        seed=42,
        session_id="s01",
        topic="/oak/rgb/image_raw/compressed",
        interval_ns=500_000_000,
    )
    phase_b = mod.deterministic_phase_ns(
        seed=42,
        session_id="s01",
        topic="/oak/rgb/image_raw/compressed",
        interval_ns=500_000_000,
    )

    assert phase_a == phase_b
    assert 0 <= phase_a < 500_000_000
