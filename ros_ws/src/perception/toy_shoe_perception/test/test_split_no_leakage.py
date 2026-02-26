from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path


def _load_split_module():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "prepare_dataset_split.py"
    )
    spec = importlib.util.spec_from_file_location("prepare_dataset_split", script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_assignments_have_zero_session_overlap() -> None:
    mod = _load_split_module()

    session_ids = ["s01", "s02", "s03", "s04", "s05", "s06", "s07", "s08"]
    assignments = mod.assign_sessions_to_splits(
        session_ids=session_ids,
        seed=42,
        train_ratio=0.7,
        val_ratio=0.15,
        session_meta={},
    )

    mod.validate_no_session_overlap(assignments)

    train = {s for s, split in assignments.items() if split == "train"}
    val = {s for s, split in assignments.items() if split == "val"}
    test = {s for s, split in assignments.items() if split == "test"}

    assert train.isdisjoint(val)
    assert train.isdisjoint(test)
    assert val.isdisjoint(test)


def test_split_builder_reserves_mixed_clutter_in_test(tmp_path: Path) -> None:
    mod = _load_split_module()

    dataset_dir = tmp_path / "dataset"
    (dataset_dir / "images" / "all").mkdir(parents=True)
    (dataset_dir / "labels" / "all").mkdir(parents=True)

    samples = []
    for session_id in ["s01", "s02", "s03", "s07"]:
        sample_id = f"{session_id}_a"
        image_rel = f"images/all/{sample_id}.jpg"
        label_rel = f"labels/all/{sample_id}.txt"
        (dataset_dir / image_rel).write_bytes(b"fake")
        (dataset_dir / label_rel).write_text("", encoding="utf-8")
        samples.append(
            {
                "sample_id": sample_id,
                "session_id": session_id,
                "image_relpath": image_rel,
                "label_relpath": label_rel,
            }
        )

    with (dataset_dir / "samples.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["sample_id", "session_id", "image_relpath", "label_relpath"])
        writer.writeheader()
        writer.writerows(samples)

    session_meta = {
        "s01": {"scene_type": "toy_only"},
        "s02": {"scene_type": "shoe_only"},
        "s03": {"scene_type": "empty_floor"},
        "s07": {"scene_type": "mixed_clutter"},
    }

    assignments = mod.assign_sessions_to_splits(
        session_ids=["s01", "s02", "s03", "s07"],
        seed=42,
        train_ratio=0.7,
        val_ratio=0.15,
        session_meta=session_meta,
    )

    assert assignments["s07"] == "test"

    rows = mod.materialize_split(
        dataset_dir=dataset_dir,
        samples=samples,
        assignments=assignments,
    )
    assert rows

    manifest_sessions = mod.summarize_assignments(assignments)
    assert "s07" in manifest_sessions["test"]
