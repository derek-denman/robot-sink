#!/usr/bin/env python3
"""Build deterministic session-separated train/val/test splits for toy-vs-shoe I0."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple

CANONICAL_CLASSES = ["toy", "shoe"]


def _stable_hash_int(seed: int, key: str) -> int:
    digest = hashlib.sha256(f"{seed}:{key}".encode("utf-8")).hexdigest()
    return int(digest[:16], 16)


def load_samples(samples_csv: Path) -> List[Dict[str, str]]:
    with samples_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {
            "sample_id",
            "session_id",
            "image_relpath",
            "label_relpath",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"samples.csv missing required columns: {sorted(missing)}")
        rows = [dict(row) for row in reader]
    if not rows:
        raise ValueError("samples.csv contains no rows")
    return rows


def load_capture_manifest(capture_manifest_csv: Optional[Path]) -> Dict[str, Dict[str, str]]:
    if capture_manifest_csv is None:
        return {}
    with capture_manifest_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        out: Dict[str, Dict[str, str]] = {}
        for row in reader:
            session_id = row.get("session_id", "").strip()
            if not session_id:
                continue
            out[session_id] = dict(row)
        return out


def compute_split_counts(n_sessions: int, train_ratio: float, val_ratio: float) -> Dict[str, int]:
    if n_sessions <= 0:
        raise ValueError("No sessions to split")

    if n_sessions == 1:
        return {"train": 1, "val": 0, "test": 0}
    if n_sessions == 2:
        return {"train": 1, "val": 0, "test": 1}

    train_n = max(1, int(round(train_ratio * n_sessions)))
    val_n = max(1, int(round(val_ratio * n_sessions)))

    if train_n + val_n >= n_sessions:
        val_n = 1
        train_n = max(1, n_sessions - 2)

    test_n = n_sessions - train_n - val_n
    if test_n < 1:
        test_n = 1
        if train_n >= val_n and train_n > 1:
            train_n -= 1
        elif val_n > 1:
            val_n -= 1
        else:
            train_n = max(1, train_n - 1)

    return {"train": train_n, "val": val_n, "test": test_n}


def assign_sessions_to_splits(
    session_ids: Sequence[str],
    seed: int,
    train_ratio: float,
    val_ratio: float,
    session_meta: Mapping[str, Mapping[str, str]],
) -> Dict[str, str]:
    ordered = sorted(session_ids, key=lambda sid: (_stable_hash_int(seed, sid), sid))
    counts = compute_split_counts(len(ordered), train_ratio=train_ratio, val_ratio=val_ratio)

    assignment: Dict[str, str] = {}

    mixed_sessions = [
        sid
        for sid in ordered
        if (session_meta.get(sid, {}).get("scene_type", "").strip() == "mixed_clutter")
    ]

    if counts["test"] > 0 and mixed_sessions:
        reserved_test = sorted(mixed_sessions, key=lambda sid: (_stable_hash_int(seed + 17, sid), sid))[0]
        assignment[reserved_test] = "test"
        counts["test"] -= 1

    for sid in ordered:
        if sid in assignment:
            continue

        if counts["train"] > 0:
            assignment[sid] = "train"
            counts["train"] -= 1
            continue

        if counts["val"] > 0:
            assignment[sid] = "val"
            counts["val"] -= 1
            continue

        assignment[sid] = "test"
        if counts["test"] > 0:
            counts["test"] -= 1

    return assignment


def validate_no_session_overlap(assignments: Mapping[str, str]) -> None:
    by_split: MutableMapping[str, set[str]] = {"train": set(), "val": set(), "test": set()}
    for session_id, split in assignments.items():
        if split not in by_split:
            raise ValueError(f"Unknown split '{split}' in assignments")
        by_split[split].add(session_id)

    if by_split["train"] & by_split["val"]:
        raise ValueError("Session overlap detected between train and val")
    if by_split["train"] & by_split["test"]:
        raise ValueError("Session overlap detected between train and test")
    if by_split["val"] & by_split["test"]:
        raise ValueError("Session overlap detected between val and test")


def _reset_split_dirs(dataset_dir: Path) -> None:
    for split in ("train", "val", "test"):
        for kind in ("images", "labels"):
            target = dataset_dir / kind / split
            if target.exists():
                shutil.rmtree(target)
            target.mkdir(parents=True, exist_ok=True)


def _link_or_copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    try:
        dst.hardlink_to(src)
    except OSError:
        shutil.copy2(src, dst)


def materialize_split(
    dataset_dir: Path,
    samples: Sequence[Mapping[str, str]],
    assignments: Mapping[str, str],
) -> List[Dict[str, str]]:
    _reset_split_dirs(dataset_dir)

    rows: List[Dict[str, str]] = []
    for sample in sorted(samples, key=lambda row: (row["session_id"], row["sample_id"])):
        split = assignments[sample["session_id"]]
        src_image = dataset_dir / sample["image_relpath"]
        src_label = dataset_dir / sample["label_relpath"]

        if not src_image.exists():
            raise FileNotFoundError(f"Missing source image: {src_image}")
        if not src_label.exists():
            raise FileNotFoundError(f"Missing source label: {src_label}")

        image_name = Path(sample["image_relpath"]).name
        label_name = Path(sample["label_relpath"]).name
        dst_image_rel = f"images/{split}/{image_name}"
        dst_label_rel = f"labels/{split}/{label_name}"

        _link_or_copy(src_image, dataset_dir / dst_image_rel)
        _link_or_copy(src_label, dataset_dir / dst_label_rel)

        row = dict(sample)
        row["split"] = split
        row["split_image_relpath"] = dst_image_rel
        row["split_label_relpath"] = dst_label_rel
        rows.append(row)

    return rows


def write_ultralytics_data_yaml(dataset_dir: Path) -> None:
    data_yaml = dataset_dir / "data.yaml"
    content = "\n".join(
        [
            f"path: {dataset_dir.resolve()}",
            "train: images/train",
            "val: images/val",
            "test: images/test",
            "names:",
            "  0: toy",
            "  1: shoe",
            "",
        ]
    )
    data_yaml.write_text(content, encoding="utf-8")


def summarize_assignments(assignments: Mapping[str, str]) -> Dict[str, List[str]]:
    sessions_by_split: MutableMapping[str, List[str]] = {"train": [], "val": [], "test": []}
    for session_id, split in assignments.items():
        sessions_by_split[split].append(session_id)
    for split in sessions_by_split:
        sessions_by_split[split].sort()
    return dict(sessions_by_split)


def write_outputs(
    dataset_dir: Path,
    assignment_rows: Sequence[Mapping[str, str]],
    assignments: Mapping[str, str],
    seed: int,
    train_ratio: float,
    val_ratio: float,
) -> None:
    with (dataset_dir / "split_assignments.csv").open("w", newline="", encoding="utf-8") as f:
        fieldnames = list(assignment_rows[0].keys()) if assignment_rows else []
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(assignment_rows)

    sessions_by_split = summarize_assignments(assignments)
    manifest = {
        "seed": seed,
        "train_ratio": train_ratio,
        "val_ratio": val_ratio,
        "test_ratio": 1.0 - train_ratio - val_ratio,
        "sessions_by_split": sessions_by_split,
        "session_overlap": {
            "train_val": sorted(set(sessions_by_split["train"]) & set(sessions_by_split["val"])),
            "train_test": sorted(set(sessions_by_split["train"]) & set(sessions_by_split["test"])),
            "val_test": sorted(set(sessions_by_split["val"]) & set(sessions_by_split["test"])),
        },
    }
    manifest["no_session_overlap"] = not any(manifest["session_overlap"].values())

    (dataset_dir / "split_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build deterministic session-separated YOLO dataset split")
    parser.add_argument("--dataset-dir", type=Path, required=True, help="Canonical dataset root with samples.csv")
    parser.add_argument("--manifest", type=Path, default=None, help="Optional capture manifest CSV")
    parser.add_argument("--seed", type=int, default=42, help="Deterministic split seed")
    parser.add_argument("--train-ratio", type=float, default=0.7, help="Session ratio for train split")
    parser.add_argument("--val-ratio", type=float, default=0.15, help="Session ratio for val split")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.train_ratio <= 0 or args.train_ratio >= 1:
        raise SystemExit("--train-ratio must be in (0,1)")
    if args.val_ratio < 0 or args.val_ratio >= 1:
        raise SystemExit("--val-ratio must be in [0,1)")
    if args.train_ratio + args.val_ratio >= 1:
        raise SystemExit("train_ratio + val_ratio must be < 1")

    dataset_dir = args.dataset_dir
    samples_csv = dataset_dir / "samples.csv"

    if not samples_csv.exists():
        raise SystemExit(f"Missing samples.csv: {samples_csv}")

    samples = load_samples(samples_csv)
    session_meta = load_capture_manifest(args.manifest)

    session_ids = sorted({row["session_id"] for row in samples})
    assignments = assign_sessions_to_splits(
        session_ids=session_ids,
        seed=int(args.seed),
        train_ratio=float(args.train_ratio),
        val_ratio=float(args.val_ratio),
        session_meta=session_meta,
    )
    validate_no_session_overlap(assignments)

    assignment_rows = materialize_split(dataset_dir=dataset_dir, samples=samples, assignments=assignments)
    write_ultralytics_data_yaml(dataset_dir)
    write_outputs(
        dataset_dir=dataset_dir,
        assignment_rows=assignment_rows,
        assignments=assignments,
        seed=int(args.seed),
        train_ratio=float(args.train_ratio),
        val_ratio=float(args.val_ratio),
    )

    sessions_by_split = summarize_assignments(assignments)
    print("Session split summary:")
    for split in ("train", "val", "test"):
        print(f"  {split}: {len(sessions_by_split[split])} sessions")
    print(f"Wrote split files under: {dataset_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
