#!/usr/bin/env python3
"""Reproducible Ultralytics YOLO training wrapper for toy-vs-shoe I0."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def find_repo_root(start: Path) -> Path:
    current = start.resolve()
    for candidate in [current] + list(current.parents):
        if (candidate / ".git").exists():
            return candidate
    raise RuntimeError(f"Could not locate repo root from {start}")


def git_commit_sha(repo_root: Path) -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()
    except Exception:
        return "unknown"


def compute_dataset_version_id(dataset_dir: Path) -> str:
    tracked_files = [
        "samples.csv",
        "split_manifest.json",
        "split_assignments.csv",
        "data.yaml",
        "classes.json",
    ]

    digest = hashlib.sha256()
    for rel in sorted(tracked_files):
        digest.update(rel.encode("utf-8"))
        digest.update(b"\0")
        path = dataset_dir / rel
        if path.exists() and path.is_file():
            digest.update(path.read_bytes())
        else:
            digest.update(b"__missing__")
        digest.update(b"\0")

    return digest.hexdigest()


def set_reproducibility(seed: int, deterministic: bool) -> None:
    os.environ["PYTHONHASHSEED"] = str(seed)
    random.seed(seed)

    try:
        import numpy as np  # type: ignore

        np.random.seed(seed)
    except Exception:
        pass

    try:
        import torch  # type: ignore

        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        torch.backends.cudnn.deterministic = bool(deterministic)
        torch.backends.cudnn.benchmark = not bool(deterministic)
    except Exception:
        pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train toy-vs-shoe YOLO model with reproducible settings")
    parser.add_argument("--dataset-dir", type=Path, required=True, help="Dataset directory containing data.yaml")
    parser.add_argument("--out-dir", type=Path, required=True, help="Output root for runs + manifest")
    parser.add_argument("--weights", type=str, default="yolo11n.pt", help="Initial model weights")
    parser.add_argument("--epochs", type=int, default=50, help="Training epochs")
    parser.add_argument("--imgsz", type=int, default=640, help="Training image size")
    parser.add_argument("--batch", type=int, default=16, help="Batch size")
    parser.add_argument("--workers", type=int, default=8, help="Data loader workers")
    parser.add_argument("--patience", type=int, default=20, help="Early-stop patience")
    parser.add_argument("--seed", type=int, default=42, help="Global random seed")
    parser.add_argument(
        "--deterministic",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable deterministic training behavior",
    )
    parser.add_argument("--device", type=str, default="", help="Device override (e.g., 0, cpu)")
    parser.add_argument("--run-name", type=str, default="toy_shoe_i0", help="Run name under out-dir/runs")
    return parser.parse_args()


def write_manifest(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    args = parse_args()

    dataset_dir = args.dataset_dir.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    data_yaml = dataset_dir / "data.yaml"
    if not data_yaml.exists():
        raise SystemExit(f"Missing dataset config: {data_yaml}")

    script_path = Path(__file__).resolve()
    repo_root = find_repo_root(script_path)

    try:
        from ultralytics import YOLO, __version__ as ultralytics_version  # type: ignore
    except ImportError as exc:
        raise SystemExit("Ultralytics is not installed. Install with: pip install ultralytics") from exc

    set_reproducibility(seed=int(args.seed), deterministic=bool(args.deterministic))

    dataset_version_id = compute_dataset_version_id(dataset_dir)
    manifest_path = out_dir / "run_manifest.json"
    train_kwargs: Dict[str, Any] = {
        "data": str(data_yaml),
        "epochs": int(args.epochs),
        "imgsz": int(args.imgsz),
        "batch": int(args.batch),
        "workers": int(args.workers),
        "patience": int(args.patience),
        "seed": int(args.seed),
        "deterministic": bool(args.deterministic),
        "project": str(out_dir / "runs"),
        "name": str(args.run_name),
        "exist_ok": True,
    }
    if args.device:
        train_kwargs["device"] = args.device

    manifest: Dict[str, Any] = {
        "status": "running",
        "started_at_utc": now_utc_iso(),
        "ultralytics_version": ultralytics_version,
        "seed": int(args.seed),
        "deterministic": bool(args.deterministic),
        "weights": str(args.weights),
        "dataset_dir": str(dataset_dir),
        "dataset_data_yaml": str(data_yaml),
        "dataset_version_id": dataset_version_id,
        "git_commit": git_commit_sha(repo_root),
        "train_kwargs": train_kwargs,
    }
    write_manifest(manifest_path, manifest)

    try:
        model = YOLO(str(args.weights))
        train_result = model.train(**train_kwargs)

        run_dir = None
        if getattr(model, "trainer", None) is not None:
            run_dir = str(getattr(model.trainer, "save_dir", ""))

        manifest.update(
            {
                "status": "completed",
                "finished_at_utc": now_utc_iso(),
                "run_dir": run_dir,
                "train_result": str(train_result),
            }
        )
        write_manifest(manifest_path, manifest)
    except Exception as exc:
        manifest.update(
            {
                "status": "failed",
                "finished_at_utc": now_utc_iso(),
                "error": str(exc),
            }
        )
        write_manifest(manifest_path, manifest)
        raise

    print(f"Training complete. Manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
