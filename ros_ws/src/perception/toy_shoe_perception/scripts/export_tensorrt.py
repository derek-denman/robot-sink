#!/usr/bin/env python3
"""Export a YOLO model to TensorRT engine with I0-safe defaults."""

from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


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


def find_repo_root(start: Path) -> Path:
    current = start.resolve()
    for candidate in [current] + list(current.parents):
        if (candidate / ".git").exists():
            return candidate
    raise RuntimeError(f"Unable to locate repo root from {start}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export TensorRT engine from Ultralytics weights")
    parser.add_argument("--weights", type=Path, required=True, help="Path to .pt weights")
    parser.add_argument("--out-dir", type=Path, required=True, help="Directory for export manifest/artifacts")
    parser.add_argument("--imgsz", type=int, default=640, help="Export image size")
    parser.add_argument("--half", action=argparse.BooleanOptionalAction, default=True, help="Use FP16")
    parser.add_argument("--dynamic", action=argparse.BooleanOptionalAction, default=False, help="Dynamic shapes")
    parser.add_argument("--batch", type=int, default=1, help="Engine max batch")
    parser.add_argument("--nms", action=argparse.BooleanOptionalAction, default=True, help="Bake NMS in engine")
    parser.add_argument("--device", type=str, default="", help="Optional export device")
    return parser.parse_args()


def write_manifest(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    args = parse_args()

    if not args.weights.exists() or not args.weights.is_file():
        raise SystemExit(f"--weights not found: {args.weights}")

    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = out_dir / "export_manifest.json"

    try:
        from ultralytics import YOLO, __version__ as ultralytics_version  # type: ignore
    except ImportError as exc:
        raise SystemExit("Ultralytics is not installed. Install with: pip install ultralytics") from exc

    repo_root = find_repo_root(Path(__file__))

    export_kwargs: Dict[str, Any] = {
        "format": "engine",
        "imgsz": int(args.imgsz),
        "half": bool(args.half),
        "dynamic": bool(args.dynamic),
        "batch": int(args.batch),
        "nms": bool(args.nms),
    }
    if args.device:
        export_kwargs["device"] = args.device

    manifest: Dict[str, Any] = {
        "status": "running",
        "started_at_utc": now_utc_iso(),
        "weights": str(args.weights.resolve()),
        "export_kwargs": export_kwargs,
        "ultralytics_version": ultralytics_version,
        "git_commit": git_commit_sha(repo_root),
        "note": "TensorRT engines must be built on the target Jetson environment for deployment compatibility.",
    }
    write_manifest(manifest_path, manifest)

    try:
        model = YOLO(str(args.weights))
        engine_path = model.export(**export_kwargs)

        manifest.update(
            {
                "status": "completed",
                "finished_at_utc": now_utc_iso(),
                "engine_path": str(engine_path),
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

    print(f"Export complete. Manifest: {manifest_path}")
    print(f"Engine path: {manifest.get('engine_path', '')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
