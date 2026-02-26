#!/usr/bin/env python3
"""Mine hard frames for relabeling from safety-first evaluation outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Sequence


def dedupe_preserve_order(items: Sequence[str]) -> List[str]:
    seen = set()
    out: List[str] = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        out.append(item)
    return out


def load_critical_report(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def mine_frames(report: Dict[str, Any], top_k: int) -> List[str]:
    paths = [str(p) for p in report.get("unsafe_frame_paths", []) if str(p).strip()]

    if not paths:
        paths = [str(stem) for stem in report.get("unsafe_frame_stems", [])]

    instances = report.get("critical_instances", [])
    if instances:
        ranked = sorted(
            instances,
            key=lambda item: (
                float(item.get("iou", 0.0)),
                bool(item.get("center_in_box", False)),
            ),
            reverse=True,
        )
        ranked_paths = [str(item.get("frame_path", item.get("frame_stem", ""))) for item in ranked]
        paths = ranked_paths + paths

    unique = dedupe_preserve_order([p for p in paths if p])
    if top_k > 0:
        unique = unique[:top_k]
    return unique


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mine hard/unsafe frames from critical_error_report.json")
    parser.add_argument("--critical-report", type=Path, required=True, help="Path to critical_error_report.json")
    parser.add_argument("--out", type=Path, required=True, help="Output txt file with one frame path per line")
    parser.add_argument("--top-k", type=int, default=200, help="Max number of hard frames to emit (0=all)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.critical_report.exists():
        raise SystemExit(f"Critical report does not exist: {args.critical_report}")

    report = load_critical_report(args.critical_report)
    hard_frames = mine_frames(report=report, top_k=int(args.top_k))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(hard_frames) + ("\n" if hard_frames else ""), encoding="utf-8")

    summary = {
        "critical_report": str(args.critical_report),
        "hard_frame_count": len(hard_frames),
        "top_k": int(args.top_k),
        "hard_frames_txt": str(args.out),
    }
    (args.out.parent / "hard_frame_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    print(f"Wrote hard frames: {args.out} ({len(hard_frames)} entries)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
