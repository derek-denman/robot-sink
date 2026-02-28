#!/usr/bin/env python3
"""Safety-first evaluator for toy-vs-shoe detection outputs.

Outputs:
- metrics.json
- confusion.csv
- critical_error_report.json
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Box:
    class_id: int
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def parse_pickup_zone(zone_csv: str) -> Tuple[float, float, float, float]:
    parts = [float(p.strip()) for p in zone_csv.split(",")]
    if len(parts) != 4:
        raise ValueError("pickup zone must be x1,y1,x2,y2")
    x1, y1, x2, y2 = parts
    if x2 <= x1 or y2 <= y1:
        raise ValueError("pickup zone must satisfy x2>x1 and y2>y1")
    return (clamp01(x1), clamp01(y1), clamp01(x2), clamp01(y2))


def yolo_to_box(parts: Sequence[str], is_prediction: bool) -> Box:
    if len(parts) < 5:
        raise ValueError(f"Invalid YOLO row (expected >=5 columns): {' '.join(parts)}")

    cls = int(float(parts[0]))
    xc = float(parts[1])
    yc = float(parts[2])
    w = float(parts[3])
    h = float(parts[4])
    conf = float(parts[5]) if (is_prediction and len(parts) >= 6) else 1.0

    x1 = clamp01(xc - (w / 2.0))
    y1 = clamp01(yc - (h / 2.0))
    x2 = clamp01(xc + (w / 2.0))
    y2 = clamp01(yc + (h / 2.0))

    return Box(class_id=cls, x1=x1, y1=y1, x2=x2, y2=y2, conf=conf)


def load_yolo_labels(labels_dir: Path, is_prediction: bool) -> Dict[str, List[Box]]:
    out: Dict[str, List[Box]] = {}
    for txt in sorted(labels_dir.rglob("*.txt")):
        boxes: List[Box] = []
        for line in txt.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            boxes.append(yolo_to_box(parts, is_prediction=is_prediction))
        out[txt.stem] = boxes
    return out


def area(box: Box) -> float:
    return max(0.0, box.x2 - box.x1) * max(0.0, box.y2 - box.y1)


def iou(a: Box, b: Box) -> float:
    ix1 = max(a.x1, b.x1)
    iy1 = max(a.y1, b.y1)
    ix2 = min(a.x2, b.x2)
    iy2 = min(a.y2, b.y2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = area(a) + area(b) - inter
    if union <= 0.0:
        return 0.0
    return inter / union


def center(box: Box) -> Tuple[float, float]:
    return ((box.x1 + box.x2) / 2.0, (box.y1 + box.y2) / 2.0)


def point_in_xyxy(x: float, y: float, zone: Tuple[float, float, float, float]) -> bool:
    return zone[0] <= x <= zone[2] and zone[1] <= y <= zone[3]


def center_in_box(inner: Box, outer: Box) -> bool:
    cx, cy = center(inner)
    return outer.x1 <= cx <= outer.x2 and outer.y1 <= cy <= outer.y2


def zone_box(zone: Tuple[float, float, float, float]) -> Box:
    return Box(class_id=-1, x1=zone[0], y1=zone[1], x2=zone[2], y2=zone[3], conf=1.0)


def box_in_pickup_zone(box: Box, pickup_zone: Tuple[float, float, float, float]) -> bool:
    cx, cy = center(box)
    if point_in_xyxy(cx, cy, pickup_zone):
        return True
    return iou(box, zone_box(pickup_zone)) > 0.0


def overlap_for_critical(pred_toy: Box, gt_shoe: Box, critical_iou_thresh: float) -> bool:
    return iou(pred_toy, gt_shoe) >= critical_iou_thresh or center_in_box(pred_toy, gt_shoe)


def greedy_match(gt: Sequence[Box], pred: Sequence[Box], iou_thresh: float) -> List[Tuple[int, int, float]]:
    candidates: List[Tuple[float, int, int]] = []
    for gi, g in enumerate(gt):
        for pi, p in enumerate(pred):
            score = iou(g, p)
            if score >= iou_thresh:
                candidates.append((score, gi, pi))

    candidates.sort(key=lambda item: item[0], reverse=True)
    used_gt = set()
    used_pred = set()
    matches: List[Tuple[int, int, float]] = []

    for score, gi, pi in candidates:
        if gi in used_gt or pi in used_pred:
            continue
        used_gt.add(gi)
        used_pred.add(pi)
        matches.append((gi, pi, score))

    return matches


def extract_session_id(frame_stem: str) -> str:
    match = re.match(r"^(s\d{2,})[_\-].*", frame_stem)
    if match:
        return match.group(1)
    return "unknown"


def load_scene_type_map(capture_manifest: Optional[Path]) -> Dict[str, str]:
    if capture_manifest is None:
        return {}

    mapping: Dict[str, str] = {}
    with capture_manifest.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sid = row.get("session_id", "").strip()
            if not sid:
                continue
            mapping[sid] = row.get("scene_type", "").strip()
    return mapping


def resolve_frame_path(stem: str, gt_dir: Path, images_dir: Optional[Path]) -> str:
    if images_dir is not None:
        for ext in (".jpg", ".jpeg", ".png", ".bmp", ".webp"):
            candidate = images_dir / f"{stem}{ext}"
            if candidate.exists():
                return str(candidate)
    return str(gt_dir / f"{stem}.txt")


def compute_precision(tp: int, fp: int) -> float:
    denom = tp + fp
    return (tp / denom) if denom > 0 else 0.0


def compute_recall(tp: int, fn: int) -> float:
    denom = tp + fn
    return (tp / denom) if denom > 0 else 0.0


def compute_next_data_prescription(
    toy_recall_mixed: Optional[float],
    critical_rate: float,
    target_toy_recall: float,
    target_critical_rate: float,
) -> Optional[Dict[str, Any]]:
    needs_more = False
    recommendations: List[str] = []
    min_sessions = 0

    if toy_recall_mixed is None or toy_recall_mixed < target_toy_recall:
        needs_more = True
        recommendations.append("Collect more mixed_clutter sessions with toy near occluding clutter.")
        min_sessions += 2

    if critical_rate >= target_critical_rate:
        needs_more = True
        recommendations.append("Collect more pickup-risk shoe-only and mixed_clutter sessions in pickup zone.")
        min_sessions += 2

    if not needs_more:
        return None

    min_sessions = max(2, min_sessions)
    return {
        "minimum_additional_sessions": min_sessions,
        "recommendations": recommendations,
    }


def evaluate(
    gt_map: Mapping[str, Sequence[Box]],
    pred_map: Mapping[str, Sequence[Box]],
    gt_dir: Path,
    images_dir: Optional[Path],
    pickup_zone: Tuple[float, float, float, float],
    iou_thresh: float,
    critical_iou_thresh: float,
    toy_class_id: int,
    shoe_class_id: int,
    scene_type_by_session: Mapping[str, str],
    target_toy_recall: float,
    target_critical_rate: float,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]], Dict[str, Any]]:
    stems = sorted(set(gt_map.keys()) | set(pred_map.keys()))

    class_stats: Dict[int, Dict[str, int]] = {
        toy_class_id: {"tp": 0, "fp": 0, "fn": 0},
        shoe_class_id: {"tp": 0, "fp": 0, "fn": 0},
    }

    mixed_toy_tp = 0
    mixed_toy_fn = 0

    pickup_risk_shoe_count = 0
    critical_error_count = 0
    critical_instances: List[Dict[str, Any]] = []
    unsafe_stems = set()

    for stem in stems:
        gt_boxes = list(gt_map.get(stem, []))
        pred_boxes = list(pred_map.get(stem, []))

        for class_id in (toy_class_id, shoe_class_id):
            gt_c = [b for b in gt_boxes if b.class_id == class_id]
            pred_c = [b for b in pred_boxes if b.class_id == class_id]
            matches = greedy_match(gt_c, pred_c, iou_thresh=iou_thresh)
            tp = len(matches)
            fn = len(gt_c) - tp
            fp = len(pred_c) - tp
            class_stats[class_id]["tp"] += tp
            class_stats[class_id]["fn"] += fn
            class_stats[class_id]["fp"] += fp

        session_id = extract_session_id(stem)
        if scene_type_by_session.get(session_id) == "mixed_clutter":
            gt_toy = [b for b in gt_boxes if b.class_id == toy_class_id]
            pred_toy = [b for b in pred_boxes if b.class_id == toy_class_id]
            mixed_matches = greedy_match(gt_toy, pred_toy, iou_thresh=iou_thresh)
            mixed_toy_tp += len(mixed_matches)
            mixed_toy_fn += (len(gt_toy) - len(mixed_matches))

        gt_shoes = [b for b in gt_boxes if b.class_id == shoe_class_id]
        pred_toys = [b for b in pred_boxes if b.class_id == toy_class_id]

        for gt_shoe in gt_shoes:
            if not box_in_pickup_zone(gt_shoe, pickup_zone):
                continue
            pickup_risk_shoe_count += 1

            matched_toy = None
            for pred_toy in pred_toys:
                if not box_in_pickup_zone(pred_toy, pickup_zone):
                    continue
                if overlap_for_critical(pred_toy, gt_shoe, critical_iou_thresh=critical_iou_thresh):
                    matched_toy = pred_toy
                    break

            if matched_toy is not None:
                critical_error_count += 1
                unsafe_stems.add(stem)
                critical_instances.append(
                    {
                        "frame_stem": stem,
                        "frame_path": resolve_frame_path(stem=stem, gt_dir=gt_dir, images_dir=images_dir),
                        "gt_shoe_box": {
                            "x1": gt_shoe.x1,
                            "y1": gt_shoe.y1,
                            "x2": gt_shoe.x2,
                            "y2": gt_shoe.y2,
                        },
                        "pred_toy_box": {
                            "x1": matched_toy.x1,
                            "y1": matched_toy.y1,
                            "x2": matched_toy.x2,
                            "y2": matched_toy.y2,
                            "conf": matched_toy.conf,
                        },
                        "iou": iou(matched_toy, gt_shoe),
                        "center_in_box": center_in_box(matched_toy, gt_shoe),
                    }
                )

    toy_stats = class_stats[toy_class_id]
    shoe_stats = class_stats[shoe_class_id]

    toy_precision = compute_precision(toy_stats["tp"], toy_stats["fp"])
    toy_recall = compute_recall(toy_stats["tp"], toy_stats["fn"])
    shoe_precision = compute_precision(shoe_stats["tp"], shoe_stats["fp"])
    shoe_recall = compute_recall(shoe_stats["tp"], shoe_stats["fn"])

    toy_recall_mixed = None
    if (mixed_toy_tp + mixed_toy_fn) > 0:
        toy_recall_mixed = compute_recall(mixed_toy_tp, mixed_toy_fn)

    critical_rate = (critical_error_count / pickup_risk_shoe_count) if pickup_risk_shoe_count > 0 else 0.0

    next_data = compute_next_data_prescription(
        toy_recall_mixed=toy_recall_mixed,
        critical_rate=critical_rate,
        target_toy_recall=target_toy_recall,
        target_critical_rate=target_critical_rate,
    )

    metrics = {
        "thresholds": {
            "iou_match": iou_thresh,
            "critical_iou": critical_iou_thresh,
            "pickup_zone_xyxy_norm": list(pickup_zone),
            "target_toy_recall_mixed_clutter": target_toy_recall,
            "target_critical_rate": target_critical_rate,
        },
        "toy": {
            "tp": toy_stats["tp"],
            "fp": toy_stats["fp"],
            "fn": toy_stats["fn"],
            "precision": toy_precision,
            "recall": toy_recall,
        },
        "shoe": {
            "tp": shoe_stats["tp"],
            "fp": shoe_stats["fp"],
            "fn": shoe_stats["fn"],
            "precision": shoe_precision,
            "recall": shoe_recall,
        },
        "toy_recall_mixed_clutter": toy_recall_mixed,
        "critical_error": {
            "count": critical_error_count,
            "pickup_risk_shoe_count": pickup_risk_shoe_count,
            "rate": critical_rate,
        },
        "go_no_go": {
            "toy_recall_mixed_clutter_pass": (toy_recall_mixed is not None and toy_recall_mixed >= target_toy_recall),
            "critical_error_rate_pass": critical_rate < target_critical_rate,
        },
    }
    if next_data is not None:
        metrics["next_data_prescription"] = next_data

    confusion_rows = [
        {
            "class_id": toy_class_id,
            "class_name": "toy",
            "tp": toy_stats["tp"],
            "fp": toy_stats["fp"],
            "fn": toy_stats["fn"],
            "precision": toy_precision,
            "recall": toy_recall,
        },
        {
            "class_id": shoe_class_id,
            "class_name": "shoe",
            "tp": shoe_stats["tp"],
            "fp": shoe_stats["fp"],
            "fn": shoe_stats["fn"],
            "precision": shoe_precision,
            "recall": shoe_recall,
        },
    ]

    critical_report = {
        "thresholds": {
            "critical_iou": critical_iou_thresh,
            "critical_rule": "IoU>=threshold OR pred_toy_center_in_gt_shoe_box",
            "pickup_zone_xyxy_norm": list(pickup_zone),
        },
        "pickup_risk_shoe_count": pickup_risk_shoe_count,
        "critical_error_count": critical_error_count,
        "critical_error_rate": critical_rate,
        "unsafe_frame_stems": sorted(unsafe_stems),
        "unsafe_frame_paths": [
            resolve_frame_path(stem=stem, gt_dir=gt_dir, images_dir=images_dir)
            for stem in sorted(unsafe_stems)
        ],
        "critical_instances": critical_instances,
    }

    if next_data is not None:
        critical_report["next_data_prescription"] = next_data

    return metrics, confusion_rows, critical_report


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def write_confusion_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["class_id", "class_name", "tp", "fp", "fn", "precision", "recall"],
        )
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate toy-vs-shoe predictions with safety-first metrics")
    parser.add_argument("--pred-dir", type=Path, required=True, help="Directory of YOLO prediction .txt files")
    parser.add_argument("--gt-dir", type=Path, required=True, help="Directory of YOLO ground-truth .txt files")
    parser.add_argument("--out-dir", type=Path, required=True, help="Output directory for evaluation artifacts")
    parser.add_argument("--images-dir", type=Path, default=None, help="Optional image directory for unsafe frame paths")
    parser.add_argument("--capture-manifest", type=Path, default=None, help="Optional capture manifest CSV")
    parser.add_argument("--pickup-zone", type=str, default="0.30,0.20,0.95,0.98", help="pickup zone x1,y1,x2,y2")
    parser.add_argument("--iou-thresh", type=float, default=0.5, help="IoU threshold for TP matching")
    parser.add_argument("--critical-iou-thresh", type=float, default=0.3, help="IoU threshold in critical overlap rule")
    parser.add_argument("--toy-class-id", type=int, default=0, help="Toy class id")
    parser.add_argument("--shoe-class-id", type=int, default=1, help="Shoe class id")
    parser.add_argument("--target-toy-recall", type=float, default=0.70, help="I0 mixed_clutter toy recall target")
    parser.add_argument("--target-critical-rate", type=float, default=0.05, help="I0 critical error rate target")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.pred_dir.exists() or not args.pred_dir.is_dir():
        raise SystemExit(f"--pred-dir not found or not a directory: {args.pred_dir}")
    if not args.gt_dir.exists() or not args.gt_dir.is_dir():
        raise SystemExit(f"--gt-dir not found or not a directory: {args.gt_dir}")

    pickup_zone = parse_pickup_zone(args.pickup_zone)
    gt_map = load_yolo_labels(args.gt_dir, is_prediction=False)
    pred_map = load_yolo_labels(args.pred_dir, is_prediction=True)
    scene_type_map = load_scene_type_map(args.capture_manifest)

    metrics, confusion_rows, critical_report = evaluate(
        gt_map=gt_map,
        pred_map=pred_map,
        gt_dir=args.gt_dir,
        images_dir=args.images_dir,
        pickup_zone=pickup_zone,
        iou_thresh=float(args.iou_thresh),
        critical_iou_thresh=float(args.critical_iou_thresh),
        toy_class_id=int(args.toy_class_id),
        shoe_class_id=int(args.shoe_class_id),
        scene_type_by_session=scene_type_map,
        target_toy_recall=float(args.target_toy_recall),
        target_critical_rate=float(args.target_critical_rate),
    )

    write_json(args.out_dir / "metrics.json", metrics)
    write_confusion_csv(args.out_dir / "confusion.csv", confusion_rows)
    write_json(args.out_dir / "critical_error_report.json", critical_report)

    print(f"Wrote: {args.out_dir / 'metrics.json'}")
    print(f"Wrote: {args.out_dir / 'confusion.csv'}")
    print(f"Wrote: {args.out_dir / 'critical_error_report.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
