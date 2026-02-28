from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _load_eval_module():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_toy_shoe.py"
    spec = importlib.util.spec_from_file_location("evaluate_toy_shoe", script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _box(mod, cls_id: int, x1: float, y1: float, x2: float, y2: float):
    return mod.Box(class_id=cls_id, x1=x1, y1=y1, x2=x2, y2=y2, conf=1.0)


def test_critical_metric_requires_spatial_overlap_not_coexistence(tmp_path: Path) -> None:
    mod = _load_eval_module()

    gt_map = {
        "s07_frame_0001": [
            _box(mod, 1, 0.40, 0.40, 0.60, 0.60),
        ]
    }
    pred_map = {
        "s07_frame_0001": [
            _box(mod, 0, 0.70, 0.70, 0.85, 0.85),
        ]
    }

    metrics, _, critical_report = mod.evaluate(
        gt_map=gt_map,
        pred_map=pred_map,
        gt_dir=tmp_path,
        images_dir=None,
        pickup_zone=(0.0, 0.0, 1.0, 1.0),
        iou_thresh=0.5,
        critical_iou_thresh=0.3,
        toy_class_id=0,
        shoe_class_id=1,
        scene_type_by_session={"s07": "mixed_clutter"},
        target_toy_recall=0.70,
        target_critical_rate=0.05,
    )

    assert metrics["critical_error"]["pickup_risk_shoe_count"] == 1
    assert metrics["critical_error"]["count"] == 0
    assert metrics["critical_error"]["rate"] == 0.0
    assert critical_report["unsafe_frame_stems"] == []


def test_critical_metric_counts_overlap_or_center_in_box(tmp_path: Path) -> None:
    mod = _load_eval_module()

    gt_map = {
        "s08_frame_0001": [
            _box(mod, 1, 0.30, 0.30, 0.70, 0.70),
        ],
        "s08_frame_0002": [
            _box(mod, 1, 0.30, 0.30, 0.70, 0.70),
        ],
    }
    pred_map = {
        "s08_frame_0001": [
            _box(mod, 0, 0.45, 0.45, 0.75, 0.75),
        ],
        "s08_frame_0002": [
            _box(mod, 0, 0.49, 0.49, 0.51, 0.51),
        ],
    }

    metrics, _, critical_report = mod.evaluate(
        gt_map=gt_map,
        pred_map=pred_map,
        gt_dir=tmp_path,
        images_dir=None,
        pickup_zone=(0.0, 0.0, 1.0, 1.0),
        iou_thresh=0.5,
        critical_iou_thresh=0.3,
        toy_class_id=0,
        shoe_class_id=1,
        scene_type_by_session={"s08": "mixed_clutter"},
        target_toy_recall=0.70,
        target_critical_rate=0.05,
    )

    assert metrics["critical_error"]["pickup_risk_shoe_count"] == 2
    assert metrics["critical_error"]["count"] == 2
    assert metrics["critical_error"]["rate"] == 1.0
    assert len(critical_report["unsafe_frame_stems"]) == 2
