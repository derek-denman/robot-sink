# Toy-vs-Shoe I0 Runbook

## Scope
Iteration-0 builds a fast, safety-biased binary detector (`toy`, `shoe`) for pickup gating.

## Non-negotiables
- CVAT is labeling-only. Ignore CVAT split artifacts (`train.txt`, `val.txt`, CVAT `data.yaml`).
- Dataset split is generated only by repo scripts with session separation.
- Critical safety error is overlap-based in pickup zone: predicted `toy` must overlap GT `shoe` (IoU >= 0.3 or toy center in shoe box).
- Runtime detection message is `vision_msgs/msg/Detection2DArray` on `/perception/detections_2d`.
- Reproducibility is required: fixed seed + deterministic training.

## I0 Targets
- Data: 12 sessions (`toy_only x3`, `shoe_only x3`, `mixed_clutter x4`, `empty_floor x2`), at least 2 lighting conditions, at least 4 backgrounds.
- Labels: <= 500 instances per class.
- Acceptance:
  - `toy` recall >= 70% on mixed-clutter test sessions.
  - `shoe->toy` critical error rate < 5% on pickup-risk subset.
- Deployment: TensorRT FP16 on Jetson (`imgsz=640`, `batch=1`, `dynamic=False`, `nms=True`).

## Directory Conventions
- Raw bag sessions: `ros_ws/src/perception/toy_shoe_perception/data/raw_bags/`
- Extracted frames + metadata: `ros_ws/src/perception/toy_shoe_perception/dataset/extracted/`
- CVAT imports/exports: `ros_ws/src/perception/toy_shoe_perception/dataset/cvat/`
- Canonical YOLO dataset: `ros_ws/src/perception/toy_shoe_perception/dataset/yolo_i0/`
- Train/eval artifacts: `ros_ws/src/perception/toy_shoe_perception/artifacts/`

## End-to-End Commands
1. Extract deterministic frames from each session:
```bash
python3 ros_ws/src/perception/toy_shoe_perception/scripts/extract_rosbag_images.py \
  --bags-dir ros_ws/src/perception/toy_shoe_perception/data/raw_bags \
  --out-dir ros_ws/src/perception/toy_shoe_perception/dataset/extracted \
  --fps 2.0 --seed 42
```
2. Label frames in CVAT with only `toy` and `shoe`.
3. Import CVAT Ultralytics YOLO exports (ignore CVAT split files):
```bash
python3 ros_ws/src/perception/toy_shoe_perception/scripts/import_cvat_ultralytics_yolo.py \
  --exports-dir ros_ws/src/perception/toy_shoe_perception/dataset/cvat/exports \
  --session-map ros_ws/src/perception/toy_shoe_perception/dataset/extracted/session_index.json \
  --out-dir ros_ws/src/perception/toy_shoe_perception/dataset/yolo_i0
```
4. Build session-separated split:
```bash
python3 ros_ws/src/perception/toy_shoe_perception/scripts/prepare_dataset_split.py \
  --dataset-dir ros_ws/src/perception/toy_shoe_perception/dataset/yolo_i0 \
  --manifest docs/perception/i0_capture_manifest.csv \
  --seed 42
```
5. Train reproducibly:
```bash
python3 ros_ws/src/perception/toy_shoe_perception/scripts/train_yolo.py \
  --dataset-dir ros_ws/src/perception/toy_shoe_perception/dataset/yolo_i0 \
  --out-dir ros_ws/src/perception/toy_shoe_perception/artifacts/train_i0 \
  --seed 42 --epochs 50 --imgsz 640
```
6. Evaluate with safety-first critical metric:
```bash
python3 ros_ws/src/perception/toy_shoe_perception/scripts/evaluate_toy_shoe.py \
  --pred-dir ros_ws/src/perception/toy_shoe_perception/artifacts/train_i0/preds \
  --gt-dir ros_ws/src/perception/toy_shoe_perception/dataset/yolo_i0/labels/test \
  --pickup-zone 0.30,0.20,0.95,0.98
```
7. Mine hard frames for relabeling:
```bash
python3 ros_ws/src/perception/toy_shoe_perception/scripts/mine_hard_frames.py \
  --critical-report ros_ws/src/perception/toy_shoe_perception/artifacts/eval_i0/critical_error_report.json \
  --out ros_ws/src/perception/toy_shoe_perception/artifacts/eval_i0/hard_frames.txt
```
8. Export TensorRT on Jetson:
```bash
python3 ros_ws/src/perception/toy_shoe_perception/scripts/export_tensorrt.py \
  --weights ros_ws/src/perception/toy_shoe_perception/artifacts/train_i0/weights/best.pt \
  --out-dir ros_ws/src/perception/toy_shoe_perception/artifacts/export_i0
```
9. Run detector + safety gate:
```bash
ros2 launch toy_shoe_perception toy_shoe_i0.launch.py
```

## Go/No-Go
- GO only when both hold:
  - mixed-clutter toy recall >= 0.70
  - pickup-risk critical shoe->toy rate < 0.05
- If NO-GO, run hard-frame mining and collect targeted additional sessions before retraining.
