# Toy-vs-Shoe I0 Report

## Metadata
- Date:
- Branch:
- Commit:
- Model:
- Seed:
- Evaluator version:

## Dataset Summary
- Sessions captured:
- Lighting conditions:
- Backgrounds:
- Labeled instances (`toy`):
- Labeled instances (`shoe`):

## Split Integrity
- Session overlap train/val/test: pass/fail
- Notes:

## Training
- Command:
- Ultralytics version:
- Deterministic setting:
- Final metrics snapshot:

## Safety-First Evaluation
- Toy recall on mixed-clutter test:
- Shoe->Toy critical error rate on pickup-risk subset:
- Pickup zone definition:
- Critical metric rule (IoU >= 0.3 OR center-in-box):

## Deployment Export
- TensorRT engine path:
- Export settings (`imgsz=640`, `half=True`, `dynamic=False`, `batch=1`, `nms=True`):
- Jetson build details:

## Runtime Validation
- `/perception/detections_2d` live: pass/fail
- `/perception/toy_candidates` live: pass/fail
- `/perception/shoe_blocked` live: pass/fail
- Debounce/hold behavior validated: pass/fail

## Failure Modes and Next Data Prescription
- Failure mode:
- Hard-frame count:
- Minimum additional sessions before retrain:
- Targeted scene categories for next capture:
