# Toy-vs-Shoe Training + Data Capture Guide (Beginner)

This guide explains exactly how to use the **Training / Data Capture** page in the console and how to label data in CVAT for Iteration-0.

## Before You Start
- Robot stack is running on Jetson.
- You can open the console UI in a browser (for example: `http://10.0.0.178:8080/`).
- OAK camera stream is available.
- You know the two labels allowed for I0: `toy` and `shoe`.

## What You Will Do
1. Verify camera framing in **Pre-Capture Camera Preview**.
2. Record one session at a time in **Training + Data Capture**.
3. Label each session in CVAT.
4. Export CVAT tasks in Ultralytics YOLO format.
5. Place exports in the repo for import/split/train scripts.

## Part A: Use The Training / Data Capture Page

### 1) Open the page and verify live camera
1. In the console, open the **Training** tab.
2. In **Pre-Capture Camera Preview**:
- Select a camera topic.
- Click **Apply Camera Stream**.
- Confirm the view is the correct room and angle.

If the scene is wrong, do not capture yet. Fix camera topic/position first.

### 2) Name the session
Use one unique tag per session in the capture box.

Examples:
- `s01_toy_only_room_a`
- `s04_shoe_only_room_b`
- `s07_mixed_clutter_room_c`
- `s11_empty_floor_room_d`

Use a naming pattern you can track later in CVAT and reports.

### 3) Set capture settings
Recommended starting values:
- `Target images`: `120`
- `Extractor FPS`: `2.0`
- `Buffer (sec)`: `8`
- `Duration override (sec)`: `0` (leave 0 unless you need exact manual duration)

The page shows planned runtime so you can verify expected recording length.

### 4) Capture data
1. Click **Start Timed Capture**.
2. Let it run to completion.
3. Confirm output paths shown on page:
- Bag storage root
- Active bag output
- Extracted image output

### 5) Repeat across required session types
For I0 target coverage, collect:
- `toy_only x3`
- `shoe_only x3`
- `mixed_clutter x4`
- `empty_floor x2`

Also ensure:
- At least 2 lighting conditions.
- At least 4 different backgrounds.

### 6) Quality gate after every session
Open a few extracted images and check:
- Right room.
- Correct camera angle.
- Subject visible when expected.
- No broken/blank frames.

If a session is wrong (wrong room, empty when it should not be, bad angle), discard and recapture.

## Part B: Label In CVAT (First Time)

### 1) Open CVAT
- In browser, open `http://<jetson-ip>:8080`.
- Sign in.

### 2) Create one task per session
1. Click **Create new task**.
2. Task name: match the session tag exactly.
3. Labels: add exactly:
- `toy`
- `shoe`

Do not add other labels for I0.

### 3) Upload images for that session
- Upload only images from that single session.
- Keep sessions separate (do not combine sessions in one task).

### 4) Draw boxes
- Draw a tight rectangle around each visible toy and shoe.
- Label every visible instance.
- If uncertain, skip the frame and flag it for review instead of guessing.

### 5) Review before export
- Spot-check random frames for missing boxes.
- Confirm labels are only `toy` or `shoe`.
- Confirm task name matches session name.

### 6) Export
1. In CVAT task, click **Export task dataset**.
2. Choose **Ultralytics YOLO 1.0**.
3. Save zip file to:

`ros_ws/src/perception/toy_shoe_perception/dataset/cvat/exports/`

Important for this repo:
- CVAT split files (`train.txt`, `val.txt`, `test.txt`, `data.yaml`) are ignored.
- Train/val/test split is generated later by repo scripts with session separation.

## Common Mistakes To Avoid
- Capturing in the wrong room or wrong camera topic.
- Mixing multiple sessions into one CVAT task.
- Using labels other than `toy` and `shoe`.
- Labeling only "easy" frames and skipping difficult ones.
- Relying on CVAT-generated split files.

## After Labeling (Next Command Step)
Once CVAT export zips are in `dataset/cvat/exports/`, run import + split scripts from the I0 runbook:
- `docs/perception/toy_shoe_i0_runbook.md`
