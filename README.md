# Living-Room Toy Tidy Robot

A compact **4WD skid-steer mobile manipulator** that can **navigate a living room**, **identify dog toys (and avoid picking up shoes/other objects)**, pick toys with a **RoArm-M2-S**, and **drop them into a toy bin**.

This project is intentionally scoped as an **8-week portfolio build**: reliable, demoable, and designed so you can upgrade major subsystems (especially the drivetrain) without rewriting everything.

---

## Vision

Build a real-home robotics demo that showcases:

- **Mobile autonomy** in a cluttered, mostly-carpet indoor environment
- **Robust perception** (toy vs non-toy classification) with depth-aware picking
- **Manipulation** (approach → grasp → carry → place)
- **Systems engineering** (power distribution, safety, deterministic I/O, maintainable software)

---

## MVP (Minimum Viable Product)

**MVP demo:** starting from a known pose, the robot autonomously collects **N** dog toys (start with N=3), deposits them in the bin, and ignores shoes.

Minimum capabilities:

1. **Drive base MVP**
   - Teleop driving
   - Emergency stop + watchdog “safe stop”
   - Stable velocity control on carpet

2. **Navigation MVP**
   - Map the living room, localize, and autonomously drive to a goal
   - Basic obstacle avoidance and recovery behaviors

3. **Perception MVP**
   - Detect and classify: `dog_toy` vs `shoe` (and optionally other negatives)
   - Produce a 3D pick target from RGB-D

4. **Manipulation MVP**
   - Drive to standoff pose near the toy
   - Align, grasp, lift, carry, and drop into the bin
   - Retry logic for failed grasps

---

## Future Goals

### Autonomy
- Multi-room “coverage” behaviors and smarter search patterns
- Behavior-tree task sequencing and recovery tuning (Nav2 BT-based autonomy)
- Bin docking refinement (fiducials + fine alignment)

### Perception
- Instance segmentation for better grasp points
- Hard-negative mining to reduce false positives (shoes, socks, remotes)
- Better “toy present / toy removed” verification

### Manipulation
- MoveIt 2 integration for collision-aware planning
- Grasp-quality scoring + regrasp behaviors
- Faster pick cycle time and higher success rate on soft plush toys

### Hardware
- **Brushless drivetrain upgrade** (per-wheel controllers, CAN bus, improved efficiency)
- Optional suspension/compliance to reduce skid-steer scrub on plush carpet
- Additional safety sensors (bumpers, cliff sensors, proximity)
- FPGA coprocessor milestone (encoder decode/timestamping/safety gating) after MVP

---

## High-Level Tech Overview

### Hardware

- **Compute:** Jetson Orin NX 16GB (main autonomy + perception computer)
- **Carrier:** Waveshare JETSON-ORIN-IO-BASE-B (9–19V input, NVMe + Wi-Fi M.2)

- **Base controller (MVP real-time / I/O):** **BeagleBone Green Gateway**
  - Handles **wheel encoders**, **motor driver control**, and **safety I/O**
  - Connected to Jetson over **USB 2.0** (initially serial; can upgrade to USB-network gadget later)

- **Drivetrain:** 4WD true skid-steer
  - 4× Pololu 37D brushed gearmotors with **encoders**
  - **All 4 encoders are used** (odometry + slip/health monitoring)
  - Sabertooth 2×60 motor driver (left pair on CH1, right pair on CH2)

- **Encoder electrical interface (important):**
  - Encoders run at **5V**
  - Encoder **A/B signals are level-shifted/buffered** down to **3.3V** for BeagleBone GPIO
  - Target approach: a single **8-channel buffer/translator** (A/B × 4 wheels), e.g. **74LVC245A-class** (5V-tolerant inputs at 3.3V Vcc)

- **Motor driver control + safety (important):**
  - BeagleBone → Sabertooth via **TTL serial (8N1) to S1**
  - Dedicated **safety/stop line** to **S2** (wired fail-safe)
  - **Common ground** between BeagleBone and Sabertooth at **COM/0V**
  - Do **not** back-feed Sabertooth **5V** terminal (it’s a BEC output)

- **Wheels:** 12mm hex short-course tires (~110mm OD × ~50mm width)
  - Wheel interface standardized on **12mm hex** via Pololu 6mm→12mm adapters

- **Sensors:**
  - OAK-D Lite (stereo depth + RGB) over **USB 3.x**
  - RPLIDAR A1 over **USB-Serial** (fastest integration)

- **Manipulator:** RoArm-M2-S (connected directly to Jetson over **USB-Serial @ 115200**)

- **Power:** single **LiFePO4 12V-class** battery + fused distribution + DC/DC rails
  - USB stability considerations (short cables, strain relief; powered hub if needed)

### Software

- **ROS 2** as the system backbone
- **Nav2** for mapping/localization/planning and behavior-tree navigation
- **ros2_control + diff_drive_controller** to keep the drivetrain abstracted
- **Perception pipeline** (toy vs shoe) using RGB-D + a lightweight detector
- **Task logic** (search → approach → pick → deliver) implemented as a BT or a simple state machine (MVP), migrating to BT for robustness
- **Jetson ↔ BeagleBone protocol:** start simple (framed serial), add watchdog + rate limits, then upgrade transport if needed

---

## Architecture Notes

### Compute + control split (MVP)
- **Jetson**: perception, navigation, task logic, arm driver
- **BeagleBone**: deterministic I/O (encoders, motor driver control, safety lines)

### Wiring-ready comms map (MVP)
- Jetson → OAK-D Lite: **USB 3.x**
- Jetson → RPLIDAR A1: **USB-Serial**
- Jetson → RoArm-M2-S: **USB-Serial @ 115200**
- Jetson ↔ BeagleBone: **USB 2.0**
- BeagleBone → Sabertooth 2×60: **TTL serial to S1**, **stop line to S2**, **COM/0V common ground**

### 4WD skid-steer model in ROS
Even though the robot has 4 driven wheels, it is controlled as a differential base by grouping wheels per side:

- Left-front + left-rear = “left side”
- Right-front + right-rear = “right side”

In ROS 2, `diff_drive_controller` supports multiple wheels per side via lists of wheel joints.

### Why 4 encoders (not just 2)
- Better odometry (averaging + filtering)
- Detect drift/slip (encoder disagreement)
- Detect mechanical issues early (binding wheel, loose hub, etc.)

### Brushless-ready by design
The “upgrade boundary” is the **wheel velocity command + wheel feedback** interface. When moving to brushless:
- Keep `ros2_control` + `diff_drive_controller` unchanged
- Swap motors + add per-wheel BLDC controllers (likely CAN-based)
- Re-implement only the low-level hardware interface

---

## 8-Week Schedule

### Week 1 — Mechanical + power bring-up
- Chassis assembly, motor mounting, wheel/hub alignment
- Power distribution (fuses, switch, E-stop)
- Bench test each motor + each encoder

### Week 2 — Drive base MVP
- Closed-loop velocity control on **BeagleBone**
- Teleop in ROS 2 (cmd_vel → wheel velocities)
- Encoder logging, tuning acceleration limits for carpet
- Bring up safety line behavior (S2 stop/fail-safe)

### Week 3 — Sensors online
- Bring up LiDAR and RGB-D, verify USB stability
- Establish frame tree (`tf2`) and timestamp sanity
- Record rosbag datasets for testing

### Week 4 — Navigation MVP
- Build map + localize
- Goal navigation + obstacle avoidance
- Recovery behaviors (replan, clear costmap, backup/spin)

### Week 5 — Perception MVP
- Toy vs shoe detection in-room
- Depth-based 3D pick target generation
- Threshold tuning + hard negatives

### Week 6 — Manipulation integration
- RoArm ROS node + calibration
- Implement pick primitive + grasp verification

### Week 7 — Bin delivery + task loop
- Add bin localization (fiducial recommended)
- Full loop: search → pick → deliver → repeat
- Retry logic and failure handling

### Week 8 — Reliability + portfolio polish
- Metrics: success rate, cycle time, false-pick rate
- Demo video + runbook
- Documentation: wiring diagram, calibration steps, tuning notes

---

## Repository Layout

```

.
├── README.md
├── bom/
│   └── robot_bom_links_fixed.xlsx
├── hardware/
│   ├── cad/
│   ├── wiring/
│   └── mounts/
├── ros_ws/
│   ├── src/
│   │   ├── base_bringup/
│   │   ├── base_description/        # URDF, meshes
│   │   ├── base_hardware/           # ros2_control HW interface
│   │   ├── nav2_config/
│   │   ├── perception/
│   │   ├── manipulation/
│   │   └── task_manager/
│   └── docker/                      # optional
└── docs/
├── bringup.md
├── calibration.md
├── safety.md
└── demo_runbook.md

```

---

## Safety

- Use a physical **E-stop** and conservative acceleration limits.
- Fuse battery branches (drive, compute/sensors, arm) and label wiring.
- Keep an “arm motion zone” clear during operation.
- Default to fail-safe behavior on comms loss.
- Implement a **dedicated stop path** to the motor driver (Sabertooth **S2**) that defaults to **STOP** on reboot/comms loss.
- Maintain **common grounding** between controller and motor driver (COM/0V reference), and **do not back-feed** the motor driver’s 5V/BEC terminal.

---

## Key References (hardware + software)

- Jetson Orin NX Series datasheet: https://developer.nvidia.com/downloads/jetson-orin-nx-series-data-sheet
- Waveshare JETSON-ORIN-IO-BASE-B: https://www.waveshare.com/jetson-orin-io-base-b.htm
- BeagleBone Green Gateway overview: https://wiki.seeedstudio.com/BeagleBone-Green-Gateway/
- Sabertooth 2×60: https://www.dimensionengineering.com/products/sabertooth2x60
- Pololu 37D gearmotor w/ encoder (example: #4755): https://www.pololu.com/product/4755
- Pololu 6mm→12mm hex adapter (example: #2686): https://www.pololu.com/product/2686
- Nav2 behavior trees: https://docs.nav2.org/behavior_trees/index.html
- diff_drive_controller (ros2_control): https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
- OAK-D Lite: https://shop.luxonis.com/products/oak-d-lite-1
- RPLIDAR A1 datasheet (A1M8 v3.0 PDF): https://files.seeedstudio.com/products/114992561/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf
- RoArm-M2-S wiki: https://www.waveshare.com/wiki/RoArm-M2-S
```
