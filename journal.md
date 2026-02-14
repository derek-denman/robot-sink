# Project Journal — Living-Room Toy Tidy Robot
**Date:** 2026-02-13 

## What I worked on today

Today I scoped and planned a portfolio robotics project: a compact **4WD true skid-steer mobile manipulator** (≤ **18"×18"**, target **< 50 lb**, **semi‑plush carpet**) that can **navigate a living room**, **detect dog toys while rejecting shoes**, pick toys using a **RoArm‑M2‑S**, and place them in a **toy bin**.

### Deliverables I created
- A **full BOM spreadsheet** (Excel) with part numbers, vendor links, unit prices, extended prices, category totals, and an Include (Y/N) toggle.
- A **README.md** describing the vision, MVP, future goals, architecture, and an 8‑week schedule.

---

## Topics covered (summary)

### 1) Project definition and MVP behavior loop
I defined the MVP loop as:
1) search/patrol → 2) detect a toy → 3) approach + align → 4) pick → 5) navigate to bin → 6) drop → 7) repeat.

I also identified “reliability scaffolding” for a home demo:
- Put a fiducial (e.g., AprilTag) on the bin for consistent drop-off alignment.
- Keep conservative reject rules for non-toys (e.g., shoes) to reduce embarrassing false pickups.

### 2) Compute platform decision
I decided to use a **Jetson Orin NX 16GB** (rather than a Jetson Nano) for enough headroom to run:
- RGB‑D perception
- mapping/localization/navigation
- task-level autonomy

I chose **Option B (module + carrier)** and settled on the **Waveshare JETSON‑ORIN‑IO‑BASE‑B** carrier.

### 3) Drivetrain approach
I chose a **4WD true skid steer** design:
- 4 brushed gearmotors (one per wheel)
- Control is “differential style” by grouping motors per side:
  - left-front + left-rear share the left command
  - right-front + right-rear share the right command

I explicitly decided to use **all 4 encoders**, not just one per side, so I can:
- average for better odometry
- detect slip by comparing encoders within a side
- catch mechanical issues early (dragging wheel, loose hub, etc.)

### 4) Brushed vs brushless (trade study)
I chose **brushed motors** for Phase 1 simplicity and 8‑week execution success, but I designed for a later brushless upgrade by:
- standardizing the interface as “wheel velocity commands + wheel state feedback”
- planning wiring/layout with space for per-wheel controllers later (e.g., CAN-based)
- standardizing wheel mounting to a common interface (12mm hex)

### 5) Motor driver selection
I selected the **Sabertooth 2×60** to drive the brushed motors for Phase 1 because of its current headroom and simplicity.

I noted alternate paths (for future iteration), such as:
- controllers with tighter integration of encoder closed-loop control
- per-wheel controllers when transitioning to brushless

### 6) Wheel size trade study for ≤18"×18" on semi‑plush carpet
I compared wheel size classes by:
- carpet “float” (width/softness)
- skid-steer scrub torque in turns (wider/grippier increases current spikes)
- packaging and stability in a small footprint

Decision:
- I chose **Option 2**: short-course style **~110 mm OD × ~50 mm wide**, **12 mm hex** wheels/tires as the best compromise.

For futureproofing:
- I standardized the wheel interface using a **6 mm shaft → 12 mm hex adapter**, so I can swap wheels easily and keep the wheel standard when I later swap motors.

### 7) Power system decision
I decided to use a **LiFePO4** battery for safety and practicality indoors, with:
- fused power distribution (separate branches for drivetrain / compute+sensors / arm)
- dedicated DC/DC rails for stable logic power
- an e-stop that cuts drivetrain power (and optionally arm power) in a predictable way

### 8) Sensors and high-level software stack
I converged on:
- **2D LiDAR (RPLIDAR A1 class)** for mapping/localization and obstacle avoidance
- **RGB‑D camera (OAK‑D Lite class)** for toy vs shoe classification and depth-based picking
- **ROS 2** as the integration layer
- **Nav2** for navigation
- **ros2_control + diff_drive_controller** as the drivetrain abstraction boundary

---

## Trade studies (decisions + rationale)

### A) Compute: Nano vs Orin NX 16GB
**Decision:** Orin NX 16GB  
**Rationale:** much better headroom for perception + navigation + logging while staying on-board.

### B) Actuation: brushed vs brushless (Phase 1)
**Decision:** brushed DC now  
**Rationale:** faster bring-up, fewer unknowns, simplest control/driver stack.  
**Futureproofing:** preserve the wheel-velocity interface and standardized wheel mounting so brushless becomes mostly a hardware swap.

### C) Driver: Sabertooth 2×60 vs alternatives
**Decision:** Sabertooth 2×60  
**Rationale:** robust, high current headroom for carpet skid-steer turning, simple interface.

### D) Wheels: small/narrow vs mid/short-course vs large/wide
**Decision:** short-course ~110×50 mm 12mm hex (Option 2)  
**Rationale:** better “float” on semi-plush carpet than narrow wheels, without the huge scrub penalty of very wide off-road wheels.

---

## Lessons learned

- **Skid-steer on carpet** will always slip in turns; it’s normal. I should plan to rely on SLAM/localization rather than trusting wheel odometry alone.
- **Current spikes** during turning are a real design constraint on carpet. Soft-start / acceleration limiting matters as much as motor size.
- **Futureproofing is mostly interfaces**:
  - keep ROS-side drivetrain commands as wheel velocities
  - standardize mechanical interfaces (12mm hex)
  - leave room and wiring strategy for future per-wheel controllers
- **Home robotics reliability** improves dramatically with simple markers (bin tag), conservative reject rules, and a scoped environment.

---

## Decisions I locked in today

- **Base:** 4WD true skid steer, ≤18"×18", target <50 lb, semi‑plush carpet
- **Encoders:** use **all 4 encoders**
- **Compute:** Jetson Orin NX 16GB (module + carrier)
- **Carrier:** Waveshare JETSON‑ORIN‑IO‑BASE‑B
- **Motors:** Pololu 37D brushed gearmotors with encoders (initially the 100:1 class)
- **Motor driver:** Sabertooth 2×60
- **Wheels:** short-course ~110×50 mm, 12mm hex (Option 2)
- **Wheel interface:** 6mm shaft → 12mm hex adapters
- **Power:** LiFePO4 + fused distribution + DC/DC rails

---

## Next steps

1. Finalize battery capacity (Ah) based on expected runtime and weight distribution.
2. Lay out the chassis in CAD (motor placement, wheelbase/track, arm mount, battery low and centered).
3. Define wiring harnesses (fuse sizing, wire gauge, connectors, e-stop logic).
4. Stand up the ROS workspace:
   - base bringup
   - ros2_control hardware interface
   - nav2 configuration
   - perception pipeline scaffolding
5. Start collecting a small in-room dataset for toy vs shoe training and testing.

---

## Key reference URLs I used today

- Jetson Orin NX series datasheet: https://developer.nvidia.com/downloads/jetson-orin-nx-series-data-sheet
- Waveshare JETSON‑ORIN‑IO‑BASE‑B: https://www.waveshare.com/jetson-orin-io-base-b.htm
- Sabertooth 2×60: https://www.dimensionengineering.com/products/sabertooth2x60
- Pololu 4755 (example motor): https://www.pololu.com/product/4755
- Pololu 2686 (6mm → 12mm hex adapter): https://www.pololu.com/product/2686/specs
- Nav2 behavior trees: https://docs.nav2.org/behavior_trees/index.html
- diff_drive_controller docs: https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
- OAK‑D Lite: https://shop.luxonis.com/products/oak-d-lite-1
- RPLIDAR A1M8 datasheet PDF: https://files.seeedstudio.com/products/114992561/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf
- RoArm‑M2‑S wiki: https://www.waveshare.com/wiki/RoArm-M2-S
