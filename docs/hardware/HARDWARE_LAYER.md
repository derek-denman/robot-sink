# Hardware Layer (Source of Truth)

This document is the current hardware wiring source of truth for the robot stack. Values not verified in-repo are explicitly marked `UNKNOWN` with a concrete resolution step.

## Assumptions / Unknowns
- Canonical base controller board is **BeagleBone Green Gateway**; this document uses P8/P9 header naming from current repo mappings.
- Sabertooth runtime configuration is `packetized`, `baud: 38400`, `address: 128` (`beaglebone/host_daemon/config.yaml`).
- PRU1 TX is currently `P9_31` candidate with `tx_pru1_r30_bit: 0`; physical pinmux confirmation is required.
- Sabertooth S2 stop output is Linux GPIO `48` (commented as `gpio1_16 / P9_15`); electrical mapping must be confirmed on hardware.
- Jetson carrier power input connector type and pinout are `UNKNOWN` in current repo files.
- Branch fuse values, wire gauges, connector part numbers, and many harness details are `UNKNOWN` in current repo files.
- Physical BBB E-stop input pin is `UNKNOWN`; current daemon implements software/API estop and GPIO stop-output, not a defined physical estop input pin.
- Exact RPLIDAR unit variant (`A1` vs `A1M8`) is `UNKNOWN`; repo references both.
- Left/right wheel physical connector assignment (LF/LR/RF/RR to encoder indices 0..3) is `UNKNOWN`; odometry config currently assumes left indices `[0,2]`, right indices `[1,3]`.

## System Topology
```text
OAK-D Lite (USB 3.x) -----------------------> Jetson Orin NX (Waveshare IO-BASE-B family)
RPLIDAR A1/A1M8 (USB-serial /dev/ttyRPLIDAR) -^
RoArm-M2-S (USB-serial /dev/ttyROARM) -------^

Jetson USB networking (192.168.7.1) <------> BeagleBone (192.168.7.2)

BeagleBone PRU0 (R31 bits 0..7) <----------- Encoder A/B x4 (via 74LVC245A level buffer)
BeagleBone PRU1 TX (candidate P9_31) -------> Sabertooth 2x60 S1 (serial)
BeagleBone GPIO48 (candidate P9_15) --------> Sabertooth 2x60 S2 (stop/inhibit)
BeagleBone GND ------------------------------> Sabertooth COM/0V (common reference)
```

## Bill of Materials (BOM)
| Subsystem | Item | Manufacturer | Exact Model | Qty | Key Electrical Specs | Connector Type | Vendor/Manual URL | Mounting/Accessory Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Power | Main battery | Bioenno | BLF-1230A | 1 | 12V nominal, 30Ah LiFePO4 | `UNKNOWN — battery terminal style not captured` | `UNKNOWN — URL not recorded in repo` | Mount low/center for stability; secure with strap/bracket. |
| Power | Main battery switch | Blue Sea | 6006 | 1 | Battery disconnect switch; current rating `UNKNOWN` | `UNKNOWN — stud size not captured` | `UNKNOWN — URL not recorded in repo` | Mount externally accessible for service and emergency power cut. |
| Power | ANL fuse block | Blue Sea | 5005 | 1 | ANL fuse holder/distribution; electrical limits `UNKNOWN` | ANL fuse + stud terminals (exact size `UNKNOWN`) | `UNKNOWN — URL not recorded in repo` | Mount close to battery positive feed. |
| Power | Main ANL fuse | Blue Sea | 5125 | 1 | 100A ANL fuse | ANL blade + stud hardware | `UNKNOWN — URL not recorded in repo` | Confirm installed amp rating physically before final release. |
| Power | DC bus bar | Blue Sea | 2301 | 1 | Power distribution bus; amp rating `UNKNOWN` | Stud terminals (exact size `UNKNOWN`) | `UNKNOWN — URL not recorded in repo` | Use as star return/distribution point. |
| Power regulation | DC/DC converter (5V rail) | Mean Well | DDR-60G-5 | 1 | 5V output rail; power/current limits `UNKNOWN — verify datasheet` | `UNKNOWN — terminal style not captured` | `UNKNOWN — URL not recorded in repo` | Feed BBB logic and 5V encoder/translator domain per final harness design. |
| Power regulation | DC/DC converter (12V rail) | Mean Well | DDR-60G-12 | 1 | 12V output rail; power/current limits `UNKNOWN — verify datasheet` | `UNKNOWN — terminal style not captured` | `UNKNOWN — URL not recorded in repo` | Candidate regulated 12V branch (arm or compute branch depending final build). |
| Compute | Jetson carrier board | Waveshare | JETSON-ORIN-IO-BASE-B family | 1 | Carrier for Orin NX; power connector/pinout `UNKNOWN` | `UNKNOWN — input connector type not captured` | https://www.waveshare.com/jetson-orin-io-base-b.htm | Requires Orin NX module, NVMe, and stable supply. |
| Compute | Main compute module | NVIDIA | Jetson Orin NX 16GB | 1 | Embedded compute module; no onboard storage in this kit | Board-to-board module connector | https://developer.nvidia.com/downloads/jetson-orin-nx-series-data-sheet | Installed on Waveshare carrier. |
| Base control | Base controller board | Seeed / BeagleBone | BeagleBone Green Gateway | 1 | 3.3V GPIO domain; PRU0/PRU1 used for encoder/motor control | P8/P9 headers (as wired in repo docs) | https://wiki.seeedstudio.com/BeagleBone-Green-Gateway/ | Current pin map in repo uses P8/P9 naming. |
| Drive | Motor driver | Dimension Engineering | Sabertooth 2x60 | 1 | Dual-channel brushed driver; serial control on S1, stop on S2 | Screw terminals + signal pins (exact mechanical details `UNKNOWN`) | https://www.dimensionengineering.com/products/sabertooth2x60 | Keep COM tied to BBB ground; do not back-feed Sabertooth 5V/BEC pin. |
| Drive | Gearmotor with encoder | Pololu | 4755 | 4 | 12V motor with quadrature encoder (A/B) | Motor leads + encoder connector (pin order `UNKNOWN`) | https://www.pololu.com/product/4755 | Two motors per side share one Sabertooth channel. |
| Perception | RGB-D camera | Luxonis | OAK-D Lite | 1 | USB 3.1 Gen1 data/power path required | USB device connector (`USB-C` on device side per project notes) | https://shop.luxonis.com/products/oak-d-lite-1 | Use USB 3.x path; powered hub may be required if brownouts occur. |
| Perception | 2D lidar | Slamtec | RPLIDAR A1/A1M8 (`exact variant UNKNOWN`) | 1 | USB-serial path in current stack (`/dev/ttyRPLIDAR`) | USB-serial adapter/cable (`exact connector UNKNOWN`) | https://files.seeedstudio.com/products/114992561/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf | Maintain clear scan plane; warm up before mapping-grade runs. |
| Manipulator | Robot arm | Waveshare | RoArm-M2-S | 1 | 12V 5A class power requirement (project context); USB-serial comms | Power connector `UNKNOWN`; USB-serial cable (`/dev/ttyROARM`) | https://www.waveshare.com/wiki/RoArm-M2-S | E-stop power-cut behavior for arm is not yet verified. |
| Signal conditioning | Encoder level buffer | STMicroelectronics class part | 74LVC245A | 1 | 5V encoder-domain inputs translated to 3.3V BBB domain | IC pins or breakout headers (`exact board SKU UNKNOWN`) | https://www.st.com/resource/en/datasheet/cd00002333.pdf | Use one channel per A/B signal (8 total for 4 encoders). |
| Safety | Physical E-stop mushroom | IDEC | 22mm mushroom (`exact contact block model UNKNOWN`) | 1 | Contact ratings `UNKNOWN`; wired behavior to drivetrain/arm cutoff `UNKNOWN` | Panel-mount 22mm + contact block terminals | `UNKNOWN — URL not recorded in repo` | Physical wiring implementation details not yet captured in repo. |
| USB infrastructure | Powered USB hub (if needed) | `UNKNOWN` | `UNKNOWN` | 1 (optional) | Must sustain OAK-D + serial devices under load | Upstream/downstream USB connectors `UNKNOWN` | `UNKNOWN — not yet selected` | Add only if direct Jetson ports show instability. |
| Harness | Power connectors set | `UNKNOWN` | `UNKNOWN` | `UNKNOWN` | Branch current and voltage class dependent | Ring terminals, ferrules, quick-disconnects (`all UNKNOWN`) | `UNKNOWN — not yet selected` | Select after branch fuse/gauge decisions are finalized. |
| Harness | Signal connectors/cable assemblies | `UNKNOWN` | `UNKNOWN` | `UNKNOWN` | 3.3V/5V signal harness for encoders and control I/O | JST/Molex/other families `UNKNOWN` | `UNKNOWN — not yet selected` | Lock connector family during harness release. |

## Wiring (Text-First Diagram Equivalent)

### A) Power Tree
Text flow (current intended architecture):

```text
Battery (+) -> Blue Sea 6006 switch -> Blue Sea 5005 ANL block + Blue Sea 5125 (100A) -> Blue Sea 2301 distribution point -> branch feeds
Battery (-) ---------------------------------------------------------------> star return bus/ground reference
```

Branch targets:
- Drivetrain branch -> Sabertooth B+/B-.
- Compute branch -> Jetson carrier power input (`connector/pins UNKNOWN`).
- Arm branch -> RoArm-M2-S 12V path (`connector/fuse/gauge UNKNOWN`).
- BBB + logic branch -> BBB supply and 5V/3.3V signal-conditioning rails (`exact topology UNKNOWN`).

**Power Branch Table**

| Branch | Voltage | Expected Current | Fuse | Wire Gauge | Connector | Return/Ground Path |
| --- | --- | --- | --- | --- | --- | --- |
| Main battery feed to switch/fuse block | 12V nominal | `UNKNOWN` | Blue Sea 5125 (100A) at ANL block | `UNKNOWN` | Ring terminal studs (`size UNKNOWN`) | Battery negative to star return bus |
| Drivetrain (Sabertooth 2x60) | 12V nominal | `UNKNOWN — depends traction load/current spikes` | `UNKNOWN — branch fuse value not captured` | `UNKNOWN` | Sabertooth power terminals (`exact type UNKNOWN`) | Sabertooth B- and COM tied to common ground |
| Jetson + sensors branch | `UNKNOWN — direct battery vs regulated rail not yet captured` | `UNKNOWN` | `UNKNOWN` | `UNKNOWN` | Jetson carrier input connector (`UNKNOWN`) | Return to star ground/bus bar |
| RoArm-M2-S branch | 12V class | `UNKNOWN` | `UNKNOWN` | `UNKNOWN` | Arm power connector (`UNKNOWN`) | Return to star ground/bus bar |
| BBB + logic branch | `UNKNOWN — board supply path not captured` | `UNKNOWN` | `UNKNOWN` | `UNKNOWN` | BBB power connector (`UNKNOWN`) | Return to star ground; BBB GND shared with Sabertooth COM |

Ground strategy:
- Star return strategy at central distribution/bus location.
- BBB GND and Sabertooth COM/0V must share common reference.
- Chassis bonding point and policy are `UNKNOWN`.

Connectors currently referenced (exact mechanical sizes pending):
- Battery/switch/fuse/bus-bar ring terminals: `UNKNOWN — stud sizes not in repo`.
- Ferrules for converter and distribution terminals: `UNKNOWN — gauge/terminal spec not in repo`.
- High-current quick disconnect (XT60/Anderson/etc.): `UNKNOWN — family not selected`.
- Jetson power input plug/barrel style: `UNKNOWN`.
- RoArm dedicated power connector: `UNKNOWN`.

### B) Jetson Orin NX Carrier Wiring
| Connection | From | To | Cable/Connector | Interface | Notes |
| --- | --- | --- | --- | --- | --- |
| Carrier power input | Power distribution branch | Waveshare carrier power input pins | `UNKNOWN — connector type/pinout not in repo` | DC power in | Must be resolved from Waveshare manual + silkscreen before harness freeze. |
| OAK-D Lite data/power | Jetson USB 3.x host port | OAK-D Lite USB device port | USB 3.0/3.1 Gen1 cable (`device-side USB-C per project notes`) | USB 3.x | Keep short and strain-relieved; powered hub may be required under load. |
| RPLIDAR serial path | Jetson USB host | RPLIDAR USB-serial adapter | USB cable (`exact connector ends UNKNOWN`) | USB-serial (`/dev/ttyRPLIDAR`) | Udev alias managed by `jetson/udev/99-robot-devices.rules`. |
| RoArm serial path | Jetson USB host | RoArm USB-serial adapter | USB cable (`exact connector ends UNKNOWN`) | USB-serial (`/dev/ttyROARM`) | Default runtime uses `/dev/ttyROARM`; ROS node currently stubbed unless configured. |
| Jetson-BBB primary link | Jetson USB-Ethernet gadget interface | BBB USB gadget interface | USB data cable (`exact connector ends UNKNOWN`) | USB networking (`192.168.7.1` <-> `192.168.7.2`) | Primary control-plane link in current stack. |
| Jetson-BBB fallback link | Jetson serial USB path | BBB serial USB path | USB serial cable (`UNKNOWN`) | USB serial fallback | Mentioned as fallback in top-level architecture docs. |

### C) BeagleBone Header Pin Map (Signal -> BBB Pin)
| Signal Name | From Device + Pin | To BBB Header Pin | Voltage | Direction | Notes |
| --- | --- | --- | --- | --- | --- |
| `wheel0_A` | Encoder wheel0 A output via 74LVC245A (`channel CH1 intended`) | `P8_45` (`pru0_r31_0`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel0_A`; verify physical wheel index mapping. |
| `wheel0_B` | Encoder wheel0 B output via 74LVC245A (`channel CH2 intended`) | `P8_46` (`pru0_r31_1`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel0_B`. |
| `wheel1_A` | Encoder wheel1 A output via 74LVC245A (`channel CH3 intended`) | `P8_43` (`pru0_r31_2`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel1_A`. |
| `wheel1_B` | Encoder wheel1 B output via 74LVC245A (`channel CH4 intended`) | `P8_44` (`pru0_r31_3`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel1_B`. |
| `wheel2_A` | Encoder wheel2 A output via 74LVC245A (`channel CH5 intended`) | `P8_41` (`pru0_r31_4`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel2_A`. |
| `wheel2_B` | Encoder wheel2 B output via 74LVC245A (`channel CH6 intended`) | `P8_42` (`pru0_r31_5`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel2_B`. |
| `wheel3_A` | Encoder wheel3 A output via 74LVC245A (`channel CH7 intended`) | `P8_39` (`pru0_r31_6`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel3_A`. |
| `wheel3_B` | Encoder wheel3 B output via 74LVC245A (`channel CH8 intended`) | `P8_40` (`pru0_r31_7`) | 3.3V at BBB input (5V on encoder side) | In | Config source: `pins.encoders.wheel3_B`. |
| Sabertooth S1 serial TX | BBB PRU1 TX output bit0 | `P9_31` (candidate `pru1_r30_0`) | 3.3V TTL | Out | Candidate only; must confirm active pinmux and physical wiring. |
| Sabertooth S2 stop/inhibit | BBB GPIO stop output (`gpio48`) | `P9_15` candidate per config comment | 3.3V GPIO | Out | Asserted on boot/estop/watchdog; released only after arm. |
| Physical E-stop input to BBB | IDEC contact block wiring | `UNKNOWN` | `UNKNOWN` | In | No physical estop input pin mapping in current daemon config/docs. |
| Optional motor inhibit output | Same logical stop path as S2 | `GPIO48` (candidate `P9_15`) | 3.3V GPIO | Out | Implemented as dedicated safety stop line in current software architecture. |

Translator/buffering note:
- Encoder A/B channels are documented as 5V-domain signals translated to BBB 3.3V input domain via a 74LVC245A-class buffer.
- Exact assembled board SKU, OE/DIR strap details, and physical pin/channel orientation remain `UNKNOWN` and must be captured during harness verification.

### D) Motor Driver + Motors
Current motor driver is Sabertooth 2x60, with serial control mode configured as packetized by default.

**Motor Driver Terminal Map**

| Terminal | Connected To | Signal/Power | Voltage | Notes |
| --- | --- | --- | --- | --- |
| `B+` | Drivetrain fused power branch positive | Main motor power | 12V nominal | Branch fuse and wire gauge currently `UNKNOWN`. |
| `B-` | Battery/branch return | Main motor return | 0V reference | Must tie into common return/star ground strategy. |
| `M1 output pair` | Left motor bank (two motors in parallel) | Motor channel 1 output | PWM motor voltage from driver | Left bank is LF+LR by design intent; exact physical split point `UNKNOWN`. |
| `M2 output pair` | Right motor bank (two motors in parallel) | Motor channel 2 output | PWM motor voltage from driver | Right bank is RF+RR by design intent; exact physical split point `UNKNOWN`. |
| `S1` | BBB PRU1 serial TX path | Packetized serial command input | 3.3V TTL serial | Configured for packetized mode, `baud=38400`, `address=128`. |
| `S2` | BBB GPIO48 stop line path | Stop/inhibit line | 3.3V GPIO | Asserted by default until explicit arm. |
| `COM / 0V` | BBB ground and system signal ground | Logic reference ground | 0V | Required shared reference with BBB for S1/S2 signaling. |
| `5V/BEC` | Not used for external backfeed | Internal BEC output | 5V output | Do not back-feed from BBB/external supply. |

Control-mode and banking notes:
- Packetized command bytes used by PRU1 firmware: left channel commands `0/1`, right channel commands `4/5`.
- Physical LF/LR/RF/RR motor-to-bank wiring and connector labels are `UNKNOWN` unless captured on current harness photos.
- Odom defaults imply left encoder indices `[0,2]` and right `[1,3]` (`ros_ws/src/base_bringup/config/base_bringup.yaml`).
- For current sharing with two motors per channel: keep paired motors as matched model (Pololu 4755), keep branch wiring symmetry where possible, and define per-motor protection values as part of unresolved harness design (`UNKNOWN` ratings).

### E) Encoders
**Encoder Channel Map**

| Wheel | Encoder A | Encoder B | BBB Pins | Translator Channel | Supply Rail |
| --- | --- | --- | --- | --- | --- |
| Wheel 0 | `wheel0_A` | `wheel0_B` | `P8_45`, `P8_46` | `CH1`, `CH2` (intended mapping; verify physically) | 5V encoder side -> 3.3V BBB side |
| Wheel 1 | `wheel1_A` | `wheel1_B` | `P8_43`, `P8_44` | `CH3`, `CH4` (intended mapping; verify physically) | 5V encoder side -> 3.3V BBB side |
| Wheel 2 | `wheel2_A` | `wheel2_B` | `P8_41`, `P8_42` | `CH5`, `CH6` (intended mapping; verify physically) | 5V encoder side -> 3.3V BBB side |
| Wheel 3 | `wheel3_A` | `wheel3_B` | `P8_39`, `P8_40` | `CH7`, `CH8` (intended mapping; verify physically) | 5V encoder side -> 3.3V BBB side |

Pinout and compatibility notes:
- Pololu 37D encoder connector exact pin order is `UNKNOWN` in current repo and must be verified from official Pololu 4755 documentation before final crimping.
- Do not connect 5V encoder outputs directly into BBB PRU input pins; keep all encoder A/B paths buffered/translated to 3.3V.
- Keep encoder A/B routing as paired signal wiring and separated from high-current motor cables.

### F) RoArm-M2-S
- Power target is a 12V/5A class branch from main distribution (connector type and exact fuse/gauge are `UNKNOWN`).
- Comms path is USB-serial to Jetson as `/dev/ttyROARM` (udev-managed).
- E-stop behavior for arm power cut is `UNKNOWN`; current documented stop path is guaranteed for drivetrain through Sabertooth S2 and software estop.

### G) Sensors
- OAK-D Lite requires USB 3.x path (USB 3.0/3.1 Gen1 class). If brownouts/disconnects occur, add powered hub strategy.
- RPLIDAR A1 path is USB-serial through `/dev/ttyRPLIDAR` in current stack.
- Mounting notes (brief): keep lidar scan plane clear of arm sweep; secure USB cables with strain relief; keep sensor cables away from motor power runs where possible.

## Cables & Connectors
**USB/Ethernet Cable Table**

| Link | Minimum Spec | Connector Ends | Recommended Max Length | Shielding/Power Notes | Status |
| --- | --- | --- | --- | --- | --- |
| Jetson <-> OAK-D Lite | USB 3.0/3.1 Gen1 | Host USB 3.x to device USB-C (host-end connector type `UNKNOWN`) | `UNKNOWN — keep as short as practical` | High-throughput path; powered hub may be required if bus power is unstable. | Partial (device endpoints known; exact cable SKU not fixed) |
| Jetson <-> RPLIDAR | USB 2.0+ data-capable serial path | `UNKNOWN — depends on lidar adapter variant` | `UNKNOWN` | Must enumerate as `/dev/ttyRPLIDAR`; avoid power-only cables. | Partial |
| Jetson <-> RoArm | USB 2.0+ data-capable serial path | `UNKNOWN — depends on arm adapter variant` | `UNKNOWN` | Must enumerate as `/dev/ttyROARM`; avoid power-only cables. | Partial |
| Jetson <-> BBB (USB networking) | USB 2.0 data cable | `UNKNOWN — Jetson host connector to BBB USB client connector` | `UNKNOWN` | Must support data for gadget Ethernet (`192.168.7.x`). | Partial |
| Service Ethernet (if used) | Cat5e minimum (Cat6 acceptable) | RJ45 to RJ45 | `UNKNOWN` | Shielded vs unshielded choice not captured in repo. | Optional / Unknown |

Power and signal cabling requirements:
- Power branch wire gauges are `UNKNOWN` until branch-current and fuse selections are finalized.
- Insulation type and temperature rating are `UNKNOWN`; finalize alongside connector family selection.
- Ferrule sizes and ring-terminal stud sizes are `UNKNOWN`; derive from final gauge and hardware studs.
- Encoder A/B should be routed as paired signals; keep physically separated from motor-current runs.
- Add strain relief and service loops at moving/maintenance points (camera, lidar, arm, and controller ingress points).

## Construction Notes
- Mount battery low and near center-of-mass.
- Keep main fuse block physically close to battery positive output.
- Place main disconnect switch where it is reachable without removing covers.
- Use central bus bar/distribution region to minimize long return loops.
- Reserve service loops on all USB and signal harnesses for maintenance.
- Label harnesses consistently. Recommended scheme for this repo: `PWR-<branch>-<from>-<to>` and `SIG-<bus>-<from>-<to>`.
- Route high-current motor wiring away from encoder and serial signal wiring to reduce coupling/noise risk.

## Unknowns & Resolution Procedures
| ID | Missing Fact | Where to Check | Command/Action | Evidence to Save | Status |
| --- | --- | --- | --- | --- | --- |
| U-001 | Jetson carrier power input connector type and pinout | Waveshare carrier manual + physical silkscreen on installed board | Photograph connector and nearby silkscreen; capture manual page with connector/pin naming; annotate polarity and pin numbers | Photo of connector/silkscreen + manual screenshot | OPEN |
| U-002 | PRU1 TX header/pinmux confirmation for `P9_31` | `beaglebone/host_daemon/config.yaml`, active pinmux on BBB | Run: `grep -n "tx_header_pin\\|tx_pru1_r30_bit" beaglebone/host_daemon/config.yaml`; `config-pin -q P9_31`; verify PRU TX waveform with scope while sending commands | Command output + scope capture (idle-high serial) | OPEN |
| U-003 | GPIO48 physical header mapping to Sabertooth S2 | `beaglebone/host_daemon/config.yaml`, BBB pinout reference, live GPIO state | Run: `grep -n "s2_stop_gpio_number\\|s2_stop_active_high" beaglebone/host_daemon/config.yaml`; `gpioinfo | rg "gpio1_16|line 16|48"` (or `/sys/class/gpio/gpio48` checks); verify continuity to S2 wire | Command output + annotated photo/continuity notes | OPEN |
| U-004 | Physical BBB E-stop input wiring and exact pin | Harness build, E-stop contact block, BBB header continuity | Trace E-stop contact with DMM continuity to BBB headers; capture final pin and polarity (NO/NC) | Wiring photo + continuity log + final pin assignment | OPEN |
| U-005 | Branch fuse values, wire gauges, and connector SKUs | Physical harness, purchased parts, install notes | Read fuse markings, measure/verify wire gauge, record connector part numbers used at each branch endpoint | Harness photos + branch worksheet | OPEN |
| U-006 | Pololu 4755 encoder connector pin order and signal names | Official Pololu 4755 product page/datasheet | Pull official pinout table/diagram and map each harness wire color to `Vcc/GND/A/B`; update encoder section with verified pin order | Datasheet excerpt + annotated wiring map | OPEN |
| U-007 | RoArm 12V power connector type and estop power-cut behavior | RoArm hardware docs + live bench test | Identify physical power connector; perform continuity/voltage check during estop assert/release cycle to confirm whether arm supply is cut | Connector photo + bench voltage log | OPEN |
| U-008 | Physical motor pair split location (paralleled at Sabertooth vs distribution block) | Drivetrain harness path and terminal blocks | Inspect wiring path from M1/M2 terminals to both motors each side; document split node and wire gauge | Annotated wiring photos | OPEN |
| U-009 | Exact RPLIDAR model variant (`A1` vs `A1M8`) | Device label, procurement record, launch package assumptions | Photograph lidar label and compare with procurement entry; confirm selected driver profile remains `rplidar_a1_launch.py` | Device label photo + note in BOM | OPEN |
| U-010 | Chassis grounding/bonding policy and tie-point | Chassis design docs and physical electrical bonding points | Perform continuity checks between chassis/frame and electrical negative; document intended single-point bond (or isolation) | Continuity measurements + photo of bond point | OPEN |

## Verification Checklist
Command checks:

Run on Jetson (or host with access to Jetson environment):

```bash
ls -l /dev/ttyRPLIDAR /dev/ttyROARM
lsusb
ip -4 addr
ping -c 3 192.168.7.2
```

Run on BeagleBone in repo root:

```bash
curl -s http://127.0.0.1:8080/api/status | jq '.dry_run, .pru, .encoder.timestamp_us, .encoder.counts, .encoder.velocity_tps, .s2_stop_asserted'
python3 beaglebone/tools/cli_test.py status
python3 beaglebone/tools/cli_test.py arm
python3 beaglebone/tools/log_encoders.py --period 0.1
```

Physical checks:
- Verify battery -> switch -> fuse -> bus bar continuity and polarity before connecting loads.
- Verify common ground continuity between BBB GND and Sabertooth COM/0V.
- Verify S2 line is asserted at boot and released only after explicit arm.
- Verify encoder A/B logic levels are 3.3V at BBB input side of translator.
- Verify OAK-D remains stable under full data load on USB 3.x path.
- Verify harness strain relief, service loops, and separation between motor power and low-level signal wiring.
