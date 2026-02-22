# 05 - USB Devices and udev Rules

This setup creates stable serial aliases:

- RPLIDAR A1 -> `/dev/ttyRPLIDAR`
- RoArm-M2-S -> `/dev/ttyROARM`

Rules file: `jetson/udev/99-robot-devices.rules`

## Install Rules

```bash
cd ~/robot-sink
./jetson/scripts/setup_udev_rules.sh
```

Replug RPLIDAR and RoArm after install.

## Verify Device IDs

```bash
lsusb
udevadm info -a -n /dev/ttyUSB0 | less
udevadm info -a -n /dev/ttyUSB1 | less
```

Look for `idVendor`, `idProduct`, and serial attributes.

## Verify Stable Aliases

```bash
ls -l /dev/ttyRPLIDAR
ls -l /dev/ttyROARM
```

Expected:

- Symlinks exist and point to active ttyUSB/ttyACM devices
- Group ownership includes `dialout`

## If `/dev/ttyROARM` Is Missing

1. Check all serial candidates:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

2. Inspect IDs for each candidate:

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL'
udevadm info --query=property --name=/dev/ttyUSB1 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL'
udevadm info --query=property --name=/dev/ttyACM0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_MODEL'
```

3. If both adapters are `ID_VENDOR_ID=10c4` and `ID_MODEL_ID=ea60`, use `ID_MODEL` to split:

- `CP2102_USB_to_UART_Bridge_Controller` -> `/dev/ttyRPLIDAR`
- `CP2102N_USB_to_UART_Bridge_Controller` -> `/dev/ttyROARM`

4. If both report the same `ID_MODEL`, pin by serial:

- Add `ATTRS{serial}=="<rplidar-serial>"` to the RPLIDAR rule
- Add `ATTRS{serial}=="<roarm-serial>"` to the RoArm rule

5. Reload rules:

```bash
./jetson/scripts/setup_udev_rules.sh
```

## If Both Devices Use Same USB-UART Chipset

Refine the udev rule with serial matching:

```bash
udevadm info -a -n /dev/ttyUSBX | grep -m1 '{serial}'
```

Then add `ATTRS{serial}=="<serial-value>"` to the rule and reload.

## Sensor Mounting and Positioning

These mechanical constraints are prerequisites for reliable mapping/localization and perception.

### RPLIDAR A1 Placement

- Goal: keep an unobstructed 360 degree scan plane.
- Mount strategy:
  - Mount lidar above arm sweep volume, or
  - Keep arm permanently below lidar scan plane and enforce stowed arm during mapping/nav.
- Occlusions:
  - Limited robot-body occlusion is acceptable.
  - Avoid configurations where lidar only sees a narrow forward slot.
- SLAM cleanup:
  - If needed, apply minimum-range filtering to ignore near-field robot structure.
- Operational notes from vendor guidance:
  - Warm up lidar for >2 minutes before accuracy-sensitive mapping.
  - Avoid strong light interference and direct outdoor sunlight.

### OAK-D Lite Placement

- Mechanical mounting options:
  - 1/4-20 tripod mount.
  - VESA 7.5 cm pattern with M4 screws.
- Orientation:
  - Rigid forward-facing mount; avoid vibration/wobble.
  - Set height/pitch to capture both:
    - floor workspace for pick tasks
    - objects at normal working distance.
- Depth behavior:
  - Keep expected targets inside the camera’s practical depth band.
  - Respect minimum-Z limits in close-range manipulation setups.
- Power/connectivity:
  - Use USB3 ports/cables for stable RGB+depth throughput.
  - If device browns out or disconnects under load, use a powered hub or supplemental power.

## Transform Measurement Mini-Checklist

Before running SLAM/Nav2:

1. Measure `base_link -> lidar_link` offset: `x, y, z, yaw`.
2. Measure `base_link -> camera_link` offset: `x, y, z, yaw`.
3. Put values into URDF/Xacro.
4. Verify TF tree and transform directionality:

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

Expected:

- TF tree includes `map -> odom -> base_link` and sensor frames.
- Echo outputs stable, non-jumping transforms.
