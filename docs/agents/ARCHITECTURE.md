# Architecture

Back: [Agent Index](INDEX.md) | [Root Start Here](../../AGENTS.md)

## Text Diagram
```text
OAK-D Lite (USB3)  ----> |
RPLIDAR A1 (USB-serial) ->| Jetson (sensors + autonomy + arm + ROS/task runtime)
RoArm-M2-S (USB-serial)-> |

Jetson -- USB networking (192.168.7.1 <-> 192.168.7.2) --> BeagleBone

BeagleBone PRU0 <--- Encoder A/B signals (4 wheels)
BeagleBone PRU1 ---> Sabertooth 2x60 S1 (serial)
BeagleBone GPIO ---> Sabertooth 2x60 S2 (stop line)
```

## Safety Layers
- Boot default is STOP.
- Explicit ARM is required before non-zero motor commands.
- Watchdog on host and PRU1 forces stop on command timeout.

## Key Config Files
- `beaglebone/host_daemon/config.yaml`
- `jetson/udev/99-robot-devices.rules`
- `jetson/systemd/robot-jetson-stack.service`
