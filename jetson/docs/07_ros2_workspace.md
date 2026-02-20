# 07 - ROS 2 Workspace

## ROS Distribution Choice

Target runtime: ROS 2 Humble.

Reasoning:

- Humble aligns with Ubuntu 22.04 (`jammy`) and is a stable LTS target.
- JetPack 6.x uses Ubuntu 22.04 base and is the cleanest path for Humble apt installs.

If your Jetson image is Ubuntu 20.04 (`focal`), Humble apt binaries are not the standard path. Options:

1. Move to a JetPack release with Ubuntu 22.04 base.
2. Build Humble from source.
3. Run Humble inside Docker.

## Install Dependencies

```bash
cd ~/robot-sink
./jetson/scripts/install_jetson_deps.sh
```

## Build Workspace

```bash
source /opt/ros/humble/setup.bash
cd ~/robot-sink/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Expected Runtime Components

Minimum bench bring-up target:

- OAK-D Lite node (`depthai_ros_driver` if installed)
- RPLIDAR node (`rplidar_ros`)
- RoArm serial node (custom or stub)
- Task stack (custom or placeholder)
- BeagleBone link bridge (UDP stub included)

`jetson/scripts/run_stack.sh` will run real nodes when present, else fallback stubs so process supervision and logging still work.
