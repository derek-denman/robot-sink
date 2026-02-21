# Environments

Back: [Agent Index](INDEX.md) | [Root Start Here](../../AGENTS.md)

## Jetson Environment
- Docs: `jetson/docs/*`
- Key scripts:
  - `jetson/scripts/install_jetson_deps.sh`
  - `jetson/scripts/run_stack.sh`
  - `jetson/scripts/setup_udev_rules.sh`
  - `jetson/scripts/setup_bb_usb_network.sh`

## BeagleBone Environment
- Docs: `beaglebone/docs/*`
- Key scripts:
  - `beaglebone/scripts/install_deps.sh`
  - `beaglebone/scripts/build_pru.sh`
  - `beaglebone/scripts/deploy_firmware.sh`
  - `beaglebone/scripts/enable_services.sh`

## Version Checks (both machines)
```bash
uname -r
cat /etc/os-release
python3 --version
```

## Python Packaging Note (Debian PEP 668)
- On modern Debian, system Python is externally managed.
- Install Python deps in a venv (example already used by this repo):
  - `beaglebone/host_daemon/.venv`
- Do not rely on system `pip install` for host daemon dependencies.
