# Agent Handbook Index

Back: [Root Start Here](../../AGENTS.md)

## Table Of Contents
- [Architecture](ARCHITECTURE.md)
- [Environments](ENVIRONMENTS.md)
- [Build, Deploy, Run](BUILD_DEPLOY_RUN.md)
- [Networking](NETWORKING.md)
- [Known Issues](KNOWN_ISSUES.md)
- [Debug Playbook](DEBUG_PLAYBOOK.md)
- [Contributing For Agents](CONTRIBUTING_FOR_AGENTS.md)

## New Agent Checklist (10 minutes)
1. Read `AGENTS.md`.
2. Identify the target machine for this task: Jetson or BeagleBone.
3. Check versions on target machine:
   ```bash
   uname -r
   cat /etc/os-release
   python3 --version
   ```
4. Verify USB networking path expected for this task:
   ```bash
   ip -4 addr
   ping -c 2 192.168.7.2
   ```
5. Verify expected scripts exist before running anything:
   ```bash
   ls -l jetson/scripts/run_stack.sh jetson/scripts/setup_bb_usb_network.sh
   ls -l beaglebone/scripts/install_deps.sh beaglebone/scripts/build_pru.sh beaglebone/scripts/deploy_firmware.sh
   ls -l sync_bb.sh
   ```
