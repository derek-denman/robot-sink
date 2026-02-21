# Known Issues

Back: [Agent Index](INDEX.md) | [Root Start Here](../../AGENTS.md)

## 1) `pip` fails with `externally-managed-environment`
- Symptom: dependency install fails on Debian with PEP 668 error.
- Diagnosis: system Python is externally managed; system `pip` install is blocked.
- Fix: use repo venv workflow:
  ```bash
  ./beaglebone/scripts/install_deps.sh
  ```
  This creates/updates `beaglebone/host_daemon/.venv` and installs requirements there.

## 2) PRU Software Support Package not available in apt
- Symptom: `build_pru.sh` cannot find PRU SSP headers/linker command file.
- Diagnosis: `pru-software-support-package` may be unavailable on current image.
- Fix:
  ```bash
  git clone https://github.com/TexasInstruments/pru-software-support-package.git /home/debian/pru-software-support-package
  export PRU_SSP=/home/debian/pru-software-support-package
  export PRU_CMD_FILE=/home/debian/pru-software-support-package/include/am335x-pru.cmd
  ./beaglebone/scripts/build_pru.sh
  ```
  If needed, symlink expected command file name to match build expectations.

## 3) PRU remoteproc RPMsg boot failure on `6.19.0-bone8`
- Symptom: `deploy_firmware.sh` fails to start PRU firmware.
- Diagnosis: `dmesg` shows `.kick method not defined` and boot failure `-22`.
- Current status: unresolved in current baseline; do not treat as fixed.
- Mitigation options:
  - use a kernel/DT combo known to support PRU RPMsg kick, or
  - run diagnostic firmware paths that avoid RPMsg vdev while isolating base I/O.

## 4) `bb-usb-gadgets` unstable (`Device or resource busy` / missing IPv4)
- Symptom: USB gadget networking is flaky or comes up without usable IPv4.
- Diagnosis: gadget and network manager interactions are racing.
- Fix: use stable `systemd-networkd` config on BeagleBone `usb0` with static `192.168.7.2/24` and restart `systemd-networkd`.
