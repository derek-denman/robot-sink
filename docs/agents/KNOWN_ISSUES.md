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

## 3) PRU remoteproc RPMsg boot failure (`.kick method not defined`, `Boot failed: -22`)
- Symptom: `deploy_firmware.sh` fails to start `remoteproc1/remoteproc2`.
- Diagnosis: kernel logs show missing PRU virtio kick support for RPMsg firmware.
- Status: scripted remediation path is now available.
- Use:
  ```bash
  ./beaglebone/scripts/pru_rpmsg_fixup.sh --plan
  ./beaglebone/scripts/pru_rpmsg_fixup.sh --apply
  sudo reboot
  ./beaglebone/scripts/deploy_firmware.sh
  ```
- Notes:
  - `pru_rpmsg_fixup.sh` only selects kernels that exist in `/boot/vmlinuz-*`.
  - Overlay names encode expected kernel series (for example, `...-4-19-TI-...`).
  - Detailed guidance: `beaglebone/docs/troubleshooting.md` and `beaglebone/docs/bringup.md`.

### 3a) Kernel Oops in RPMsg kick path (`pru_rproc_kick`)
- Symptom: `dmesg` shows `Internal error: Oops` with `PC is at pru_rproc_kick...` after daemon or RPMsg writes.
- Diagnosis: active kernel/overlay combination is unstable for RPMsg kick path on this image.
- Note: some `6.1-ti` stacks are known unstable for PRU RPMsg; prefer `5.10-ti` when available.
- Use:
  ```bash
  ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --plan
  ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --apply
  sudo reboot
  ./beaglebone/scripts/deploy_firmware.sh
  ```
- Revert:
  ```bash
  ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --revert
  sudo reboot
  ```

### 3b) PRU Boot Fails `IRQ vring not found` / `IRQ kick not found` (`Boot failed: -6`)
- Symptom: `pru-rproc ... IRQ vring not found` or `IRQ kick not found` and PRU remoteproc boot fails with `-6`.
- Diagnosis: PRU DT node lacks full RPMsg interrupt wiring (`vring` and `kick`) for `pru_rproc`.
- Use:
  ```bash
  ./beaglebone/scripts/pru_rproc_irq_fix.sh --plan
  ./beaglebone/scripts/pru_rproc_irq_fix.sh --apply
  sudo reboot
  ./beaglebone/scripts/deploy_firmware.sh
  ```
- Revert:
  ```bash
  ./beaglebone/scripts/pru_rproc_irq_fix.sh --revert
  sudo reboot
  ```

## 4) `bb-usb-gadgets` unstable (`Device or resource busy` / missing IPv4)
- Symptom: USB gadget networking is flaky or comes up without usable IPv4.
- Diagnosis: gadget and network manager interactions are racing.
- Fix: use stable `systemd-networkd` config on BeagleBone `usb0` with static `192.168.7.2/24` and restart `systemd-networkd`.
