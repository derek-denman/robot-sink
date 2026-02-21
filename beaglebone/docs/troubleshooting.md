# Troubleshooting

## 1) PRU remoteproc fails: `.kick method not defined` / `Boot failed -22`

### Symptom

`./beaglebone/scripts/deploy_firmware.sh` fails to start one or both PRUs, and `dmesg` includes lines like:

```text
rproc-virtio ... .kick method not defined for 4a334000.pru
remoteproc ... failed to probe subdevices ... -22
remoteproc ... Boot failed: -22
```

### Cause

The firmware can load as ELF, but the active kernel + device tree + PRU overlay combination does not provide RPMsg virtio kick support.

### Diagnose

```bash
for r in /sys/class/remoteproc/remoteproc1 /sys/class/remoteproc/remoteproc2; do echo "== $r =="; cat $r/name; cat $r/firmware; cat $r/state; done
dmesg | egrep -i 'remoteproc|rproc-virtio|rpmsg|pru' | tail -n 80
ls -l /dev/rpmsg* /dev/ttyRPMSG* 2>/dev/null || true
uname -r
grep -E '^(uname_r|uboot_overlay_pru)=' /boot/uEnv.txt || true
```

If `uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo`, RPMsg remoteproc firmware is not expected to work.

### Fix path (safe + reversible)

Use the fixup script:

```bash
./beaglebone/scripts/pru_rpmsg_fixup.sh --plan
./beaglebone/scripts/pru_rpmsg_fixup.sh --apply
sudo reboot
./beaglebone/scripts/deploy_firmware.sh
```

If needed, restore last backup:

```bash
./beaglebone/scripts/pru_rpmsg_fixup.sh --revert
sudo reboot
```

Notes:

- The script only selects kernels already installed in `/boot/vmlinuz-*`.
- Overlay names imply kernel series compatibility. Example: `AM335X-PRU-RPROC-4-19-TI-...` is for a 4.19-ti series kernel.
- If no installed kernel has a matching `AM335X-PRU-RPROC-*` overlay, install a compatible `*-ti` kernel (commonly 5.10-ti), then rerun `--plan`.

## 2) rpmsg Device Missing

Expected defaults:

- `/dev/rpmsg_pru30`
- `/dev/rpmsg_pru31`

If names differ, update `beaglebone/host_daemon/config.yaml` (`pru.encoder_rpmsg_dev`, `pru.motor_rpmsg_dev`).

## 3) Daemon Runs But No Encoder Updates

- Confirm encoder input pins are muxed to PRU0 inputs.
- Confirm 5V->3.3V buffering is present and grounds are common.
- Spin each wheel by hand and monitor:

```bash
python3 beaglebone/tools/log_encoders.py --period 0.1
```

## 4) Motors Do Not Move

- Confirm mode/baud/address in `config.yaml` match Sabertooth DIP setup.
- Confirm S1 receives TX from selected PRU1 TX pin.
- Confirm S2 is released only after arm.
- Confirm no active estop/watchdog state in `/api/status`.

## 5) Immediate Watchdog Trips

Likely command update timeout is too short for your test loop.

- Increase `safety.command_timeout_ms` in `config.yaml`.
- Keep GUI/CLI sending commands periodically while armed.

## 6) Validate Firmware Build/Load

Build:

```bash
export PRU_SSP=~/pru-software-support-package
export PRU_CMD_FILE=$PRU_SSP/include/am335x-pru.cmd
./beaglebone/scripts/build_pru.sh
```

Deploy:

```bash
./beaglebone/scripts/deploy_firmware.sh
```

Validate:

```bash
for r in /sys/class/remoteproc/remoteproc1 /sys/class/remoteproc/remoteproc2; do echo "== $r =="; cat $r/name; cat $r/firmware; cat $r/state; done
dmesg | egrep -i 'remoteproc|rproc-virtio|rpmsg|pru' | tail -n 80
ls -l /dev/rpmsg* /dev/ttyRPMSG* 2>/dev/null || true
```
