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

## 2) Kernel Oops in `pru_rproc_kick` during RPMsg traffic

### Symptom

PRUs appear to load and `/dev/rpmsg_pru30` and `/dev/rpmsg_pru31` may exist, but `dmesg` shows Oops traces such as:

```text
Internal error: Oops
PC is at pru_rproc_kick+0x...
rpmsg_send_offchannel_raw -> virtqueue_kick -> pru_rproc_kick
```

This usually crashes user-space writers (`Segmentation fault` in daemon process after RPMsg writes) and leaves RPMsg unstable.

### Diagnose

```bash
for r in /sys/class/remoteproc/remoteproc1 /sys/class/remoteproc/remoteproc2; do echo "== $r =="; cat $r/name; cat $r/firmware; cat $r/state; done
dmesg -T | egrep -i 'remoteproc|rproc-virtio|rpmsg|pru_rproc_kick|oops|segfault' | tail -n 120
ls -l /dev/rpmsg* /dev/ttyRPMSG* 2>/dev/null || true
```

### Mitigation (kernel hop with backup/revert)

```bash
./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --plan
# optional explicit target:
# ./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --plan --kernel 5.10.168-ti-r83 --overlay AM335X-PRU-RPROC-PRUCAPE-00A0.dtbo

./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --apply
sudo reboot
./beaglebone/scripts/deploy_firmware.sh
```

Revert if needed:

```bash
./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --revert
sudo reboot
```

Notes:

- TI has acknowledged RPMsg instability on some `6.1-ti` PRU remoteproc stacks; prefer `5.10-ti` when available.
- Reference: https://e2e.ti.com/support/processors-group/processors/f/processors-forum/1428220/am6442-remoteproc-kernel-panic-during-rpmsg-operation

### Stability check (post-reboot)

```bash
sudo dmesg -C
sudo dmesg -wH > /tmp/rpmsg_watch.log 2>&1 &
DMESG_PID=$!

sudo /home/debian/robot-sink/beaglebone/host_daemon/.venv/bin/python3 \
  /home/debian/robot-sink/beaglebone/host_daemon/bbb_base_daemon.py \
  --config /home/debian/robot-sink/beaglebone/host_daemon/config.yaml \
  > /tmp/bbb_daemon.log 2>&1 &
DAEMON_PID=$!

sleep 2
for i in $(seq 1 20); do python3 beaglebone/tools/cli_test.py status >/dev/null || true; sleep 0.25; done

grep -Eiq 'pru_rproc_kick|Internal error: Oops|Boot failed: -22|kick method not defined' /tmp/rpmsg_watch.log \
  && echo "FAIL: RPMsg kernel path unstable" \
  || echo "PASS: no RPMsg kick-path crash signature observed"

kill $DAEMON_PID || true
sudo kill $DMESG_PID || true
```

## 3) PRU boot fails with `IRQ vring not found` / `IRQ kick not found` / `Boot failed: -6`

### Symptom

On some `6.1-ti` images, `deploy_firmware.sh` reports:

```text
pru-rproc ... error -ENXIO: IRQ vring not found
pru-rproc ... error -ENXIO: IRQ kick not found
remoteproc ... unable to get vring interrupt, status = -6
remoteproc ... Boot failed: -6
```

### Cause

The running DTB can expose PRU nodes without full RPMsg interrupt wiring (`interrupt-parent` / `interrupts` / `interrupt-names`) expected by `pru_rproc` (`vring` and `kick`).

### Fix path

Use the in-repo IRQ fix overlay helper:

```bash
./beaglebone/scripts/pru_rproc_irq_fix.sh --plan
./beaglebone/scripts/pru_rproc_irq_fix.sh --apply
sudo reboot
./beaglebone/scripts/deploy_firmware.sh
```

Revert if needed:

```bash
./beaglebone/scripts/pru_rproc_irq_fix.sh --revert
sudo reboot
```

## 4) Plan-of-record fallback: UIO overlay + daemon `--dry-run`

### When to use this

Use this when RPMsg is consistently unstable (kernel Oops in `pru_rproc_kick`) across available kernels and overlay fixes.

### Apply

```bash
./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --plan
./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --apply
sudo reboot
```

### Validate after reboot

```bash
grep -E '^(uname_r|uboot_overlay_pru|uboot_overlay_addr4)=' /boot/uEnv.txt || true
systemctl cat bbb-base-daemon.service | grep -E '^ExecStart='
python3 beaglebone/tools/cli_test.py status
```

Expected:

- `uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo`
- `uboot_overlay_addr4` absent
- daemon `ExecStart` includes `--dry-run`
- API status reports `"dry_run": true`

### Revert

```bash
./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --revert
sudo reboot
```

## 5) rpmsg Device Missing

Expected defaults:

- `/dev/rpmsg_pru30`
- `/dev/rpmsg_pru31`

If names differ, update `beaglebone/host_daemon/config.yaml` (`pru.encoder_rpmsg_dev`, `pru.motor_rpmsg_dev`).

## 6) Daemon Runs But No Encoder Updates

- Confirm encoder input pins are muxed to PRU0 inputs.
- Confirm 5V->3.3V buffering is present and grounds are common.
- Spin each wheel by hand and monitor:

```bash
python3 beaglebone/tools/log_encoders.py --period 0.1
```

## 7) Motors Do Not Move

- Confirm mode/baud/address in `config.yaml` match Sabertooth DIP setup.
- Confirm S1 receives TX from selected PRU1 TX pin.
- Confirm S2 is released only after arm.
- Confirm no active estop/watchdog state in `/api/status`.

## 8) Immediate Watchdog Trips

Likely command update timeout is too short for your test loop.

- Increase `safety.command_timeout_ms` in `config.yaml`.
- Keep GUI/CLI sending commands periodically while armed.

## 9) Validate Firmware Build/Load

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
