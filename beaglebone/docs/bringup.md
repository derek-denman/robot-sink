# Bring-Up Guide

## 1) Wiring + Safety Preconditions

- Verify common ground between BBGG and Sabertooth COM/0V.
- Do **not** connect any external source to Sabertooth 5V/BEC terminal.
- Keep S2 wired as dedicated stop line.
- Start bench tests with drive wheels lifted/off-ground.

## 2) Install Dependencies

```bash
./beaglebone/scripts/install_deps.sh
```

## 3) Kernel/overlay prerequisites for RPMsg

RPMsg PRU firmware requires a **remoteproc/RPROC** kernel + overlay pair.

Quick checks:

```bash
uname -r
grep -E '^(uname_r|uboot_overlay_pru)=' /boot/uEnv.txt || true
ls -1 /boot/vmlinuz-* | sed 's|.*/vmlinuz-||'
ls -1 /boot/dtbs/$(uname -r)/overlays/AM335X-PRU-*.dtbo 2>/dev/null || true
```

If `uboot_overlay_pru` is `AM335X-PRU-UIO-00A0.dtbo`, or no `AM335X-PRU-RPROC-*` overlay exists for the selected kernel, run:

```bash
./beaglebone/scripts/pru_rpmsg_fixup.sh --plan
# then, if the plan looks correct:
./beaglebone/scripts/pru_rpmsg_fixup.sh --apply
sudo reboot
```

Overlay names like `AM335X-PRU-RPROC-4-19-TI-...` imply a matching kernel series; do not mix unrelated series.
If both `5.10-ti` and `6.1-ti` are installed, prefer `5.10-ti` for PRU RPMsg stability.

## 4) Build PRU Firmware

```bash
export PRU_SSP=~/pru-software-support-package
export PRU_CMD_FILE=$PRU_SSP/include/am335x-pru.cmd
./beaglebone/scripts/build_pru.sh
```

Expected outputs:

- `beaglebone/pru_fw/pru0_encoders/am335x-pru0-fw`
- `beaglebone/pru_fw/pru1_sabertooth/am335x-pru1-fw`

## 5) Deploy Firmware + Start PRUs

```bash
./beaglebone/scripts/deploy_firmware.sh
```

Expected deploy success criteria:

- `remoteproc1` and `remoteproc2` report `state=running`
- firmware entries show `am335x-pru0-fw` and `am335x-pru1-fw`
- RPMsg device nodes are present (`/dev/rpmsg*` and/or `/dev/ttyRPMSG*`)

Validation checks:

```bash
for r in /sys/class/remoteproc/remoteproc1 /sys/class/remoteproc/remoteproc2; do echo "== $r =="; cat $r/name; cat $r/firmware; cat $r/state; done
dmesg | egrep -i 'remoteproc|rproc-virtio|rpmsg|pru' | tail -n 80
ls -l /dev/rpmsg* /dev/ttyRPMSG* 2>/dev/null || true
```

If `deploy_firmware.sh` reports kernel Oops signatures (`pru_rproc_kick`) or boot `-22`, use kernel-hop tooling:

```bash
./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --plan
./beaglebone/scripts/pru_rpmsg_kernel_hop.sh --apply
sudo reboot
./beaglebone/scripts/deploy_firmware.sh
```

If deploy fails with PRU IRQ mapping errors (`Boot failed: -6`, `IRQ vring not found`, or `IRQ kick not found`), apply the PRU IRQ overlay fix:

```bash
./beaglebone/scripts/pru_rproc_irq_fix.sh --plan
./beaglebone/scripts/pru_rproc_irq_fix.sh --apply
sudo reboot
./beaglebone/scripts/deploy_firmware.sh
```

## 6) Start Daemon (manual run first)

```bash
python3 beaglebone/host_daemon/bbb_base_daemon.py \
  --config beaglebone/host_daemon/config.yaml
```

Dry run (no hardware writes):

```bash
python3 beaglebone/host_daemon/bbb_base_daemon.py \
  --config beaglebone/host_daemon/config.yaml \
  --dry-run
```

## 7) RPMsg Stability Regression Check

Run this on BeagleBone after firmware deploy to verify no kernel crash in `pru_rproc_kick` under live traffic:

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

dmesg -T | egrep -i 'remoteproc|rpmsg|pru_rproc_kick|oops|segfault' | tail -n 120

grep -Eiq 'pru_rproc_kick|Internal error: Oops|Boot failed: -22|kick method not defined' /tmp/rpmsg_watch.log \
  && echo "FAIL: RPMsg kernel path unstable" \
  || echo "PASS: no RPMsg kick-path crash signature observed"

kill $DAEMON_PID || true
sudo kill $DMESG_PID || true
```

## 8) Validate API + Encoder Flow

```bash
python3 beaglebone/tools/cli_test.py status
python3 beaglebone/tools/cli_test.py arm
python3 beaglebone/tools/cli_test.py watch
```

Reset encoders:

```bash
python3 beaglebone/tools/cli_test.py reset-encoders
```

## 9) GUI

Open:

- `http://<beaglebone-ip>:8080`

Use sequence:

1. Arm
2. Move sliders at low values
3. Trigger E-STOP
4. Re-arm after hold period

## 10) Enable Service

```bash
./beaglebone/scripts/enable_services.sh
```

## 11) Bench Plan

1. **No motors connected**: verify PRU1 TX waveform on selected TX pin and baud timing.
2. **Motors off-ground**: verify direction, S2 stop action, and watchdog trip behavior.
3. **Hand-spin wheels**: verify encoder counts and sign in GUI/CLI.
4. **Low-speed smoke test**: short manual drive bursts with immediate E-stop checks.
