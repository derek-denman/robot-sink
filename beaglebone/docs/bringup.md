# Bring-Up Guide (UIO Runtime)

## 1) Wiring + Safety Preconditions

- Verify common ground between BBGG and Sabertooth COM/0V.
- Do **not** connect any external source to Sabertooth 5V/BEC terminal.
- Keep S2 wired as dedicated stop line.
- Start bench tests with drive wheels lifted/off-ground.

## 2) Install Dependencies

```bash
./beaglebone/scripts/install_deps.sh
```

## 3) Select UIO PRU Overlay

```bash
grep -E '^(uname_r|uboot_overlay_pru|uboot_overlay_addr4)=' /boot/uEnv.txt || true
```

Expected overlay:

- `uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo`

If needed:

```bash
./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --plan
./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --apply
sudo reboot
```

## 4) Build PRU Firmware (ELF + BIN)

```bash
export PRU_SSP=~/pru-software-support-package
export PRU_CMD_FILE=$PRU_SSP/include/am335x-pru.cmd
./beaglebone/scripts/build_pru.sh
```

Expected outputs:

- `beaglebone/pru_fw/pru0_encoders/am335x-pru0-fw`
- `beaglebone/pru_fw/pru0_encoders/am335x-pru0-fw.bin`
- `beaglebone/pru_fw/pru1_sabertooth/am335x-pru1-fw`
- `beaglebone/pru_fw/pru1_sabertooth/am335x-pru1-fw.bin`

## 5) Start Daemon (manual run first)

```bash
python3 beaglebone/host_daemon/bbb_base_daemon.py \
  --config beaglebone/host_daemon/config.yaml
```

Dry run remains available as explicit debug option only:

```bash
python3 beaglebone/host_daemon/bbb_base_daemon.py \
  --config beaglebone/host_daemon/config.yaml \
  --dry-run
```

## 6) Enable Service + UIO Permissions

```bash
./beaglebone/scripts/enable_services.sh
```

This installs:

- `bbb-base-daemon.service`
- `99-pruss-uio.rules` so `User=debian` can open `/dev/uio*`

## 7) PRU-UIO Verification (required)

### A) UIO devices exist

```bash
ls -l /dev/uio*
for i in /sys/class/uio/uio*; do echo "== $i"; cat "$i/name"; done
```

### B) Deep UIO visibility

```bash
lsmod | grep -E 'uio|pruss'
for u in /sys/class/uio/uio*; do echo "== $u"; cat "$u/name"; done
for m in /sys/class/uio/uio0/maps/map*; do echo "== $m"; cat "$m/name"; cat "$m/addr"; cat "$m/size"; done
```

### C) API shows live encoder telemetry

```bash
curl -s http://127.0.0.1:8080/api/status | jq '.dry_run, .pru, .encoder.timestamp_us, .encoder.counts, .encoder.velocity_tps'
sleep 1
curl -s http://127.0.0.1:8080/api/status | jq '.encoder.timestamp_us'
```

Expected:

- `dry_run=false`
- `pru.encoder_online=true` (when PRU firmware is running)
- `timestamp_us` increments between calls
- counts/velocity change when wheels spin

### D) Logs are clean

```bash
journalctl -u bbb-base-daemon.service -b -n 200 --no-pager -l
```

Expected:

- no `rpmsg_open` attempts
- UIO discovery/layout/heartbeat diagnostics present

## 8) GUI + Bench Plan

Open `http://<beaglebone-ip>:8080` and use sequence:

1. Arm
2. Move sliders at low values
3. Trigger E-STOP
4. Re-arm after hold period

Bench sequence:

1. No motors connected: scope PRU1 TX waveform.
2. Motors off-ground: verify direction, S2 stop, watchdog.
3. Hand-spin wheels: verify encoder counts/sign.
4. Low-speed smoke test with immediate estop checks.
