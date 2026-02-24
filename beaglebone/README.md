# BeagleBone MVP Base Controller Stack

Complete BBGG base-controller stack for the living-room toy tidy robot MVP:

- `pru_fw/pru0_encoders`: 4-wheel quadrature decode + velocity + shared-memory telemetry
- `pru_fw/pru1_sabertooth`: Sabertooth serial TX (packetized + simplified), watchdog, ramp, estop gate
- `host_daemon`: safe-by-default daemon with REST + WebSocket + static GUI serving
- `gui/static`: local web UI for arm/estop/motor + live telemetry
- `tools`: CLI and logger for bench testing
- `scripts`: install/build/deploy/service helpers

## Quick Start (UIO Mode)

1. Install dependencies on BBB:
   - `./beaglebone/scripts/install_deps.sh`
2. Build PRU firmware + `.bin` images:
   - `./beaglebone/scripts/build_pru.sh`
3. Ensure UIO overlay is selected:
   - `grep -E '^(uboot_overlay_pru)=' /boot/uEnv.txt`
   - expected: `uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo`
4. Start daemon manually first:
   - `python3 beaglebone/host_daemon/bbb_base_daemon.py --config beaglebone/host_daemon/config.yaml`
5. Open UI:
   - `http://<beaglebone-ip>:8080`
6. Enable autostart service (after manual validation):
   - `./beaglebone/scripts/enable_services.sh`

## Safety Defaults

- Boot state is always **STOP**.
- Daemon starts with estop asserted; no non-zero motor output until explicit `/api/arm`.
- Watchdog trips estop when command updates stop.
- Estop command asserts stop path and requires explicit re-arm.

## Runtime API

- `GET /api/status`
- `POST /api/arm`
- `POST /api/estop`
- `POST /api/motor`
- `POST /api/reset_encoders`
- `GET /ws`

## PRU-UIO Verification

```bash
ls -l /dev/uio*
for i in /sys/class/uio/uio*; do echo "== $i"; cat "$i/name"; done
curl -s http://127.0.0.1:8080/api/status | jq '.dry_run, .pru, .encoder.timestamp_us, .encoder.counts, .encoder.velocity_tps'
sleep 1
curl -s http://127.0.0.1:8080/api/status | jq '.encoder.timestamp_us'
journalctl -u bbb-base-daemon.service -b -n 200 --no-pager -l
```

Expected:

- `dry_run=false`
- `pru.encoder_online=true` when PRU firmware is active
- `timestamp_us` increases between calls
- encoder counts change when wheels are spun
- no `rpmsg_open` attempts in daemon logs

## Config

Edit `beaglebone/host_daemon/config.yaml` for pin mappings and tunables:

- UIO transport + map offsets (`pru.uio.*`)
- encoder runtime (`encoder.sample_period_us`, `encoder.velocity_interval_ms`)
- Sabertooth mode/baud/address/tx bit/ramping/watchdogs
- S2 stop GPIO
- API host/port rates

Details: see `beaglebone/docs/`.
