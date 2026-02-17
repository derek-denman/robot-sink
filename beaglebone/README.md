# BeagleBone MVP Base Controller Stack

Complete BBGG base-controller stack for the living-room toy tidy robot MVP:

- `pru_fw/pru0_encoders`: 4-wheel quadrature decode + velocity + rpmsg API
- `pru_fw/pru1_sabertooth`: Sabertooth serial TX (packetized + simplified), watchdog, ramp, estop gate
- `host_daemon`: safe-by-default daemon with REST + WebSocket + static GUI serving
- `gui/static`: local web UI for arm/estop/motor + live telemetry
- `tools`: CLI and logger for bench testing
- `scripts`: install/build/deploy/service helpers

## Quick Start

1. Install dependencies on BBGG:
   - `./beaglebone/scripts/install_deps.sh`
2. Build PRU firmware:
   - `./beaglebone/scripts/build_pru.sh`
3. Deploy and start PRUs via remoteproc:
   - `./beaglebone/scripts/deploy_firmware.sh`
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

## Config

Edit `beaglebone/host_daemon/config.yaml` for all pin mappings and tunables:

- encoder pin map (`pins.encoders.*`)
- encoder runtime (`encoder.sample_period_us`, `encoder.velocity_interval_ms`, `encoder.stream_hz`)
- PRU rpmsg devices
- Sabertooth mode/baud/address/tx bit/ramping/watchdogs
- S2 stop GPIO
- API host/port rates

Details: see `beaglebone/docs/`.
