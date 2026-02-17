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

## 3) Build PRU Firmware

```bash
./beaglebone/scripts/build_pru.sh
```

Expected outputs:

- `beaglebone/pru_fw/pru0_encoders/am335x-pru0-fw`
- `beaglebone/pru_fw/pru1_sabertooth/am335x-pru1-fw`

## 4) Deploy Firmware + Start PRUs

```bash
./beaglebone/scripts/deploy_firmware.sh
```

## 5) Start Daemon (manual run first)

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

## 6) Validate API + Encoder Flow

```bash
python3 beaglebone/tools/cli_test.py status
python3 beaglebone/tools/cli_test.py arm
python3 beaglebone/tools/cli_test.py watch
```

Reset encoders:

```bash
python3 beaglebone/tools/cli_test.py reset-encoders
```

## 7) GUI

Open:

- `http://<beaglebone-ip>:8080`

Use sequence:

1. Arm
2. Move sliders at low values
3. Trigger E-STOP
4. Re-arm after hold period

## 8) Enable Service

```bash
./beaglebone/scripts/enable_services.sh
```

## 9) Bench Plan

1. **No motors connected**: verify PRU1 TX waveform on selected TX pin and baud timing.
2. **Motors off-ground**: verify direction, S2 stop action, and watchdog trip behavior.
3. **Hand-spin wheels**: verify encoder counts and sign in GUI/CLI.
4. **Low-speed smoke test**: short manual drive bursts with immediate E-stop checks.
