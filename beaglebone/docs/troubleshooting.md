# Troubleshooting

## 1) PRU Firmware Not Running

Check remoteproc state:

```bash
for rp in /sys/class/remoteproc/remoteproc*; do
  echo "${rp}: $(cat ${rp}/name 2>/dev/null) state=$(cat ${rp}/state 2>/dev/null)"
done
```

If stopped:

```bash
./beaglebone/scripts/deploy_firmware.sh
```

Then inspect kernel logs:

```bash
dmesg -T | grep -Ei "pru|remoteproc|rpmsg"
```

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
make -C beaglebone/pru_fw
```

If build fails with `pru-gcc: No such file or directory` on Debian `trixie`, install the TI PRU compiler package and rerun the wrapper script:

```bash
sudo apt install -y ti-pru-cgt-v2.3
./beaglebone/scripts/build_pru.sh
```

`build_pru.sh` now supports either:

- `pru-gcc` (preferred when available), or
- TI tools `clpru` + `lnkpru` (from `ti-pru-cgt-v2.3`).

If TI link fails with unresolved `pru_rpmsg_*` symbols, your PRU SSP path is missing RPMsg implementation objects/libraries. The script auto-detects either:

- `rpmsg_lib.lib` / `pru_rpmsg*.lib`, or
- `pru_rpmsg.c`

If auto-detect fails, set one explicitly:

```bash
PRU_TI_RPMSG_LIB=/path/to/rpmsg_lib.lib ./beaglebone/scripts/build_pru.sh
# or
PRU_TI_RPMSG_SRC=/path/to/pru_rpmsg.c ./beaglebone/scripts/build_pru.sh
```

Deploy:

```bash
./beaglebone/scripts/deploy_firmware.sh
```

Verify loaded names and recent logs:

```bash
ls -l /lib/firmware/am335x-pru*-fw
journalctl -u bbb-base-daemon.service -n 200 --no-pager
```
