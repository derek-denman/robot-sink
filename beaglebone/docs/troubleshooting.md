# Troubleshooting (UIO Runtime)

## 1) UIO devices missing

```bash
ls -l /dev/uio*
for u in /sys/class/uio/uio*; do echo "== $u"; cat "$u/name"; done
lsmod | grep -E 'uio|pruss'
```

If no UIO devices appear, confirm overlay:

```bash
grep -E '^(uname_r|uboot_overlay_pru|uboot_overlay_addr4)=' /boot/uEnv.txt || true
```

Expected:

- `uboot_overlay_pru=AM335X-PRU-UIO-00A0.dtbo`

Apply if needed:

```bash
./beaglebone/scripts/pru_rpmsg_uio_workaround.sh --apply
sudo reboot
```

## 2) Daemon cannot open /dev/uio*

```bash
ls -l /dev/uio*
systemctl cat bbb-base-daemon.service | grep -E '^(User|Group|ExecStart)='
```

Fix:

```bash
./beaglebone/scripts/enable_services.sh
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=uio
sudo systemctl restart bbb-base-daemon.service
```

Expected:

- `/dev/uio*` group allows daemon user (`debian`) read/write
- daemon starts without permission errors

## 3) Layout mismatch / host cannot read telemetry

```bash
journalctl -u bbb-base-daemon.service -b -n 200 --no-pager -l
```

Look for:

- `layout sanity check failed`
- `layout magic/version/size mismatch`

Fix path:

1. Rebuild firmware and `.bin` files.
2. Verify `config.yaml` offsets (`shared_offset`, `pru*_iram_offset`, `pru*_ctrl_offset`).
3. Restart service.

```bash
./beaglebone/scripts/build_pru.sh
./beaglebone/scripts/enable_services.sh
```

## 4) Encoder telemetry stale (`timestamp_us` not advancing)

```bash
curl -s http://127.0.0.1:8080/api/status | jq '.dry_run, .pru, .encoder.timestamp_us, .encoder.counts, .encoder.velocity_tps'
sleep 1
curl -s http://127.0.0.1:8080/api/status | jq '.encoder.timestamp_us'
```

If `timestamp_us` does not change:

- ensure PRU0 heartbeat is advancing (check daemon logs)
- confirm encoder A/B wiring and level shifting
- confirm wheel spin changes counts

## 5) PRU map sanity checks

```bash
for m in /sys/class/uio/uio0/maps/map*; do
  echo "== $m"
  cat "$m/name"
  cat "$m/addr"
  cat "$m/size"
done
```

Expected on current target probe:

- `map0 name=pruss addr=0x4a300000 size=0x00080000`
- `map1 name=ddr addr=0x9c940000 size=0x00040000`

## 6) Motors do not move

- Confirm mode/baud/address in `config.yaml` match Sabertooth DIP setup.
- Confirm S1 receives TX from selected PRU1 TX pin.
- Confirm S2 stop line is released only when armed.
- Confirm no active estop/watchdog state in `/api/status`.

## 7) Immediate watchdog trips

- Increase `safety.command_timeout_ms` in `config.yaml`.
- Keep GUI/CLI sending commands periodically while armed.

## 8) Confirm no RPMsg runtime dependency remains

```bash
journalctl -u bbb-base-daemon.service -b -n 200 --no-pager -l | grep -i rpmsg || true
```

Expected:

- no `rpmsg_open` runtime attempts
