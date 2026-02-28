# Debug Playbook

Back: [Agent Index](INDEX.md) | [Root Start Here](../../AGENTS.md)

## Can’t SSH to BeagleBone over USB
BeagleBone side:
```bash
ip addr
ip route
systemctl status systemd-networkd --no-pager
sudo cat /etc/systemd/network/usb0.network
```

Mac side:
```bash
ifconfig en11
networksetup -listnetworkserviceorder
```

Then test from host:
```bash
ping -c 3 192.168.7.2
ssh debian@192.168.7.2
```

## PRU telemetry not live
```bash
ls -l /dev/uio*
for u in /sys/class/uio/uio*; do echo "== $u"; cat "$u/name"; done
journalctl -u bbb-base-daemon.service -b -n 120 --no-pager -l
```

Check API:
```bash
curl -s http://127.0.0.1:8080/api/status | jq '.dry_run, .pru, .encoder.timestamp_us, .encoder.counts, .encoder.velocity_tps'
```

## UIO map details
```bash
lsmod | grep -E 'uio|pruss'
for m in /sys/class/uio/uio0/maps/map*; do
  echo "== $m"
  cat "$m/name"
  cat "$m/addr"
  cat "$m/size"
done
```

## Moving-map TF alignment issues
Check TF chain and odom source:
```bash
ros2 topic hz /tf
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link oak-d-base-frame
```

If `odom->base_link` is stale or missing, first check for duplicate base TF publishers:
```bash
pgrep -af "base_tf.launch.py|bbb_odom_tf_bridge|base_bringup"
```

Expected: only one effective `base_bringup`/`bbb_odom_tf_bridge` publisher path.

If you are using `./jetson/scripts/run_stack.sh`, do not manually launch
`ros2 launch base_bringup base_tf.launch.py` at the same time.

If base TF is not running and stack-managed launch is disabled/unavailable, start it manually:
```bash
ros2 launch base_bringup base_tf.launch.py
```

## Encoder counts wrong direction or noisy
Checks:
- Confirm encoder A/B are buffered from 5V domain to 3.3V PRU inputs.
- Confirm common ground between encoder supply and BeagleBone.

Log live encoder data:
```bash
python3 beaglebone/tools/log_encoders.py --period 0.1
```

## Sabertooth not responding
Validate control chain:
```bash
python3 beaglebone/tools/cli_test.py status
python3 beaglebone/tools/cli_test.py arm
python3 beaglebone/tools/cli_test.py watch
```

Verify:
- S1 serial path/wiring is correct.
- S2 stop line state is released only when armed.
- ARM gate has been explicitly set.
- Watchdog is not tripping (`safety.command_timeout_ms` in config).
