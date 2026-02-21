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

## PRU firmware won’t start
```bash
for rp in /sys/class/remoteproc/remoteproc1 /sys/class/remoteproc/remoteproc2; do
  echo "$rp state=$(cat "$rp/state" 2>/dev/null) firmware=$(cat "$rp/firmware" 2>/dev/null)"
done
dmesg -T | tail -n 80
```

Retry deploy:
```bash
./beaglebone/scripts/deploy_firmware.sh
```

## RPMsg device nodes missing
```bash
dmesg -T | grep -i rpmsg
ls -l /dev/rpmsg*
```

If nodes differ from defaults, update:
- `beaglebone/host_daemon/config.yaml`

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
