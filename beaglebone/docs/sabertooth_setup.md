# Sabertooth 2x60 Setup

## Serial Modes Implemented

PRU1 firmware supports:

1. `packetized` (default)
2. `simplified`

Set in `beaglebone/host_daemon/config.yaml`:

```yaml
sabertooth:
  mode: packetized
  baud: 38400
  address: 128
```

## Packetized Serial (default)

- Framed as `[address][command][data][checksum]`
- Checksum: `(address + command + data) & 0x7F`
- Used commands:
  - M1 forward/reverse: `0` / `1`
  - M2 forward/reverse: `4` / `5`

Recommended for deterministic behavior and explicit addressing.

## Simplified Serial

Single-byte commands:

- Motor 1 byte range `1..127` (`64` near stop)
- Motor 2 byte range `128..255` (`192` near stop)

Use only if your DIP switch profile is configured for simplified mode.

## DIP Switch Guidance

Exact DIP positions depend on firmware mode + battery mode selection from the Sabertooth manual.
Use this workflow:

1. Select serial type first (packetized or simplified).
2. Set baud to match `config.yaml`.
3. For packetized mode, set address to match `config.yaml` (`128` default).
4. Power-cycle Sabertooth after DIP changes.

## Bench-Test Procedure

1. **Scope TX line first (motors disconnected).**
   - Probe BB TX pin and ground.
   - Confirm idle-high UART and expected bit period (e.g., ~26 us at 38400 baud).
2. **Connect Sabertooth S1 + COM only.**
   - Keep S2 asserted (safe stop).
3. **Release S2 via arm command and send tiny commands (±100).**
4. **Verify motor direction and stop response.**
5. **Unplug command source or stop sending updates.**
   - Confirm watchdog stop + S2 assertion.

## Safety-First Config Defaults

- PRU watchdog: `200 ms`
- Daemon command timeout: `200 ms`
- Safe-stop hold: `300 ms`
- Arm required after every estop/watchdog event
