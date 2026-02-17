# Wiring and Pins

All mappings below are defaults from `beaglebone/host_daemon/config.yaml` and are intended to be edited in config first.

## Encoder Inputs (PRU0 R31 bits 0..7)

- `wheel0_A`: `P8_45` (`pru0_r31_0`)
- `wheel0_B`: `P8_46` (`pru0_r31_1`)
- `wheel1_A`: `P8_43` (`pru0_r31_2`)
- `wheel1_B`: `P8_44` (`pru0_r31_3`)
- `wheel2_A`: `P8_41` (`pru0_r31_4`)
- `wheel2_B`: `P8_42` (`pru0_r31_5`)
- `wheel3_A`: `P8_39` (`pru0_r31_6`)
- `wheel3_B`: `P8_40` (`pru0_r31_7`)

Notes:

- Encoders are 5V-powered and must be level shifted/buffered to 3.3V before BBGG input.
- Firmware assumes push-pull quadrature A/B with Gray-code transition handling.

## Sabertooth Serial TX (PRU1)

Default candidate:

- `tx_header_pin`: `P9_31`
- `tx_pru1_r30_bit`: `0`

Because PRU pinmux availability can vary by cape/overlay, treat this as a known-good starting candidate and update both:

1. `config.yaml` (`sabertooth.tx_pru1_r30_bit`, `pins.sabertooth.tx_header_pin`)
2. BB pinmux/overlay so selected header pin is mapped to `pru1_r30_*`

## Sabertooth Stop Line (S2)

Default:

- `s2_stop_gpio_number`: `48` (example Linux GPIO number)
- `s2_stop_active_high`: `true`

Behavior:

- Daemon asserts this line on boot, estop, and watchdog trip.
- Line is released only after explicit arm request.

## Sabertooth Power/Ground Rules

- BBGG GND and Sabertooth COM/0V must share common ground.
- Do not back-feed the Sabertooth 5V/BEC pin.
- Keep S2 as dedicated safety path independent from S1 serial data.
