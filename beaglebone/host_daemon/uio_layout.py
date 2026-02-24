"""Shared PRU<->host layout definitions for UIO transport."""

from __future__ import annotations

import ctypes

UIO_LAYOUT_MAGIC = 0x42425549  # "BBUI"
UIO_LAYOUT_VERSION = 1

# Host status bits.
UIO_HOST_STATUS_LAYOUT_OK = 1 << 0
UIO_HOST_STATUS_ARMED = 1 << 1
UIO_HOST_STATUS_ESTOP = 1 << 2

# PRU0 status bits.
UIO_PRU0_STATUS_READY = 1 << 0
UIO_PRU0_STATUS_ENCODER_ACTIVE = 1 << 1

# PRU1 status bits.
UIO_PRU1_STATUS_READY = 1 << 0
UIO_PRU1_STATUS_MOTOR_ACTIVE = 1 << 1


class UIOSharedLayout(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint16),
        ("size", ctypes.c_uint16),
        ("host_status_bits", ctypes.c_uint32),
        ("pru0_status_bits", ctypes.c_uint32),
        ("pru1_status_bits", ctypes.c_uint32),
        ("host_heartbeat", ctypes.c_uint32),
        ("pru0_heartbeat", ctypes.c_uint32),
        ("pru1_heartbeat", ctypes.c_uint32),
        ("host_time_us", ctypes.c_uint32),
        ("pru0_time_us", ctypes.c_uint32),
        ("pru1_time_us", ctypes.c_uint32),
        ("encoder_timestamp_us", ctypes.c_uint32),
        ("encoder_counts", ctypes.c_int32 * 4),
        ("encoder_velocity_tps", ctypes.c_int32 * 4),
        ("encoder_sample_period_us", ctypes.c_uint32),
        ("encoder_velocity_interval_us", ctypes.c_uint32),
        ("encoder_stream_hz", ctypes.c_uint32),
        ("encoder_reset_token", ctypes.c_uint32),
        ("motor_command_left", ctypes.c_int16),
        ("motor_command_right", ctypes.c_int16),
        ("motor_applied_left", ctypes.c_int16),
        ("motor_applied_right", ctypes.c_int16),
        ("motor_command_seq", ctypes.c_uint32),
        ("motor_command_time_us", ctypes.c_uint32),
        ("motor_watchdog_ms", ctypes.c_uint32),
        ("motor_safe_stop_ms", ctypes.c_uint16),
        ("motor_ramp_step", ctypes.c_uint16),
        ("motor_control_period_us", ctypes.c_uint32),
        ("sabertooth_baud", ctypes.c_uint32),
        ("sabertooth_mode", ctypes.c_uint8),
        ("sabertooth_address", ctypes.c_uint8),
        ("sabertooth_tx_bit", ctypes.c_uint8),
        ("estop_asserted", ctypes.c_uint8),
        ("armed", ctypes.c_uint8),
        ("reserved_u8", ctypes.c_uint8 * 3),
        ("reserved_u32", ctypes.c_uint32 * 16),
    ]


UIO_SHARED_LAYOUT_SIZE = ctypes.sizeof(UIOSharedLayout)


class LayoutValidationError(RuntimeError):
    """Raised when the mapped shared-memory header does not match ABI."""


def validate_layout_header(layout: UIOSharedLayout) -> None:
    expected_size = UIO_SHARED_LAYOUT_SIZE
    if layout.magic != UIO_LAYOUT_MAGIC:
        raise LayoutValidationError(
            f"layout magic mismatch: expected=0x{UIO_LAYOUT_MAGIC:08x} got=0x{layout.magic:08x}"
        )
    if layout.version != UIO_LAYOUT_VERSION:
        raise LayoutValidationError(
            f"layout version mismatch: expected={UIO_LAYOUT_VERSION} got={layout.version}"
        )
    if layout.size != expected_size:
        raise LayoutValidationError(
            f"layout size mismatch: expected={expected_size} got={layout.size}"
        )
