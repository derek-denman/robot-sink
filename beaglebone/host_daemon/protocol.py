"""Host-side mirror of beaglebone/pru_fw/common/rpmsg_protocol.h."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Optional

BBB_MSG_MAGIC = 0x42424747
BBB_MSG_VERSION = 1
BBB_MSG_PAYLOAD_MAX = 48

BBB_MODULE_HOST = 0
BBB_MODULE_PRU0 = 1
BBB_MODULE_PRU1 = 2

BBB_MSG_TYPE_COMMAND = 1
BBB_MSG_TYPE_RESPONSE = 2
BBB_MSG_TYPE_STREAM = 3
BBB_MSG_TYPE_HEARTBEAT = 4

BBB_CMD_RESET_COUNTS = 0x10
BBB_CMD_GET_SNAPSHOT = 0x11
BBB_CMD_SET_ENC_PARAMS = 0x12
BBB_CMD_SET_STREAM = 0x13

BBB_CMD_SET_MOTOR = 0x20
BBB_CMD_SET_ESTOP = 0x21
BBB_CMD_PING = 0x22
BBB_CMD_SET_SABER_PARAMS = 0x23

BBB_CMD_ACK = 0x7E
BBB_CMD_NACK = 0x7F

BBB_SABER_MODE_SIMPLIFIED = 0
BBB_SABER_MODE_PACKETIZED = 1

MSG_STRUCT = struct.Struct("<IBBBBHH48sH")
ENC_SNAPSHOT_STRUCT = struct.Struct("<4i4iIHH")
ENC_PARAMS_STRUCT = struct.Struct("<HHHH")
MOTOR_STRUCT = struct.Struct("<hh")
ESTOP_STRUCT = struct.Struct("<B3x")
SABER_PARAMS_STRUCT = struct.Struct("<IBBBxHHHH")
ACK_STRUCT = struct.Struct("<hHI")


@dataclass(frozen=True)
class Message:
    module: int
    msg_type: int
    command: int
    sequence: int
    payload: bytes


def checksum16(data: bytes) -> int:
    total = sum(data)
    total = (total & 0xFFFF) + (total >> 16)
    total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def build_message(
    *,
    module: int,
    msg_type: int,
    command: int,
    sequence: int,
    payload: bytes = b"",
) -> bytes:
    if len(payload) > BBB_MSG_PAYLOAD_MAX:
        raise ValueError("payload too large")

    payload_padded = payload.ljust(BBB_MSG_PAYLOAD_MAX, b"\x00")
    without_checksum = MSG_STRUCT.pack(
        BBB_MSG_MAGIC,
        BBB_MSG_VERSION,
        module,
        msg_type,
        command,
        sequence & 0xFFFF,
        len(payload),
        payload_padded,
        0,
    )
    cs = checksum16(without_checksum[:-2])
    return without_checksum[:-2] + struct.pack("<H", cs)


def parse_message(raw: bytes) -> Optional[Message]:
    if len(raw) < MSG_STRUCT.size:
        return None

    raw = raw[: MSG_STRUCT.size]
    (
        magic,
        version,
        module,
        msg_type,
        command,
        sequence,
        payload_len,
        payload_padded,
        msg_checksum,
    ) = MSG_STRUCT.unpack(raw)

    if magic != BBB_MSG_MAGIC or version != BBB_MSG_VERSION:
        return None
    if payload_len > BBB_MSG_PAYLOAD_MAX:
        return None
    if checksum16(raw[:-2]) != msg_checksum:
        return None

    return Message(
        module=module,
        msg_type=msg_type,
        command=command,
        sequence=sequence,
        payload=payload_padded[:payload_len],
    )


def pack_motor(left: int, right: int) -> bytes:
    return MOTOR_STRUCT.pack(left, right)


def unpack_encoder_snapshot(payload: bytes) -> Optional[dict]:
    if len(payload) < ENC_SNAPSHOT_STRUCT.size:
        return None
    unpacked = ENC_SNAPSHOT_STRUCT.unpack(payload[: ENC_SNAPSHOT_STRUCT.size])
    return {
        "counts": list(unpacked[0:4]),
        "velocity_tps": list(unpacked[4:8]),
        "timestamp_us": unpacked[8],
        "stream_hz": unpacked[9],
        "flags": unpacked[10],
    }


def pack_estop(asserted: bool) -> bytes:
    return ESTOP_STRUCT.pack(1 if asserted else 0)


def pack_enc_params(sample_period_us: int, velocity_interval_ms: int, stream_hz: int) -> bytes:
    return ENC_PARAMS_STRUCT.pack(sample_period_us, velocity_interval_ms, stream_hz, 0)


def pack_stream_config(stream_hz: int) -> bytes:
    return struct.pack("<H", stream_hz)


def pack_sabertooth_params(
    *,
    baud: int,
    mode: int,
    address: int,
    tx_r30_bit: int,
    watchdog_ms: int,
    safe_stop_hold_ms: int,
    ramp_step: int,
    control_period_us: int,
) -> bytes:
    return SABER_PARAMS_STRUCT.pack(
        baud,
        mode,
        address,
        tx_r30_bit,
        watchdog_ms,
        safe_stop_hold_ms,
        ramp_step,
        control_period_us,
    )


def unpack_ack(payload: bytes) -> Optional[dict]:
    if len(payload) < ACK_STRUCT.size:
        return None
    status, detail, timestamp_us = ACK_STRUCT.unpack(payload[: ACK_STRUCT.size])
    return {"status": status, "detail": detail, "timestamp_us": timestamp_us}
