"""UIO transport for PRU shared-memory telemetry and motor commands."""

from __future__ import annotations

import ctypes
import mmap
import os
from pathlib import Path
import struct
import threading
import time
from typing import Any

from uio_layout import (
    LayoutValidationError,
    UIO_HOST_STATUS_ARMED,
    UIO_HOST_STATUS_ESTOP,
    UIO_HOST_STATUS_LAYOUT_OK,
    UIO_LAYOUT_MAGIC,
    UIO_LAYOUT_VERSION,
    UIO_PRU0_STATUS_ENCODER_ACTIVE,
    UIO_PRU0_STATUS_READY,
    UIO_PRU1_STATUS_MOTOR_ACTIVE,
    UIO_PRU1_STATUS_READY,
    UIO_SHARED_LAYOUT_SIZE,
    UIOSharedLayout,
    validate_layout_header,
)

_SYS_UIO_ROOT = Path("/sys/class/uio")
_DEV_ROOT = Path("/dev")

_CTRL_SOFT_RST_N_BIT = 1 << 0
_CTRL_ENABLE_BIT = 1 << 1


class UIOTransportError(RuntimeError):
    """Raised on transport setup failures."""


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip()


def _read_int(path: Path) -> int:
    return int(_read_text(path), 0)


def discover_uio_devices() -> dict[str, Path]:
    mapping: dict[str, Path] = {}
    for uio_sys in sorted(_SYS_UIO_ROOT.glob("uio*")):
        name_file = uio_sys / "name"
        if not name_file.exists():
            continue
        name = _read_text(name_file)
        dev_path = _DEV_ROOT / uio_sys.name
        mapping[name] = dev_path
    return mapping


def _resolve_uio_device_path(name: str) -> Path:
    devices = discover_uio_devices()
    if name not in devices:
        known = ", ".join(sorted(devices.keys())) or "<none>"
        raise UIOTransportError(f"UIO device '{name}' not found (known names: {known})")
    return devices[name]


def open_uio_by_name(name: str) -> int:
    """Open /dev/uio* by sysfs /name string."""
    dev_path = _resolve_uio_device_path(name)
    return os.open(dev_path, os.O_RDWR | os.O_SYNC)


class UIOTransport:
    def __init__(self, config: dict[str, Any], *, dry_run: bool, logger: Any) -> None:
        self._cfg = config
        self._dry_run = dry_run
        self._logger = logger

        self._fd: int | None = None
        self._mmap: mmap.mmap | None = None
        self._layout: UIOSharedLayout | None = None

        self._uio_name: str | None = None
        self._uio_dev_path: Path | None = None
        self._map_index = int(self._cfg.get("map_index", 0))
        self._map_size = 0

        self._heartbeat_timeout_ms = int(self._cfg.get("heartbeat_timeout_ms", 1000))
        self._startup_timeout_ms = int(self._cfg.get("startup_timeout_ms", 2500))

        self._last_pru0_hb = 0
        self._last_pru1_hb = 0
        self._last_pru0_seen = 0.0
        self._last_pru1_seen = 0.0

        self._last_error = ""
        self._lock = threading.Lock()

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def available(self) -> bool:
        return self._layout is not None

    def _fail(self, message: str) -> None:
        self._last_error = message
        self._logger.error(message)

    def _uio_sysfs(self) -> Path:
        if self._uio_dev_path is None:
            raise UIOTransportError("uio device path unavailable")
        return _SYS_UIO_ROOT / self._uio_dev_path.name

    def _get_map_entry(self, key: str) -> str:
        return _read_text(self._uio_sysfs() / "maps" / f"map{self._map_index}" / key)

    def _initialize_layout(self) -> None:
        if self._layout is None:
            raise UIOTransportError("layout not mapped")

        # Host-side sanity write/read to detect broken map/offset configuration.
        struct_addr = ctypes.addressof(self._layout)
        ctypes.memset(struct_addr, 0, UIO_SHARED_LAYOUT_SIZE)
        self._layout.magic = UIO_LAYOUT_MAGIC
        self._layout.version = UIO_LAYOUT_VERSION
        self._layout.size = UIO_SHARED_LAYOUT_SIZE
        self._layout.host_status_bits = UIO_HOST_STATUS_LAYOUT_OK | UIO_HOST_STATUS_ESTOP
        self._layout.estop_asserted = 1
        self._layout.armed = 0

        try:
            validate_layout_header(self._layout)
        except LayoutValidationError as exc:
            raise UIOTransportError(f"layout sanity check failed: {exc}") from exc

    def _read_u32(self, offset: int) -> int:
        if self._mmap is None:
            raise UIOTransportError("memory not mapped")
        return struct.unpack_from("<I", self._mmap, offset)[0]

    def _write_u32(self, offset: int, value: int) -> None:
        if self._mmap is None:
            raise UIOTransportError("memory not mapped")
        struct.pack_into("<I", self._mmap, offset, value & 0xFFFFFFFF)

    def _stop_pru(self, ctrl_offset: int) -> None:
        self._write_u32(ctrl_offset, 0)
        time.sleep(0.002)

    def _start_pru(self, ctrl_offset: int) -> None:
        self._write_u32(ctrl_offset, _CTRL_SOFT_RST_N_BIT)
        self._write_u32(ctrl_offset, _CTRL_SOFT_RST_N_BIT | _CTRL_ENABLE_BIT)

    def _resolve_fw_path(self, value: str) -> Path:
        fw_path = Path(value)
        if fw_path.is_absolute():
            return fw_path
        return (Path(__file__).resolve().parent / fw_path).resolve()

    def _load_pru_binary(self, fw_bin: Path, iram_offset: int, iram_size: int) -> None:
        if self._mmap is None:
            raise UIOTransportError("memory not mapped")
        if not fw_bin.exists():
            raise UIOTransportError(f"firmware bin missing: {fw_bin}")

        payload = fw_bin.read_bytes()
        if len(payload) > iram_size:
            raise UIOTransportError(
                f"firmware {fw_bin} ({len(payload)} bytes) exceeds IRAM size ({iram_size} bytes)"
            )

        self._mmap[iram_offset : iram_offset + iram_size] = b"\x00" * iram_size
        self._mmap[iram_offset : iram_offset + len(payload)] = payload

    def _load_and_start_prus(self) -> None:
        if self._mmap is None:
            raise UIOTransportError("memory not mapped")

        pru0_ctrl = int(self._cfg.get("pru0_ctrl_offset", 0x22000))
        pru1_ctrl = int(self._cfg.get("pru1_ctrl_offset", 0x24000))
        pru0_iram = int(self._cfg.get("pru0_iram_offset", 0x34000))
        pru1_iram = int(self._cfg.get("pru1_iram_offset", 0x38000))
        pru_iram_size = int(self._cfg.get("pru_iram_size", 0x2000))

        pru0_fw = self._resolve_fw_path(str(self._cfg.get("pru0_fw_bin", "../pru_fw/pru0_encoders/am335x-pru0-fw.bin")))
        pru1_fw = self._resolve_fw_path(str(self._cfg.get("pru1_fw_bin", "../pru_fw/pru1_sabertooth/am335x-pru1-fw.bin")))

        self._logger.info("uio_pru_loader stop+load start")
        self._stop_pru(pru0_ctrl)
        self._stop_pru(pru1_ctrl)

        self._load_pru_binary(pru0_fw, pru0_iram, pru_iram_size)
        self._load_pru_binary(pru1_fw, pru1_iram, pru_iram_size)

        self._start_pru(pru0_ctrl)
        self._start_pru(pru1_ctrl)
        self._logger.info("uio_pru_loader started pru0=%s pru1=%s", pru0_fw, pru1_fw)

    def _wait_for_heartbeat(self) -> None:
        if self._layout is None:
            return

        start_pru0 = int(self._layout.pru0_heartbeat)
        start_pru1 = int(self._layout.pru1_heartbeat)

        deadline = time.monotonic() + (self._startup_timeout_ms / 1000.0)
        saw_pru0 = False
        saw_pru1 = False

        while time.monotonic() < deadline:
            now_pru0 = int(self._layout.pru0_heartbeat)
            now_pru1 = int(self._layout.pru1_heartbeat)
            if now_pru0 != start_pru0:
                saw_pru0 = True
            if now_pru1 != start_pru1:
                saw_pru1 = True
            if saw_pru0 and saw_pru1:
                break
            time.sleep(0.02)

        if not saw_pru0:
            self._logger.warning("uio heartbeat did not advance for PRU0")
        if not saw_pru1:
            self._logger.warning("uio heartbeat did not advance for PRU1")

    def open(self) -> None:
        if self._dry_run:
            self._logger.info("uio_open skipped (dry-run)")
            return

        with self._lock:
            expected = [str(name) for name in self._cfg.get("expected_names", [])]
            discovered = discover_uio_devices()
            missing = [name for name in expected if name not in discovered]
            if missing:
                self._fail(
                    "missing expected UIO device names: "
                    + ", ".join(missing)
                    + " (configure pru.uio.expected_names)"
                )
                return

            mem_uio_name = str(self._cfg.get("mem_uio_name") or (expected[0] if expected else ""))
            if not mem_uio_name:
                if not discovered:
                    self._fail("no UIO devices discovered under /sys/class/uio")
                    return
                mem_uio_name = sorted(discovered.keys())[0]

            try:
                self._fd = open_uio_by_name(mem_uio_name)
            except Exception as exc:
                self._fail(f"failed to open UIO '{mem_uio_name}': {exc}")
                return

            self._uio_name = mem_uio_name
            self._uio_dev_path = discovered[mem_uio_name]

            try:
                self._map_size = _read_int(self._uio_sysfs() / "maps" / f"map{self._map_index}" / "size")
            except Exception as exc:
                self._fail(f"unable to read map{self._map_index} size for {mem_uio_name}: {exc}")
                self.close()
                return

            map_name = ""
            try:
                map_name = self._get_map_entry("name")
            except Exception:
                pass

            expected_map_name = str(self._cfg.get("map_name", "")).strip()
            if expected_map_name and map_name and map_name != expected_map_name:
                self._logger.warning(
                    "uio map name mismatch expected=%s got=%s", expected_map_name, map_name
                )

            layout_offset = int(self._cfg.get("shared_offset", 0x10000))
            if layout_offset < 0 or (layout_offset + UIO_SHARED_LAYOUT_SIZE) > self._map_size:
                self._fail(
                    f"shared layout out of map range (offset={layout_offset}, size={UIO_SHARED_LAYOUT_SIZE}, map_size={self._map_size})"
                )
                self.close()
                return

            try:
                page_size = os.sysconf("SC_PAGE_SIZE")
                self._mmap = mmap.mmap(
                    self._fd,
                    self._map_size,
                    flags=mmap.MAP_SHARED,
                    prot=mmap.PROT_READ | mmap.PROT_WRITE,
                    offset=page_size * self._map_index,
                )
                self._layout = UIOSharedLayout.from_buffer(self._mmap, layout_offset)
                self._initialize_layout()

                if bool(self._cfg.get("auto_start_prus", True)):
                    self._load_and_start_prus()
                self._wait_for_heartbeat()

                self._last_pru0_hb = int(self._layout.pru0_heartbeat)
                self._last_pru1_hb = int(self._layout.pru1_heartbeat)
                now = time.monotonic()
                self._last_pru0_seen = now
                self._last_pru1_seen = now
                self._last_error = ""
                self._logger.info(
                    "uio_open ok name=%s dev=%s map=%d map_name=%s map_size=%d",
                    self._uio_name,
                    self._uio_dev_path,
                    self._map_index,
                    map_name or "<unknown>",
                    self._map_size,
                )
            except Exception as exc:
                self._fail(f"uio transport init failed: {exc}")
                self.close()

    def close(self) -> None:
        with self._lock:
            self._layout = None
            if self._mmap is not None:
                self._mmap.close()
                self._mmap = None
            if self._fd is not None:
                os.close(self._fd)
                self._fd = None

    def _monotonic_us(self) -> int:
        return int(time.monotonic() * 1_000_000) & 0xFFFFFFFF

    def _update_host_header(self, *, armed: bool | None = None, estop: bool | None = None) -> None:
        if self._layout is None:
            return

        layout = self._layout
        layout.host_heartbeat = (int(layout.host_heartbeat) + 1) & 0xFFFFFFFF
        layout.host_time_us = self._monotonic_us()

        bits = int(layout.host_status_bits) | UIO_HOST_STATUS_LAYOUT_OK
        if armed is None:
            armed = bool(layout.armed)
        if estop is None:
            estop = bool(layout.estop_asserted)

        if armed:
            bits |= UIO_HOST_STATUS_ARMED
        else:
            bits &= ~UIO_HOST_STATUS_ARMED

        if estop:
            bits |= UIO_HOST_STATUS_ESTOP
        else:
            bits &= ~UIO_HOST_STATUS_ESTOP

        layout.host_status_bits = bits

    def configure_encoder(self, *, sample_period_us: int, velocity_interval_ms: int, stream_hz: int) -> bool:
        if self._dry_run:
            return True
        with self._lock:
            if self._layout is None:
                return False
            self._update_host_header()
            self._layout.encoder_sample_period_us = max(5, int(sample_period_us))
            self._layout.encoder_velocity_interval_us = max(1000, int(velocity_interval_ms) * 1000)
            self._layout.encoder_stream_hz = max(0, int(stream_hz))
            return True

    def configure_motor(
        self,
        *,
        baud: int,
        mode: int,
        address: int,
        tx_r30_bit: int,
        watchdog_ms: int,
        safe_stop_hold_ms: int,
        ramp_step: int,
        control_period_us: int,
    ) -> bool:
        if self._dry_run:
            return True
        with self._lock:
            if self._layout is None:
                return False
            self._update_host_header()
            self._layout.sabertooth_baud = max(1200, int(baud))
            self._layout.sabertooth_mode = int(mode) & 0xFF
            self._layout.sabertooth_address = int(address) & 0xFF
            self._layout.sabertooth_tx_bit = int(tx_r30_bit) & 0x1F
            self._layout.motor_watchdog_ms = max(1, int(watchdog_ms))
            self._layout.motor_safe_stop_ms = max(1, int(safe_stop_hold_ms))
            self._layout.motor_ramp_step = max(0, int(ramp_step))
            self._layout.motor_control_period_us = max(1000, int(control_period_us))
            return True

    def write_motor_command(self, left: int, right: int, *, armed: bool, estop: bool) -> bool:
        if self._dry_run:
            return True
        with self._lock:
            if self._layout is None:
                return False

            self._update_host_header(armed=armed, estop=estop)
            self._layout.armed = 1 if armed else 0
            self._layout.estop_asserted = 1 if estop else 0

            if estop or not armed:
                left = 0
                right = 0

            self._layout.motor_command_left = int(left)
            self._layout.motor_command_right = int(right)
            self._layout.motor_command_seq = (int(self._layout.motor_command_seq) + 1) & 0xFFFFFFFF
            self._layout.motor_command_time_us = self._monotonic_us()
            return True

    def reset_encoders(self) -> bool:
        if self._dry_run:
            return True
        with self._lock:
            if self._layout is None:
                return False
            self._update_host_header()
            self._layout.encoder_reset_token = (int(self._layout.encoder_reset_token) + 1) & 0xFFFFFFFF
            return True

    def read_encoder_snapshot(self) -> dict[str, Any] | None:
        if self._dry_run:
            return {
                "counts": [0, 0, 0, 0],
                "velocity_tps": [0, 0, 0, 0],
                "timestamp_us": 0,
                "pru0_online": False,
                "pru1_online": False,
                "pru0_status_bits": 0,
                "pru1_status_bits": 0,
            }

        with self._lock:
            if self._layout is None:
                return None

            self._update_host_header()
            now = time.monotonic()

            pru0_hb = int(self._layout.pru0_heartbeat)
            pru1_hb = int(self._layout.pru1_heartbeat)
            if pru0_hb != self._last_pru0_hb:
                self._last_pru0_hb = pru0_hb
                self._last_pru0_seen = now
            if pru1_hb != self._last_pru1_hb:
                self._last_pru1_hb = pru1_hb
                self._last_pru1_seen = now

            timeout_s = max(0.1, self._heartbeat_timeout_ms / 1000.0)
            pru0_status = int(self._layout.pru0_status_bits)
            pru1_status = int(self._layout.pru1_status_bits)

            pru0_online = (
                (now - self._last_pru0_seen) <= timeout_s
                and (pru0_status & UIO_PRU0_STATUS_READY) != 0
                and (pru0_status & UIO_PRU0_STATUS_ENCODER_ACTIVE) != 0
            )
            pru1_online = (
                (now - self._last_pru1_seen) <= timeout_s
                and (pru1_status & UIO_PRU1_STATUS_READY) != 0
            )

            return {
                "counts": [int(v) for v in self._layout.encoder_counts],
                "velocity_tps": [int(v) for v in self._layout.encoder_velocity_tps],
                "timestamp_us": int(self._layout.encoder_timestamp_us),
                "pru0_online": pru0_online,
                "pru1_online": pru1_online,
                "pru0_status_bits": pru0_status,
                "pru1_status_bits": pru1_status,
                "motor_applied_left": int(self._layout.motor_applied_left),
                "motor_applied_right": int(self._layout.motor_applied_right),
            }
