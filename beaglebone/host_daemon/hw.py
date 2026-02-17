"""Hardware access wrappers for rpmsg endpoints and the S2 safety GPIO."""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import List


class RPMsgEndpoint:
    def __init__(self, device: str, *, dry_run: bool, logger: logging.Logger) -> None:
        self.device = device
        self.dry_run = dry_run
        self.logger = logger
        self.fd: int | None = None

    def open(self) -> None:
        if self.dry_run:
            self.logger.info("rpmsg_open skipped (dry-run) device=%s", self.device)
            return

        try:
            self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
            self.logger.info("rpmsg_open ok device=%s fd=%d", self.device, self.fd)
        except OSError as exc:
            self.fd = None
            self.logger.warning("rpmsg_open failed device=%s err=%s", self.device, exc)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    @property
    def available(self) -> bool:
        return self.fd is not None

    def send(self, payload: bytes) -> bool:
        if self.dry_run:
            return True
        if self.fd is None:
            return False

        try:
            os.write(self.fd, payload)
            return True
        except OSError as exc:
            self.logger.warning("rpmsg_send failed device=%s err=%s", self.device, exc)
            return False

    def recv_all(self, read_size: int = 512) -> List[bytes]:
        if self.dry_run or self.fd is None:
            return []

        messages: List[bytes] = []
        while True:
            try:
                chunk = os.read(self.fd, read_size)
            except BlockingIOError:
                break
            except OSError as exc:
                self.logger.warning("rpmsg_recv failed device=%s err=%s", self.device, exc)
                break

            if not chunk:
                break

            messages.append(chunk)

        return messages


class GPIOStopLine:
    def __init__(
        self,
        gpio_number: int,
        *,
        active_high: bool,
        dry_run: bool,
        logger: logging.Logger,
    ) -> None:
        self.gpio_number = gpio_number
        self.active_high = active_high
        self.dry_run = dry_run
        self.logger = logger
        self._value_state = True
        self._initialized = False

    @property
    def gpio_path(self) -> Path:
        return Path("/sys/class/gpio") / f"gpio{self.gpio_number}"

    def _write(self, path: Path, value: str) -> None:
        path.write_text(value)

    def setup(self) -> None:
        if self.dry_run:
            self._initialized = True
            self._value_state = True
            self.logger.info("gpio_setup skipped (dry-run) gpio=%d", self.gpio_number)
            return

        gpio_root = Path("/sys/class/gpio")
        gpio_dir = self.gpio_path

        try:
            if not gpio_dir.exists():
                self._write(gpio_root / "export", str(self.gpio_number))
            self._write(gpio_dir / "direction", "out")
            self._initialized = True
            self.logger.info("gpio_setup ok gpio=%d", self.gpio_number)
        except OSError as exc:
            self._initialized = False
            self.logger.warning("gpio_setup failed gpio=%d err=%s", self.gpio_number, exc)

    def _set(self, asserted: bool) -> None:
        self._value_state = asserted
        if self.dry_run:
            return
        if not self._initialized:
            return

        value = "1" if (asserted == self.active_high) else "0"
        try:
            self._write(self.gpio_path / "value", value)
        except OSError as exc:
            self.logger.warning(
                "gpio_write failed gpio=%d asserted=%s err=%s",
                self.gpio_number,
                asserted,
                exc,
            )

    def assert_stop(self) -> None:
        self._set(True)

    def release_stop(self) -> None:
        self._set(False)

    @property
    def asserted(self) -> bool:
        return self._value_state
