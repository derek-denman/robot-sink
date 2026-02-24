"""Hardware access helpers for the BBB host daemon."""

from __future__ import annotations

import logging
from pathlib import Path


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
