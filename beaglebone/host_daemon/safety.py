"""Safety-state helpers shared by the daemon API and control loops."""

from __future__ import annotations

from dataclasses import dataclass
import time


@dataclass
class SafetySnapshot:
    armed: bool
    estop_asserted: bool
    watchdog_tripped: bool
    estop_reason: str


class SafetyController:
    def __init__(self, *, command_timeout_ms: int, safe_stop_hold_ms: int) -> None:
        self.command_timeout_s = max(0.01, command_timeout_ms / 1000.0)
        self.safe_stop_hold_s = max(0.0, safe_stop_hold_ms / 1000.0)

        self._armed = False
        self._estop_asserted = True
        self._watchdog_tripped = False
        self._estop_reason = "boot_default"

        now = time.monotonic()
        self._hold_until = now
        self._last_command_time = now

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def estop_asserted(self) -> bool:
        return self._estop_asserted

    @property
    def watchdog_tripped(self) -> bool:
        return self._watchdog_tripped

    @property
    def estop_reason(self) -> str:
        return self._estop_reason

    @property
    def last_command_time(self) -> float:
        return self._last_command_time

    def snapshot(self) -> SafetySnapshot:
        return SafetySnapshot(
            armed=self._armed,
            estop_asserted=self._estop_asserted,
            watchdog_tripped=self._watchdog_tripped,
            estop_reason=self._estop_reason,
        )

    def register_command(self, now: float) -> None:
        self._last_command_time = now

    def arm(self, now: float) -> bool:
        if now < self._hold_until:
            return False

        self._armed = True
        self._estop_asserted = False
        self._watchdog_tripped = False
        self._estop_reason = ""
        self._last_command_time = now
        return True

    def trip_estop(self, now: float, reason: str) -> None:
        self._armed = False
        self._estop_asserted = True
        self._watchdog_tripped = reason == "command_timeout"
        self._estop_reason = reason
        self._hold_until = now + self.safe_stop_hold_s

    def check_command_timeout(self, now: float) -> bool:
        if not self._armed:
            return False
        if now - self._last_command_time <= self.command_timeout_s:
            return False

        self.trip_estop(now, "command_timeout")
        return True
