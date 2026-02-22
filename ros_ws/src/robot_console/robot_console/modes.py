import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional


class RobotMode(str, Enum):
    MANUAL = "manual"
    COMMISSIONING = "commissioning"
    MAPPING = "mapping"
    NAV = "nav"
    PICKPLACE = "pickplace"
    TRAINING = "training"
    RELIABILITY = "reliability"
    DEMO = "demo"


def normalize_mode(mode_name: str) -> RobotMode:
    value = (mode_name or "").strip().lower()
    for mode in RobotMode:
        if mode.value == value:
            return mode
    raise ValueError(f"Unknown mode: {mode_name}")


@dataclass
class SafetySnapshot:
    armed: bool
    estop_latched: bool
    watchdog_ok: bool
    command_timeout_sec: float
    last_command_age_sec: Optional[float]
    last_disarm_reason: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "armed": self.armed,
            "estop_latched": self.estop_latched,
            "watchdog_ok": self.watchdog_ok,
            "command_timeout_sec": self.command_timeout_sec,
            "last_command_age_sec": self.last_command_age_sec,
            "last_disarm_reason": self.last_disarm_reason,
        }


class ModeManager:
    """Tracks operator mode and safety state for motion gating."""

    def __init__(self, command_timeout_sec: float = 0.8) -> None:
        self._lock = threading.Lock()
        self._mode = RobotMode.MANUAL
        self._armed = False
        self._estop_latched = False
        self._watchdog_ok = True
        self._command_timeout_sec = max(0.2, float(command_timeout_sec))
        self._last_motion_command_mono: Optional[float] = None
        self._last_disarm_reason = "startup"
        self._last_transition_unix = time.time()

    def set_mode(self, mode_name: str) -> RobotMode:
        mode = normalize_mode(mode_name)
        with self._lock:
            self._mode = mode
            self._last_transition_unix = time.time()
            if mode != RobotMode.MANUAL:
                self._armed = False
                self._last_disarm_reason = "mode_change"
        return mode

    def current_mode(self) -> RobotMode:
        with self._lock:
            return self._mode

    def arm(self) -> bool:
        with self._lock:
            if self._estop_latched:
                return False
            self._armed = True
            self._watchdog_ok = True
            self._last_motion_command_mono = time.monotonic()
            self._last_transition_unix = time.time()
            return True

    def disarm(self, reason: str = "operator") -> None:
        with self._lock:
            self._armed = False
            self._last_disarm_reason = reason
            self._last_transition_unix = time.time()

    def estop(self) -> None:
        with self._lock:
            self._armed = False
            self._estop_latched = True
            self._watchdog_ok = False
            self._last_disarm_reason = "estop"
            self._last_transition_unix = time.time()

    def reset_estop(self) -> None:
        with self._lock:
            self._estop_latched = False
            self._watchdog_ok = True
            self._last_disarm_reason = "estop_reset"

    def record_motion_command(self) -> None:
        with self._lock:
            self._last_motion_command_mono = time.monotonic()
            self._watchdog_ok = True

    def set_watchdog_timeout(self, timeout_sec: float) -> None:
        with self._lock:
            self._command_timeout_sec = max(0.2, float(timeout_sec))

    def command_timeout_sec(self) -> float:
        with self._lock:
            return self._command_timeout_sec

    def should_allow_motion(self) -> bool:
        with self._lock:
            return self._armed and not self._estop_latched

    def check_watchdog(self) -> bool:
        """Returns True when motion command timeout forced disarm."""
        with self._lock:
            if not self._armed or self._last_motion_command_mono is None:
                return False
            age = time.monotonic() - self._last_motion_command_mono
            if age <= self._command_timeout_sec:
                return False

            self._armed = False
            self._watchdog_ok = False
            self._last_disarm_reason = "watchdog_timeout"
            self._last_transition_unix = time.time()
            return True

    def safety_snapshot(self) -> SafetySnapshot:
        with self._lock:
            last_age: Optional[float] = None
            if self._last_motion_command_mono is not None:
                last_age = max(0.0, time.monotonic() - self._last_motion_command_mono)
            return SafetySnapshot(
                armed=self._armed,
                estop_latched=self._estop_latched,
                watchdog_ok=self._watchdog_ok,
                command_timeout_sec=self._command_timeout_sec,
                last_command_age_sec=last_age,
                last_disarm_reason=self._last_disarm_reason,
            )

    def status(self) -> Dict[str, Any]:
        snap = self.safety_snapshot()
        with self._lock:
            mode = self._mode.value
            transition_time = self._last_transition_unix

        data = snap.to_dict()
        data.update(
            {
                "mode": mode,
                "last_transition_unix": transition_time,
            }
        )
        return data
