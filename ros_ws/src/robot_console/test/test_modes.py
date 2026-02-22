import time

from robot_console.modes import ModeManager


def test_mode_manager_starts_disarmed() -> None:
    mgr = ModeManager(command_timeout_sec=0.2)
    assert not mgr.should_allow_motion()


def test_watchdog_disarms_when_commands_stop() -> None:
    mgr = ModeManager(command_timeout_sec=0.2)
    assert mgr.arm()
    mgr.record_motion_command()
    time.sleep(0.3)
    assert mgr.check_watchdog()
    assert not mgr.should_allow_motion()
