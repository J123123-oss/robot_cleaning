"""Tests for total battery current protection."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from motor_control.stall_protection import TotalCurrentMonitor


def test_total_current_requires_five_seconds_above_four_ampere():
    monitor = TotalCurrentMonitor(threshold=4.0, duration=5.0)

    assert monitor.update(4.1, enabled=True, now=0.0) is False
    assert monitor.update(4.5, enabled=True, now=4.9) is False
    assert monitor.update(4.5, enabled=True, now=5.0) is True


def test_current_below_threshold_clears_pending_alarm():
    monitor = TotalCurrentMonitor(threshold=4.0, duration=5.0)

    assert monitor.update(4.1, enabled=True, now=0.0) is False
    assert monitor.update(3.9, enabled=True, now=4.0) is False
    assert monitor.update(4.1, enabled=True, now=5.0) is False


def test_disabled_monitor_does_not_alarm_or_accumulate_time():
    monitor = TotalCurrentMonitor(threshold=4.0, duration=5.0)

    assert monitor.update(5.0, enabled=False, now=0.0) is False
    assert monitor.update(5.0, enabled=True, now=5.0) is False
