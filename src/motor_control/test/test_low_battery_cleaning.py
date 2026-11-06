"""Regression tests for the in-task low-battery protection contract."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from motor_control.motor_control import (
    ERROR_AUTO_CLEANING_LOW_BATTERY,
    should_pause_auto_cleaning_for_low_battery,
)


def test_auto_cleaning_low_battery_triggers_once_at_or_below_threshold():
    assert should_pause_auto_cleaning_for_low_battery(
        "AUTO_CLEANING", 20.0, 20.0, False
    ) is True
    assert should_pause_auto_cleaning_for_low_battery(
        "AUTO_CLEANING", 19.9, 20.0, True
    ) is False


def test_low_battery_protection_does_not_change_precleaning_start_contract():
    assert should_pause_auto_cleaning_for_low_battery(
        "NORMAL", 10.0, 20.0, False
    ) is False


def test_in_task_low_battery_has_a_distinct_error_bit():
    assert ERROR_AUTO_CLEANING_LOW_BATTERY > 256
