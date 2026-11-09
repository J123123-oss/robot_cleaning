import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from motor_control.loading_timing import advance_active_elapsed


def test_loading_timeout_pauses_during_stops_and_recovery():
    elapsed = advance_active_elapsed(8.0, 3.0, was_active=True)
    elapsed = advance_active_elapsed(elapsed, 5.0, was_active=False)
    elapsed = advance_active_elapsed(elapsed, 4.0, was_active=True)

    assert elapsed == 15.0


def test_loading_timeout_ignores_negative_clock_interval():
    assert advance_active_elapsed(8.0, -1.0, was_active=True) == 8.0
