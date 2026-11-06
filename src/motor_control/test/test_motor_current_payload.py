"""Contract tests for publishing compact per-motor current data."""

from pathlib import Path


SOURCE = Path(__file__).parents[1] / "motor_control" / "motor_control.py"


def test_robot_state_payload_contains_compact_motor_current():
    source = SOURCE.read_text(encoding="utf-8")
    assert '"motor_current": build_motor_current_payload(' in source

