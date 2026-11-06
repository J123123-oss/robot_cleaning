"""Contract tests for publishing per-motor current in robot state."""

from pathlib import Path


SOURCE = Path(__file__).parents[1] / "motor_control" / "motor_control.py"


def test_robot_state_payload_contains_per_motor_current():
    source = SOURCE.read_text(encoding="utf-8")
    assert '"motor_current": build_motor_fault_details(' in source

