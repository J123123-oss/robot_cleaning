from pathlib import Path


def test_motor_control_latches_geofence_fault_and_reports_error_bit():
    source = (
        Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    ).read_text(encoding="utf-8")
    assert "ERROR_GEOFENCE_OUTSIDE = 1024" in source
    assert '"/rtk/geofence_fault_code"' in source
    assert "def geofence_fault_callback" in source
    assert "self.geofence_fault_latched = True" in source
    assert 'self.switch_state("h")' in source
    assert "error |= ERROR_GEOFENCE_OUTSIDE" in source


def test_geofence_fault_bit_follows_existing_bitmask():
    source = (
        Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    ).read_text(encoding="utf-8")
    assert "ERROR_AUTO_CLEANING_LOW_BATTERY = 512" in source
    assert "ERROR_GEOFENCE_OUTSIDE = 1024" in source
