from pathlib import Path


def test_rtk_nav_has_sidecar_and_latched_fence_contract():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    assert "def load_geofence_sidecar" in source
    assert "def enforce_geofence" in source
    assert "self.trip_geofence_stop(\"outside_allowed_region\")" in source
    assert "if self.enforce_geofence():" in source
    assert "if not self.rtk_solution_ready or not self.current_gps:" in source
    assert 'pause_reason == "geofence_stop"' in source


def test_rtk_geofence_stop_publishes_hold_and_next_fault_bit_once():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    assert "ERROR_GEOFENCE_OUTSIDE = 1024" in source
    assert '"/keyboard/control"' in source
    assert '"/rtk/geofence_fault_code"' in source
    assert 'hold_msg.data = "h"' in source
    assert "fault_msg.data = ERROR_GEOFENCE_OUTSIDE" in source
    assert "already_latched" in source


def test_geofence_allows_initial_approach_to_first_waypoint():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    assert 'nav_state in (NavState.IDLE, NavState.INITIAL_MOVE)' in source
    assert "self.current_waypoint_idx == 0" in source
    assert "Allow the dedicated" in source


def test_initial_approach_is_checked_before_missing_sidecar_lock():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    enforce_source = source[source.index("def enforce_geofence"):source.index("def trip_geofence_stop")]
    initial_approach_guard = enforce_source.index("# The robot may start outside the work area.")
    missing_sidecar_guard = enforce_source.index("if self.geofence_payload is None:")
    assert initial_approach_guard < missing_sidecar_guard
