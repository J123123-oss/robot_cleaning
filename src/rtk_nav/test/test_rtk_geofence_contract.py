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
    assert "self.current_waypoint_idx == 0" in source
    assert "def move_to_first_waypoint" in source


def test_missing_sidecar_is_checked_before_initial_approach_allowance():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    enforce_source = source[source.index("def enforce_geofence"):source.index("def trip_geofence_stop")]
    missing_sidecar_guard = enforce_source.index("if self.geofence_payload is None:")
    assert "self.check_geofence_position()" in enforce_source


def test_geofence_recovery_readiness_is_published_for_manual_resume():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    assert '"/rtk/geofence_recovery_ready"' in source
    assert "def _update_geofence_recovery_ready" in source
    assert "self.publish_geofence_recovery_ready" in source


def test_initial_move_does_not_bypass_geofence_position_check():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    enforce_source = source[source.index("def enforce_geofence"):source.index("def trip_geofence_stop")]
    assert "Allow the dedicated" not in enforce_source
    assert "self.check_geofence_position()" in enforce_source


def test_route_sidecar_adds_loading_bridges_for_route_endpoints():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    assert "def _append_loading_bridges_to_geofence" in source
    assert "loading_bridge" in source
    assert "build_polyline_corridor" in source
