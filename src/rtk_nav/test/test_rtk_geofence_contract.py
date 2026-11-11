from pathlib import Path


def test_rtk_nav_has_sidecar_and_latched_fence_contract():
    source = (Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    assert "def load_geofence_sidecar" in source
    assert "def enforce_geofence" in source
    assert "self.trip_geofence_stop(\"outside_allowed_region\")" in source
    assert "if self.enforce_geofence():" in source
    assert "if not self.rtk_solution_ready or not self.current_gps:" in source
    assert 'pause_reason == "geofence_stop"' in source
