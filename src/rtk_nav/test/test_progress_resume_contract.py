import json
import os
import sys
import tempfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from rtk_nav.progress_store import load_progress, save_progress


def test_progress_store_round_trip_and_required_fields():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "progress.json"
        payload = {
            "version": 1,
            "route_id": "route_a",
            "route_path": "/routes/route_a.txt",
            "waypoint_idx": 7,
            "area": "E4-E5",
            "nav_state": "WAYPOINT_MOVE",
            "target_waypoint": [120.0, 30.0, 90.0],
            "updated_at": "2026-11-12T00:00:00Z",
        }
        save_progress(path, payload)
        assert load_progress(path) == payload


def test_progress_store_rejects_malformed_or_incomplete_snapshot():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "progress.json"
        path.write_text("{broken", encoding="utf-8")
        assert load_progress(path) is None
        path.write_text(json.dumps({"route_id": "route_a"}), encoding="utf-8")
        assert load_progress(path) is None


def test_rtk_node_contains_resume_and_persistence_hooks():
    source = Path("src/rtk_nav/rtk_nav/rtk_nav.py").read_text(encoding="utf-8")
    assert "progress_store" in source
    assert "/rtk/resume_progress" in source
    assert "def resume_progress_callback" in source
    assert "def _persist_progress" in source
    assert "def _clear_progress_snapshot" in source
    assert "self.current_waypoint_idx += 1" in source


if __name__ == "__main__":
    pytest.main([os.path.abspath(__file__)])
