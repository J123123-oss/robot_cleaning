from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1]))
from motor_control.rtk_point_recorder import record_rtk_point


def test_record_point_marks_invalid_rtk_status(tmp_path):
    path = tmp_path / "rtk_points.md"
    assert record_rtk_point(path, 110.1, 35.2, 4, 5, "2026-11-02 12:00:00")
    text = path.read_text(encoding="utf-8")
    assert "seq: 1" in text and "valid: false" in text
    assert "point: {lon: 110.1, lat: 35.2}" in text


def test_next_sequence(tmp_path):
    path = tmp_path / "rtk_points.md"
    record_rtk_point(path, 1, 2, 4, 4, "t1")
    record_rtk_point(path, 3, 4, 4, 4, "t2")
    assert path.read_text(encoding="utf-8").count("- seq:") == 2
    assert "seq: 2" in path.read_text(encoding="utf-8")


def test_invalid_coordinates_are_rejected(tmp_path):
    path = tmp_path / "rtk_points.md"
    assert not record_rtk_point(path, 181, 2, 4, 4)
    assert not path.exists()
