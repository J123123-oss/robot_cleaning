import importlib.util
import sys
import types
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "rtk_nav" / "generate_common_entry_area.py"


@pytest.fixture
def generator(monkeypatch):
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)
    sys.path.insert(0, str(PACKAGE_ROOT / "rtk_nav"))
    try:
        spec = importlib.util.spec_from_file_location("generate_common_entry_area", SCRIPT_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.path.remove(str(PACKAGE_ROOT / "rtk_nav"))


def test_generator_matches_full_planner_entry_geometry(generator):
    from geofence_geometry import point_allowed_by_payload, segment_allowed_by_payload

    config_path = PACKAGE_ROOT / "rtk_nav" / "config" / "full.yaml"
    payload = generator.generate_payload(config_path)

    assert payload["utm_zone"] == [49, "S"]
    assert len(payload["areas"]) == 16
    assert len(payload["bridges"]) == 33

    start = (542612.5893860583, 3948936.050272386)
    first_waypoint = (542611.8793754438, 3948930.567575089)
    assert point_allowed_by_payload(start, payload)
    assert point_allowed_by_payload(first_waypoint, payload)
    assert segment_allowed_by_payload(start, first_waypoint, payload)


def test_generator_cli_writes_requested_output(generator, tmp_path):
    config_path = PACKAGE_ROOT / "rtk_nav" / "config" / "full.yaml"
    output_path = tmp_path / "entry.fence.json"

    assert generator.main(["--config", str(config_path), "--output", str(output_path)]) == 0
    generated = generator.load_geofence_payload(str(output_path))
    assert generated["utm_zone"] == [49, "S"]
    assert len(generated["areas"]) == 16


def test_ensure_generated_creates_missing_output(generator, tmp_path, monkeypatch):
    config_path = tmp_path / "full.yaml"
    output_path = tmp_path / "common_entry_area.fence.json"
    config_path.write_text("areas: []\n", encoding="utf-8")
    calls = []

    monkeypatch.setattr(generator, "generate_payload", lambda path: {"areas": []})
    monkeypatch.setattr(
        generator,
        "write_payload",
        lambda payload, path: (
            calls.append((payload, Path(path))),
            Path(path).write_text("generated", encoding="utf-8"),
        )[-1],
    )

    assert generator.ensure_generated(config_path, output_path) is True
    assert calls == [({"areas": []}, output_path)]


def test_ensure_generated_refreshes_when_config_is_newer(
    generator, tmp_path, monkeypatch
):
    config_path = tmp_path / "full.yaml"
    output_path = tmp_path / "common_entry_area.fence.json"
    config_path.write_text("areas: []\n", encoding="utf-8")
    output_path.write_text("old", encoding="utf-8")
    old_time = 1_000
    new_time = 2_000
    generator.os.utime(config_path, (new_time, new_time))
    generator.os.utime(output_path, (old_time, old_time))
    calls = []

    monkeypatch.setattr(generator, "generate_payload", lambda path: {"areas": []})
    monkeypatch.setattr(
        generator,
        "write_payload",
        lambda payload, path: calls.append((payload, Path(path))),
    )

    assert generator.ensure_generated(config_path, output_path) is True
    assert calls == [({"areas": []}, output_path)]


def test_ensure_generated_skips_when_output_is_newer(
    generator, tmp_path, monkeypatch
):
    config_path = tmp_path / "full.yaml"
    output_path = tmp_path / "common_entry_area.fence.json"
    config_path.write_text("areas: []\n", encoding="utf-8")
    output_path.write_text("current", encoding="utf-8")
    old_time = 1_000
    new_time = 2_000
    generator.os.utime(config_path, (old_time, old_time))
    generator.os.utime(output_path, (new_time, new_time))
    calls = []

    monkeypatch.setattr(
        generator,
        "generate_payload",
        lambda path: calls.append(path),
    )

    assert generator.ensure_generated(config_path, output_path) is False
    assert calls == []


def test_ensure_generated_keeps_existing_output_when_generation_fails(
    generator, tmp_path, monkeypatch
):
    config_path = tmp_path / "full.yaml"
    output_path = tmp_path / "common_entry_area.fence.json"
    config_path.write_text("areas: []\n", encoding="utf-8")
    output_path.write_text("old", encoding="utf-8")
    generator.os.utime(config_path, (2_000, 2_000))
    generator.os.utime(output_path, (1_000, 1_000))

    def fail_generation(path):
        raise ValueError("invalid full.yaml")

    monkeypatch.setattr(generator, "generate_payload", fail_generation)

    with pytest.raises(ValueError, match="invalid full.yaml"):
        generator.ensure_generated(config_path, output_path)
    assert output_path.read_text(encoding="utf-8") == "old"


def test_ensure_generated_ignores_missing_config(generator, tmp_path, monkeypatch):
    config_path = tmp_path / "missing-full.yaml"
    output_path = tmp_path / "common_entry_area.fence.json"
    calls = []

    monkeypatch.setattr(generator, "generate_payload", lambda path: calls.append(path))

    assert generator.ensure_generated(config_path, output_path) is False
    assert calls == []
