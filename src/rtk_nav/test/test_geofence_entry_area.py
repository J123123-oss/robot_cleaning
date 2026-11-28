import ast
import os
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

PACKAGE = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE))
from rtk_nav.geofence_geometry import (
    load_geofence_payload,
    point_allowed_by_payload,
    segment_allowed_by_payload,
    validate_convex_polygon,
)


def rectangle(left, right, bottom, top):
    return {"boundary_utm": [[left, bottom], [right, bottom],
                             [right, top], [left, top]]}


@pytest.mark.parametrize("start", [(0, 0), (9, 9), (9, 0), (0, 9)])
def test_different_start_positions_reach_first_point(start):
    payload = {"areas": [rectangle(0, 10, 0, 10)], "bridges": []}
    assert segment_allowed_by_payload(start, (5, 5), payload)


def test_approach_rejects_small_uncovered_gap():
    payload = {"areas": [rectangle(0, 5, 0, 2), rectangle(5.01, 10, 0, 2)]}
    assert not segment_allowed_by_payload((1, 1), (9, 1), payload)


def test_approach_covers_bridge_rounded_endpoints():
    payload = {"areas": [], "bridges": [{"centerline_utm": [[0, 0], [2, 0]]}],
               "expansion_radius_m": 0.2}
    assert segment_allowed_by_payload((-0.1, 0), (2.1, 0), payload)


@pytest.mark.parametrize("polygon", [
    [[0, 0], [1, 0], [2, 0]],
    [[0, 0], [2, 0], [1, 1], [2, 2], [0, 2]],
    [[0, 0], [2, 2], [0, 2], [2, 0]],
    [[0, 0], [float("nan"), 1], [0, 2]],
])
def test_rejects_invalid_public_polygons(polygon):
    with pytest.raises(ValueError):
        validate_convex_polygon(polygon)


def test_public_area_is_used_only_during_entry():
    source = PACKAGE / "rtk_nav" / "rtk_nav.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    method = next(item for item in ast.walk(tree) if isinstance(item, ast.FunctionDef)
                  and item.name == "check_geofence_position")
    scope = {"math": __import__("math"), "point_allowed_by_payload": point_allowed_by_payload}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), scope)
    node = SimpleNamespace(geofence_payload={"areas": [rectangle(0, 1, 0, 1)]},
                           current_gps=(5, 5), latlon_to_utm=lambda lat, lon: (lon, lat),
                           geofence_entry_payload={"areas": [rectangle(0, 10, 0, 10)]},
                           geofence_entry_pending=True)
    assert scope["check_geofence_position"](node)
    node.geofence_entry_pending = False
    assert not scope["check_geofence_position"](node)


def test_shared_area_loads_and_rejects_zone_mismatch():
    source = PACKAGE / "rtk_nav" / "rtk_nav.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    method = next(item for item in ast.walk(tree) if isinstance(item, ast.FunctionDef)
                  and item.name == "_load_common_entry_area")
    ensure_calls = []
    scope = {"os": SimpleNamespace(path=SimpleNamespace(isfile=lambda path: True)),
             "load_geofence_payload": lambda path: {"utm_zone": [49, "S"], "areas": [rectangle(0, 10, 0, 10)]},
             "validate_convex_polygon": validate_convex_polygon,
             "ensure_generated": lambda **kwargs: ensure_calls.append(kwargs)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), scope)
    node = SimpleNamespace(geofence_entry_area_path="common_entry_area.fence.json", geofence_path="route.fence.json",
                           geofence_payload={"utm_zone": [49, "S"], "areas": []},
                           get_logger=Mock())
    scope["_load_common_entry_area"](node)
    assert len(node.geofence_entry_payload["areas"]) == 1
    assert node.geofence_payload["areas"] == []
    assert ensure_calls == []
    node.geofence_payload["utm_zone"] = [50, "S"]
    with pytest.raises(ValueError):
        scope["_load_common_entry_area"](node)


def test_default_shared_area_loading_refreshes_generated_file(tmp_path):
    source = PACKAGE / "rtk_nav" / "rtk_nav.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    method = next(item for item in ast.walk(tree) if isinstance(item, ast.FunctionDef)
                  and item.name == "_load_common_entry_area")
    route_path = tmp_path / "route.fence.json"
    entry_path = tmp_path / "common_entry_area.fence.json"
    entry_path.write_text("generated", encoding="utf-8")
    ensure_calls = []
    scope = {
        "os": os,
        "load_geofence_payload": lambda path: {
            "utm_zone": [49, "S"],
            "areas": [rectangle(0, 10, 0, 10)],
        },
        "validate_convex_polygon": validate_convex_polygon,
        "ensure_generated": lambda **kwargs: ensure_calls.append(kwargs),
    }
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), scope)
    node = SimpleNamespace(
        geofence_entry_area_path="",
        geofence_path=str(route_path),
        geofence_payload={"utm_zone": [49, "S"], "areas": []},
        get_logger=Mock(),
    )

    scope["_load_common_entry_area"](node)

    assert ensure_calls == [{"output_path": str(entry_path)}]
    assert len(node.geofence_entry_payload["areas"]) == 1


def test_load_geofence_sidecar_rejects_generated_area_failure():
    source = PACKAGE / "rtk_nav" / "rtk_nav.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    method = next(item for item in ast.walk(tree) if isinstance(item, ast.FunctionDef)
                  and item.name == "load_geofence_sidecar")
    scope = {"load_geofence_payload": lambda path: {"utm_zone": [49, "S"], "areas": []},
             "json": __import__("json")}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), scope)
    node = SimpleNamespace(
        enable_geofence=True,
        geofence_payload={"areas": []},
        geofence_entry_payload={"areas": []},
        geofence_entry_pending=False,
        geofence_outside_count=3,
        geofence_latched=True,
        publish_geofence_recovery_ready=Mock(),
        _load_common_entry_area=Mock(side_effect=ValueError("full.yaml 无效")),
        _append_loading_bridges_to_geofence=Mock(),
        get_logger=Mock(),
    )

    assert scope["load_geofence_sidecar"](node, "route.fence.json") is False
    assert node.geofence_payload is None
    node.get_logger.return_value.error.assert_called_once()


def test_initial_approach_rejection_logs_coverage_diagnostics():
    source = (PACKAGE / "rtk_nav" / "rtk_nav.py").read_text(encoding="utf-8")
    assert "[电子围栏] 首点接入拒绝:" in source
    assert "起点路线围栏覆盖" in source
    assert "起点公共接入区域覆盖" in source
    assert "直线全程覆盖" in source


def test_full_config_entry_fence_covers_reported_bridge2_approach():
    from rtk_nav.geofence_geometry import point_allowed_by_payload

    fence_path = PACKAGE / "rtk_nav" / "cleaning_path" / "common_entry_area.fence.json"
    payload = load_geofence_payload(str(fence_path))
    assert payload["utm_zone"] == [49, "S"]

    start = (542612.5893860583, 3948936.050272386)
    first_waypoint = (542611.8793754438, 3948930.567575089)
    assert point_allowed_by_payload(start, payload)
    assert point_allowed_by_payload(first_waypoint, payload)
    assert segment_allowed_by_payload(start, first_waypoint, payload)
