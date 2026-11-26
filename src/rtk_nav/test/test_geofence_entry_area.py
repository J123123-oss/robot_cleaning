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
    scope = {"os": SimpleNamespace(path=SimpleNamespace(isfile=lambda path: True)),
             "load_geofence_payload": lambda path: {"utm_zone": [49, "S"], "areas": [rectangle(0, 10, 0, 10)]},
             "validate_convex_polygon": validate_convex_polygon}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), scope)
    node = SimpleNamespace(geofence_entry_area_path="common_entry_area.fence.json", geofence_path="route.fence.json",
                           geofence_payload={"utm_zone": [49, "S"], "areas": []},
                           get_logger=Mock())
    scope["_load_common_entry_area"](node)
    assert len(node.geofence_entry_payload["areas"]) == 1
    assert node.geofence_payload["areas"] == []
    node.geofence_payload["utm_zone"] = [50, "S"]
    with pytest.raises(ValueError):
        scope["_load_common_entry_area"](node)
