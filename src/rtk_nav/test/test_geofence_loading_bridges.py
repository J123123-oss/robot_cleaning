import ast
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace


PACKAGE = Path(__file__).parents[1] / "rtk_nav"
sys.path.insert(0, str(PACKAGE))
from geofence_geometry import build_polyline_corridor, load_geofence_payload


def test_loading_valid_fence_adds_both_endpoint_bridges(tmp_path):
    tree = ast.parse((PACKAGE / "rtk_nav.py").read_text(encoding="utf-8"))
    names = {"load_geofence_sidecar", "_append_loading_bridges_to_geofence", "latlon_to_utm"}
    functions = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name in names]
    scope = {"Tuple": tuple, "math": math, "json": json,
             "load_geofence_payload": load_geofence_payload,
             "build_polyline_corridor": build_polyline_corridor}
    exec(compile(ast.Module(body=functions, type_ignores=[]), "rtk_nav.py", "exec"), scope)
    fence = tmp_path / "route.fence.json"
    fence.write_text(json.dumps({"utm_zone": [49, "S"], "expansion_radius_m": 0.2,
                                 "areas": [], "bridges": []}), encoding="utf-8")
    logger = SimpleNamespace(info=lambda message: None, error=lambda message: None)
    node = SimpleNamespace(enable_geofence=True, waypoints=[(111.0, 35.0, 0), (111.0001, 35.0, 0)],
                           loading_waypoint=(110.9999, 35.0, 0),
                           publish_geofence_recovery_ready=lambda ready: None,
                           get_logger=lambda: logger)
    node.latlon_to_utm = lambda lat, lon: scope["latlon_to_utm"](node, lat, lon)
    node._append_loading_bridges_to_geofence = lambda: scope["_append_loading_bridges_to_geofence"](node)
    assert scope["load_geofence_sidecar"](node, str(fence)) is True
    bridges = node.geofence_payload["bridges"]
    assert [bridge["name"] for bridge in bridges] == ["loading_to_route_start", "route_end_to_loading"]
    assert all(len(bridge["corridors_utm"]) > 0 for bridge in bridges)
    assert bridges[0]["centerline_utm"][0] == bridges[1]["centerline_utm"][1]
    assert bridges[0]["centerline_utm"][0] != bridges[0]["centerline_utm"][1]


def test_navigation_projection_matches_planner_fence_coordinates():
    from geofence_geometry import point_allowed_by_payload

    tree = ast.parse((PACKAGE / "rtk_nav.py").read_text(encoding="utf-8"))
    function = next(node for node in ast.walk(tree)
                    if isinstance(node, ast.FunctionDef) and node.name == "latlon_to_utm")
    scope = {"Tuple": tuple, "math": math}
    exec(compile(ast.Module(body=[function], type_ignores=[]), "rtk_nav.py", "exec"), scope)
    position = scope["latlon_to_utm"](None, 35.68301253, 111.470872336)
    assert abs(position[0] - 542607.9428977) < 0.02
    assert abs(position[1] - 3948892.9659979) < 0.02
    fence = {"expansion_radius_m": 0.2, "bridges": [{"centerline_utm": [
        [542608.5456426, 3948892.9675035], [542606.1426276, 3948893.2459861]
    ]}]}
    assert point_allowed_by_payload(position, fence)
    assert not point_allowed_by_payload((position[0], position[1] - 1.0), fence)
