import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _method(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    return next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_waypoint_tracking_reset_clears_force_bearing_history():
    method = _method("_reset_waypoint_tracking")
    source = ast.unparse(method)
    for key in (
        "force_bearing_mode", "force_bearing_target",
        "force_bearing_recalib_count", "force_bearing_min_distance",
        "distance_increase_count",
    ):
        assert key in source


def test_initial_and_normal_waypoint_transitions_call_tracking_reset():
    source = SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    method = _method("multi_waypoint_nav_generator")
    method_source = ast.unparse(method)
    assert method_source.count("self._reset_waypoint_tracking()") >= 2
    assert "self.current_waypoint_idx = 1" in method_source
