import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _method(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    return next(node for node in ast.walk(tree)
                if isinstance(node, ast.FunctionDef) and node.name == name)


def test_navigation_does_not_switch_to_target_bearing_when_projection_exceeds_segment():
    for name in ("move_to_first_waypoint", "multi_waypoint_nav_generator"):
        source = ast.unparse(_method(name))
        assert "t > 1.0" not in source


def test_explicit_force_bearing_mode_still_has_a_target_bearing_path():
    source = ast.unparse(_method("multi_waypoint_nav_generator"))
    assert "if self.nav_context.get('force_bearing_mode')" in source
    assert "path_direction = current_bearing" in source
