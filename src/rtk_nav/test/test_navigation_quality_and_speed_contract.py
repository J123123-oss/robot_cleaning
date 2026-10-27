import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _source():
    return SOURCE.read_text(encoding="utf-8")


def test_install_offset_and_orientation_float_pause_policy_are_explicit():
    source = _source()
    assert "self.rtk_install_offset = 109.43" in source
    tree = ast.parse(source)
    heading = next(node for node in ast.walk(tree)
                   if isinstance(node, ast.FunctionDef) and node.name == "heading_callback")
    heading_source = ast.unparse(heading)
    assert "orientation_quality_lost" in heading_source
    assert "not self.rtk_orientation_degraded" not in heading_source.split("quality_loss_debouncing", 1)[1]


def test_lateral_error_growth_has_a_speed_reduction_guard():
    source = _source()
    assert "def _speed_for_lateral_error_growth" in source
    assert "lateral_error_growth_count" in source
    assert "LINEAR_SPEED_BASE * 0.5" in source
