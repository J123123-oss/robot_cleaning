import ast
import math
from pathlib import Path
from typing import Tuple


RTK_SOURCE_PATH = (
    Path(__file__).resolve().parents[1] / "rtk_nav" / "rtk_nav.py"
)


def _function(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    raise AssertionError(f"function {name!r} not found")


def _load_helper():
    tree = ast.parse(RTK_SOURCE_PATH.read_text(encoding="utf-8"))
    helper = _function(tree, "combine_stanley_and_visual_correction")
    namespace = {"math": math, "Tuple": Tuple}
    exec(
        compile(ast.Module(body=[helper], type_ignores=[]), str(RTK_SOURCE_PATH), "exec"),
        namespace,
    )
    return namespace["combine_stanley_and_visual_correction"]


def test_full_speed_keeps_the_configured_total_correction_limit():
    combine = _load_helper()

    stanley, visual, total = combine(12.0, 1.2, 1.0, 1.5)

    assert math.isclose(stanley, 0.4, abs_tol=1e-9)
    assert math.isclose(visual, 1.2, abs_tol=1e-9)
    assert math.isclose(total, 1.5, abs_tol=1e-9)


def test_reduced_speed_scales_both_corrections_and_their_combined_limit():
    combine = _load_helper()

    stanley, visual, total = combine(45.0, 1.5, 0.35, 1.5)

    assert math.isclose(stanley, 0.525, abs_tol=1e-9)
    assert math.isclose(visual, 0.525, abs_tol=1e-9)
    assert math.isclose(total, 0.525, abs_tol=1e-9)


def test_invalid_speed_scale_falls_back_to_a_bounded_value():
    combine = _load_helper()

    _, _, total = combine(45.0, 1.5, float("nan"), 1.5)

    assert math.isclose(total, 1.5, abs_tol=1e-9)
