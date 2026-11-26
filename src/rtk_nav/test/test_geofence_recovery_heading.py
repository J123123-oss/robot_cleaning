import ast
import math
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def load_heading_selector():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "select_geofence_recovery_heading"
    )
    scope = {"Tuple": tuple, "math": math}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope["select_geofence_recovery_heading"]


def test_recovery_selects_forward_when_forward_requires_less_rotation():
    select_heading = load_heading_selector()

    heading, direction = select_heading(10.0, -10.0)

    assert math.isclose(heading, -10.0)
    assert direction == "FORWARD"


def test_recovery_selects_backward_when_backward_requires_less_rotation():
    select_heading = load_heading_selector()

    heading, direction = select_heading(170.0, 0.0)

    assert math.isclose(heading, 180.0)
    assert direction == "BACKWARD"


def test_initial_entry_selects_reverse_when_it_reduces_turn():
    select_heading = load_heading_selector()

    heading, direction = select_heading(170.0, 0.0)

    assert math.isclose(heading, 180.0)
    assert direction == "BACKWARD"


def test_backward_stanley_commands_are_reversed_without_swapping_sides():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    function = next(node for node in ast.walk(tree)
                    if isinstance(node, ast.FunctionDef) and node.name == "apply_drive_direction")
    scope = {"Tuple": tuple}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), "exec"), scope)

    assert scope["apply_drive_direction"](10.0, 20.0, "FORWARD") == (10.0, 20.0)
    assert scope["apply_drive_direction"](10.0, 20.0, "BACKWARD") == (-10.0, -20.0)
