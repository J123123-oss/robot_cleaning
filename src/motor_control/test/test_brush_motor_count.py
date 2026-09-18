"""Tests for brush motor topology configuration."""

import ast
from pathlib import Path
from typing import Tuple


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "motor_control"
    / "motor_control.py"
)


def _load_brush_motor_ids_function():
    """Load the topology helper without requiring a ROS installation."""
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "brush_motor_ids_for_count"
    )
    namespace = {"Tuple": Tuple}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    return namespace["brush_motor_ids_for_count"]


def test_brush_motor_count_maps_to_configured_node_ids():
    """Zero, one, and two brushes select the expected CANopen nodes."""
    brush_motor_ids_for_count = _load_brush_motor_ids_function()

    assert brush_motor_ids_for_count(0) == ()
    assert brush_motor_ids_for_count(1) == (3,)
    assert brush_motor_ids_for_count(2) == (3, 4)


def test_brush_motor_count_rejects_unsupported_values():
    """The topology contract rejects negative and larger brush counts."""
    brush_motor_ids_for_count = _load_brush_motor_ids_function()

    for value in (-1, 3):
        try:
            brush_motor_ids_for_count(value)
        except ValueError:
            pass
        else:
            raise AssertionError("unsupported brush count was accepted")
