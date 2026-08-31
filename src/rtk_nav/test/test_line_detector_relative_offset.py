# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import math
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "line_detector_node.py"


def _helpers():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    names = {
        "nav_state_allows_lateral_output",
        "capture_initial_lateral_offset",
        "calculate_relative_lateral_offset",
    }
    nodes = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "math": math,
        "WAYPOINT_MOVE_STATE": "WAYPOINT_MOVE",
    }
    exec(
        compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE_PATH), "exec"),
        namespace,
    )
    return namespace


def test_waypoint_lateral_offset_uses_first_valid_sample_as_relative_zero():
    helpers = _helpers()
    capture = helpers["capture_initial_lateral_offset"]
    relative = helpers["calculate_relative_lateral_offset"]

    initial = capture("WAYPOINT_MOVE", 0.18, None)
    assert math.isclose(initial, 0.18, abs_tol=1e-9)
    assert math.isclose(
        relative("WAYPOINT_MOVE", 0.18, initial), 0.0, abs_tol=1e-9
    )
    assert math.isclose(
        relative("WAYPOINT_MOVE", 0.24, initial), 0.06, abs_tol=1e-9
    )

    # A later frame must not move the baseline within the same waypoint move.
    assert capture("WAYPOINT_MOVE", 0.24, initial) == initial


def test_waypoint_lateral_offset_baseline_is_recreated_after_state_change():
    helpers = _helpers()
    capture = helpers["capture_initial_lateral_offset"]
    relative = helpers["calculate_relative_lateral_offset"]

    assert capture("WAYPOINT_CALIB", 0.20, None) is None
    assert math.isnan(relative("WAYPOINT_CALIB", 0.20, None))

    initial = capture("WAYPOINT_MOVE", -0.12, None)
    assert math.isclose(initial, -0.12, abs_tol=1e-9)
    assert math.isclose(
        relative("WAYPOINT_MOVE", -0.02, initial), 0.10, abs_tol=1e-9
    )
