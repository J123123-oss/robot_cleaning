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
        "wrap180",
        "undirected_angle",
        "undirected_angle_distance",
        "select_boundary_pair",
    }
    nodes = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {"math": math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE_PATH), "exec"), namespace)
    return namespace


def test_path_angles_use_undirected_180_degree_geometry():
    helpers = _helpers()
    assert helpers["wrap180"](190.0) == -170.0
    assert helpers["wrap180"](-190.0) == 170.0
    assert helpers["undirected_angle_distance"](0.0, 180.0) == 0.0
    assert helpers["undirected_angle_distance"](0.0, 90.0) == 90.0


def test_boundary_pair_center_is_zero_when_boundaries_are_symmetric():
    helpers = _helpers()
    # Record layout: x1, y1, x2, y2, length, angle, center_x, center_y.
    left = (440, 0, 440, 480, 480.0, 90.0, 440.0, 240.0)
    right = (200, 0, 200, 480, 480.0, 90.0, 200.0, 240.0)
    pair = helpers["select_boundary_pair"]([left, right], 90.0, 640, 480, 500.0)
    assert pair is not None
    _, _, left_projection, right_projection, center_projection = pair
    assert math.isclose(
        (left_projection + right_projection) / 2.0 - center_projection,
        0.0,
        abs_tol=1e-9,
    )


def test_boundary_pair_is_invariant_to_translation_along_path_axis():
    helpers = _helpers()
    lines = [
        (440, 0, 440, 480, 480.0, 90.0, 440.0, 240.0),
        (200, 0, 200, 480, 480.0, 90.0, 200.0, 240.0),
    ]
    shifted = [
        (440, 100, 440, 580, 480.0, 90.0, 440.0, 340.0),
        (200, 100, 200, 580, 480.0, 90.0, 200.0, 340.0),
    ]
    first = helpers["select_boundary_pair"](lines, 90.0, 640, 480, 500.0)
    second = helpers["select_boundary_pair"](shifted, 90.0, 640, 480, 500.0)
    assert first is not None and second is not None
    first_error = (first[2] + first[3]) / 2.0 - first[4]
    second_error = (second[2] + second[3]) / 2.0 - second[4]
    assert math.isclose(first_error, second_error, abs_tol=1e-9)


def test_boundary_pair_changes_with_path_normal_translation():
    helpers = _helpers()
    lines = [
        (470, 0, 470, 480, 480.0, 90.0, 470.0, 240.0),
        (230, 0, 230, 480, 480.0, 90.0, 230.0, 240.0),
    ]
    pair = helpers["select_boundary_pair"](lines, 90.0, 640, 480, 500.0)
    assert pair is not None
    error = (pair[2] + pair[3]) / 2.0 - pair[4]
    assert math.isclose(error, -30.0, abs_tol=1e-9)


def test_reversing_directed_axis_reverses_lateral_sign_basis():
    helpers = _helpers()
    left = (430, 0, 430, 480, 480.0, 90.0, 430.0, 240.0)
    right = (200, 0, 200, 480, 480.0, 90.0, 200.0, 240.0)
    forward = helpers["select_boundary_pair"]([left, right], 90.0, 640, 480, 500.0)
    reverse = helpers["select_boundary_pair"]([left, right], -90.0, 640, 480, 500.0)
    assert forward is not None and reverse is not None
    forward_error = (forward[2] + forward[3]) / 2.0 - forward[4]
    reverse_error = (reverse[2] + reverse[3]) / 2.0 - reverse[4]
    assert math.isclose(forward_error, -reverse_error, abs_tol=1e-9)
