# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import math
import unittest
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


class BoundaryPairGeometryTest(unittest.TestCase):
    def _select_pair(self, lines, axis_angle_deg):
        helpers = _helpers()
        self.assertIn(
            "select_boundary_pair",
            helpers,
            "line detector must declare select_boundary_pair",
        )
        pair = helpers["select_boundary_pair"](
            lines,
            axis_angle_deg,
            640,
            480,
            500.0,
        )
        self.assertIsNotNone(pair)
        return pair

    @staticmethod
    def _error(pair):
        return (pair[2] + pair[3]) / 2.0 - pair[4]

    @staticmethod
    def _cardinal_lines(axis_angle_deg, along_axis_offset=0):
        if axis_angle_deg % 180.0 == 0.0:
            start_x = along_axis_offset
            end_x = 480 + along_axis_offset
            return [
                (start_x, 360, end_x, 360, 480.0, 0.0, (start_x + end_x) / 2.0, 360.0),
                (start_x, 120, end_x, 120, 480.0, 0.0, (start_x + end_x) / 2.0, 120.0),
            ]

        start_y = along_axis_offset
        end_y = 480 + along_axis_offset
        return [
            (440, start_y, 440, end_y, 480.0, 90.0, 440.0, (start_y + end_y) / 2.0),
            (200, start_y, 200, end_y, 480.0, 90.0, 200.0, (start_y + end_y) / 2.0),
        ]

    @staticmethod
    def _asymmetric_lines(axis_angle_deg):
        if axis_angle_deg % 180.0 == 0.0:
            return [
                (0, 140, 480, 140, 480.0, 0.0, 240.0, 140.0),
                (0, 340, 480, 340, 480.0, 0.0, 240.0, 340.0),
            ]

        return [
            (200, 0, 200, 480, 480.0, 90.0, 200.0, 240.0),
            (440, 0, 440, 480, 480.0, 90.0, 440.0, 240.0),
        ]

    def test_boundary_pair_translation_along_each_cardinal_path_axis_is_invariant(self):
        for axis_angle_deg in (0.0, 90.0, 180.0, 270.0):
            base = self._select_pair(
                self._cardinal_lines(axis_angle_deg),
                axis_angle_deg,
            )
            shifted = self._select_pair(
                self._cardinal_lines(axis_angle_deg, along_axis_offset=100),
                axis_angle_deg,
            )
            self.assertAlmostEqual(
                self._error(base),
                self._error(shifted),
                delta=1e-9,
                msg=f"axis={axis_angle_deg}",
            )

    def test_boundary_pair_normal_translation_preserves_base_and_signed_errors(self):
        base = self._select_pair(
            [
                (440, 0, 440, 480, 480.0, 90.0, 440.0, 240.0),
                (200, 0, 200, 480, 480.0, 90.0, 200.0, 240.0),
            ],
            90.0,
        )
        shifted_right = self._select_pair(
            self._asymmetric_lines(90.0),
            90.0,
        )
        shifted_left = self._select_pair(
            [
                (410, 0, 410, 480, 480.0, 90.0, 410.0, 240.0),
                (170, 0, 170, 480, 480.0, 90.0, 170.0, 240.0),
            ],
            90.0,
        )

        base_error = self._error(base)
        right_error = self._error(shifted_right)
        left_error = self._error(shifted_left)
        self.assertAlmostEqual(base_error, 0.0, delta=1e-9)
        self.assertAlmostEqual(right_error, -30.0, delta=1e-9)
        self.assertAlmostEqual(left_error, 30.0, delta=1e-9)
        self.assertAlmostEqual(right_error, -left_error, delta=1e-9)
        self.assertLess(right_error, base_error)
        self.assertGreater(left_error, base_error)

    def test_boundary_pair_reverse_cardinal_axes_reverse_lateral_sign(self):
        errors = {}
        for axis_angle_deg in (0.0, 90.0, 180.0, 270.0):
            pair = self._select_pair(
                self._asymmetric_lines(axis_angle_deg),
                axis_angle_deg,
            )
            errors[axis_angle_deg] = self._error(pair)

        self.assertAlmostEqual(errors[0.0], -errors[180.0], delta=1e-9)
        self.assertAlmostEqual(errors[90.0], -errors[270.0], delta=1e-9)
