# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import unittest
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "line_detector_node.py"


def _function(tree, name):
    return next(
        (
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == name
        ),
        None,
    )


def _source_tree():
    return ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))


def _declared_parameter_names(initializer):
    names = []
    for node in ast.walk(initializer):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != "declare_parameter" or not node.args:
            continue
        if isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
            names.append(node.args[0].value)
    return names


class PathAwareBoundaryContractTest(unittest.TestCase):
    def test_detector_subscribes_to_path_context_and_declares_geometry_tuning(self):
        tree = _source_tree()
        initializer = _function(tree, "__init__")
        self.assertIsNotNone(initializer)
        initializer_source = ast.unparse(initializer)

        self.assertIn("/rtk/visual_path_context", initializer_source)
        declared = _declared_parameter_names(initializer)
        for name in (
            "line_angle_tolerance_deg",
            "path_context_timeout_sec",
            "boundary_pair_max_gap_px",
            "reacquire_frames",
        ):
            self.assertIn(name, declared)

    def test_image_callback_gates_invalid_path_context_before_detection(self):
        tree = _source_tree()
        image_callback = _function(tree, "image_callback")
        self.assertIsNotNone(image_callback)

        gated_invalid_paths = []
        for node in ast.walk(image_callback):
            if not isinstance(node, ast.If):
                continue
            condition = ast.unparse(node.test)
            body = ast.unparse(ast.Module(body=node.body, type_ignores=[]))
            if (
                "path_context" in condition
                and ("valid" in condition or "timeout" in condition)
                and "publish_invalid" in body
            ):
                gated_invalid_paths.append(node.lineno)

        self.assertTrue(
            gated_invalid_paths,
            "image_callback must publish an invalid result when path context is invalid or stale",
        )

    def test_detector_exposes_angle_and_boundary_helper_contracts(self):
        tree = _source_tree()
        angle_helper = _function(tree, "undirected_angle")
        distance_helper = _function(tree, "undirected_angle_distance")
        boundary_helper = _function(tree, "select_boundary_pair")

        self.assertIsNotNone(angle_helper)
        self.assertIsNotNone(distance_helper)
        self.assertIsNotNone(boundary_helper)

        self.assertEqual(len(angle_helper.args.args), 1)
        self.assertEqual(len(distance_helper.args.args), 2)
        self.assertEqual(
            [arg.arg for arg in boundary_helper.args.args],
            ["lines", "axis_angle_deg", "width", "height", "max_gap_px"],
        )


if __name__ == "__main__":
    unittest.main()
