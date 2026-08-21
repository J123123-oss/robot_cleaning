# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import math
import unittest
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "line_detector_node.py"


def _tree_and_source():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    return ast.parse(source), source


def _top_level_function(tree, name):
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    ) if any(
        isinstance(node, ast.FunctionDef) and node.name == name
        for node in tree.body
    ) else None


def _class_method(tree, class_name, name):
    detector_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node
        for node in detector_class.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _execute_top_level_function(tree, name):
    function = _top_level_function(tree, name)
    if function is None:
        return None
    namespace = {"math": math}
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(SOURCE_PATH),
            "exec",
        ),
        namespace,
    )
    return namespace[name]


class CoarseWhiteLineContractTest(unittest.TestCase):
    def test_source_declares_coarse_line_contract_functions_and_parameters(self):
        tree, source = _tree_and_source()
        top_level_functions = {
            node.name
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
        }

        self.assertIn("select_coarse_white_lines", top_level_functions)
        self.assertIn("merge_nearby_line_records", top_level_functions)
        for identifier in (
            "white_mask",
            "select_coarse_white_lines",
            "merge_nearby_line_records",
            "coarse_line_min_width_px",
            "coarse_line_min_support",
        ):
            self.assertIn(identifier, source)

    def test_select_coarse_white_lines_filters_thin_weak_and_short_records(self):
        tree, _ = _tree_and_source()
        select_lines = _execute_top_level_function(
            tree, "select_coarse_white_lines"
        )
        self.assertIsNotNone(select_lines)
        if select_lines is None:
            return
        lines = [
            (0, 10, 40, 10, 40.0, 0.0, 20.0, 10.0, 12.0, 0.95),
            (0, 20, 140, 20, 140.0, 0.0, 70.0, 20.0, 2.0, 0.95),
            (0, 30, 140, 30, 140.0, 0.0, 70.0, 30.0, 12.0, 0.45),
            (0, 40, 40, 40, 40.0, 0.0, 20.0, 40.0, 12.0, 0.95),
            (0, 50, 140, 50, 140.0, 0.0, 70.0, 50.0, 12.0, 0.95),
        ]

        selected = select_lines(lines, 100.0, 8.0, 0.8)

        self.assertEqual(list(selected), [lines[-1]])

    def test_merge_nearby_line_records_reduces_edges_to_centered_candidate(self):
        tree, _ = _tree_and_source()
        merge_lines = _execute_top_level_function(
            tree, "merge_nearby_line_records"
        )
        self.assertIsNotNone(merge_lines)
        if merge_lines is None:
            return
        lower_edge = (0, 90, 140, 90, 140.0, 0.0, 70.0, 90.0, 12.0, 0.95)
        upper_edge = (0, 110, 140, 110, 140.0, 0.0, 70.0, 110.0, 12.0, 0.95)

        merged = list(merge_lines([lower_edge, upper_edge], 0.0, 30.0))

        self.assertLess(len(merged), 2)
        self.assertEqual(len(merged), 1)
        center_projection = float(merged[0][7])
        self.assertAlmostEqual(center_projection, 100.0, delta=1.5)

    def test_pipeline_uses_white_mask_parallel_angle_and_no_perpendicular_validity_gate(self):
        tree, source = _tree_and_source()
        detector = ast.unparse(
            _class_method(tree, "GridLineDetector", "detect_and_draw_grid_lines")
        )
        normalized_source = source.replace(" ", "")

        self.assertIn("white_mask", detector)
        self.assertTrue(
            any(
                token in detector
                for token in ("cv2.inRange", "COLOR_BGR2HSV", "COLOR_BGR2Lab")
            )
        )
        self.assertIn("weighted_line_angle(parallel_group)", detector)
        self.assertNotIn("len(perpendicular_group)>=self.min_line_count", normalized_source)


if __name__ == "__main__":
    unittest.main()
