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


def _calls_named(node, name):
    return [
        child
        for child in ast.walk(node)
        if isinstance(child, ast.Call)
        and (
            (isinstance(child.func, ast.Name) and child.func.id == name)
            or (isinstance(child.func, ast.Attribute) and child.func.attr == name)
        )
    ]


def _call_contains_string(call, value):
    return any(
        isinstance(child, ast.Constant) and child.value == value
        for child in ast.walk(call)
    )


def _publisher_calls(function):
    return {
        ast.unparse(node.func)
        for node in ast.walk(function)
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "publish"
        )
    }


def _has_assignment(function, target_text, value):
    for node in ast.walk(function):
        if isinstance(node, ast.Assign):
            targets = node.targets
            assigned_value = node.value
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
            assigned_value = node.value
        else:
            continue
        if not isinstance(assigned_value, ast.Constant) or assigned_value.value != value:
            continue
        if any(ast.unparse(target) == target_text for target in targets):
            return True
    return False


class PathAwareBoundaryContractTest(unittest.TestCase):
    def test_detector_subscribes_to_path_context_and_declares_geometry_tuning(self):
        tree = _source_tree()
        initializer = _function(tree, "__init__")
        self.assertIsNotNone(initializer)
        initializer_source = ast.unparse(initializer)

        subscriptions = _calls_named(initializer, "create_subscription")
        self.assertTrue(subscriptions)
        self.assertTrue(
            any(
                _call_contains_string(call, "/rtk/visual_path_context")
                for call in subscriptions
            )
        )
        self.assertIn("create_subscription", initializer_source)
        declared = _declared_parameter_names(initializer)
        for name in (
            "line_angle_tolerance_deg",
            "path_context_timeout_sec",
            "boundary_pair_max_gap_px",
            "reacquire_frames",
        ):
            self.assertIn(name, declared)

    def test_image_callback_gates_invalid_path_context_before_buffering(self):
        tree = _source_tree()
        image_callback = _function(tree, "image_callback")
        self.assertIsNotNone(image_callback)

        detect_calls = _calls_named(image_callback, "detect_and_draw_grid_lines")
        self.assertFalse(detect_calls)
        gated_invalid_paths = []
        for node in ast.walk(image_callback):
            if not isinstance(node, ast.If):
                continue
            condition = ast.unparse(node.test)
            body_has_publish_invalid = bool(_calls_named(node, "publish_invalid"))
            if (
                "path_context_valid" in condition
                and "timeout" in condition
                and body_has_publish_invalid
            ):
                gated_invalid_paths.append(node.lineno)

        self.assertTrue(
            gated_invalid_paths,
            "image_callback must publish an invalid result before buffering an invalid frame",
        )

    def test_timer_callback_decodes_and_detects_the_buffered_frame(self):
        tree = _source_tree()
        timer_callback = _function(tree, "timer_callback")
        self.assertIsNotNone(timer_callback)
        self.assertTrue(_calls_named(timer_callback, "detect_and_draw_grid_lines"))

    def test_publish_invalid_resets_result_and_confidence(self):
        tree = _source_tree()
        publish_invalid = _function(tree, "publish_invalid")
        self.assertIsNotNone(publish_invalid)
        self.assertTrue(_has_assignment(publish_invalid, "result.z", 0.0))
        self.assertTrue(_has_assignment(publish_invalid, "confidence.data", 0.0))
        publisher_calls = _publisher_calls(publish_invalid)
        self.assertIn("self.angle_pub.publish", publisher_calls)
        self.assertIn("self.confidence_pub.publish", publisher_calls)

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
