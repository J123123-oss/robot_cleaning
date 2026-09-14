# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import math
import unittest
from pathlib import Path
from typing import List, Optional, Tuple


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _source_tree():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    return source, ast.parse(source)


def _function(tree, name):
    return next(
        node for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    )


class RtkQualityLossDebounceTest(unittest.TestCase):

    def test_stable_orientation_float_is_allowed_after_observation_window(self):
        source, tree = _source_tree()
        helper = _function(tree, "is_orientation_float_heading_safe")
        namespace = {
            "List": List,
            "Optional": Optional,
            "Tuple": Tuple,
            "math": math,
            "ORIENTATION_FLOAT_MAX_DEVIATION": 10.0,
            "ORIENTATION_FLOAT_STABILITY_WINDOW": 5.0,
            "ORIENTATION_FLOAT_STABILITY_RANGE": 3.0,
        }
        exec(
            compile(ast.Module(body=[helper], type_ignores=[]), str(SOURCE_PATH), "exec"),
            namespace,
        )
        is_safe = namespace["is_orientation_float_heading_safe"]

        stable_history = [(0.0, 179.0), (2.0, -180.0), (5.0, -179.0)]
        self.assertTrue(is_safe(5.0, -179.0, 179.0, 0.0, stable_history))
        self.assertFalse(is_safe(5.0, 25.0, 10.0, 0.0, stable_history))
        self.assertFalse(is_safe(4.9, 10.0, 10.0, 0.0, stable_history))

    def test_orientation_float_degraded_path_is_separate_from_dual_fixed(self):
        source, tree = _source_tree()

        self.assertIn("ORIENTATION_FLOAT_MAX_DEVIATION = 10.0", source)
        self.assertIn("ORIENTATION_FLOAT_STABILITY_WINDOW = 5.0", source)
        self.assertIn("ORIENTATION_FLOAT_SPEED_SCALE = 0.5", source)

        callback = ast.unparse(_function(tree, "heading_callback"))
        timer = ast.unparse(_function(tree, "rtk_timer_callback"))
        self.assertIn("self._last_dual_fixed_heading_deg", callback)
        self.assertIn("self.rtk_orientation_degraded", callback)
        self.assertIn("not self.rtk_orientation_degraded", callback)
        self.assertIn("ORIENTATION_FLOAT_SPEED_SCALE", timer)

    def test_orientation_float_does_not_auto_gate_when_heading_remains_safe(self):
        _, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))
        self.assertIn("quality_loss_exceeded", callback)
        self.assertIn("not self._auto_heading_gate_pending", callback)
        self.assertIn("not self.rtk_orientation_degraded", callback)

    def test_quality_loss_debounce_boundary(self):
        source, tree = _source_tree()
        self.assertIn("RTK_QUALITY_LOSS_DEBOUNCE = 8.0", source)

        helper = _function(tree, "has_rtk_quality_loss_exceeded")
        namespace = {
            "Optional": Optional,
            "RTK_QUALITY_LOSS_DEBOUNCE": 8.0,
        }
        exec(
            compile(ast.Module(body=[helper], type_ignores=[]), str(SOURCE_PATH), "exec"),
            namespace,
        )
        exceeded = namespace["has_rtk_quality_loss_exceeded"]

        self.assertFalse(exceeded(107.99, 100.0))
        self.assertTrue(exceeded(108.0, 100.0))
        self.assertFalse(exceeded(200.0, None))

    def test_heading_callback_delays_quality_pause_until_debounce_expires(self):
        _, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn("self._rtk_quality_loss_since", callback)
        self.assertIn("has_rtk_quality_loss_exceeded", callback)
        self.assertIn("RTK质量去抖", callback)

    def test_timer_keeps_navigation_running_during_quality_debounce(self):
        source, tree = _source_tree()
        timer = ast.unparse(_function(tree, "rtk_timer_callback"))

        self.assertIn("has_rtk_quality_loss_exceeded", timer)
        self.assertIn("RTK_QUALITY_LOSS_DEBOUNCE = 8.0", source)
        self.assertIn("继续当前导航", timer)


if __name__ == "__main__":
    unittest.main()
