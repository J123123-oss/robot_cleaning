# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import unittest
from pathlib import Path
from typing import Optional


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

    def test_quality_loss_debounce_boundary(self):
        source, tree = _source_tree()
        self.assertIn("RTK_QUALITY_LOSS_DEBOUNCE = 5.0", source)

        helper = _function(tree, "has_rtk_quality_loss_exceeded")
        namespace = {
            "Optional": Optional,
            "RTK_QUALITY_LOSS_DEBOUNCE": 5.0,
        }
        exec(
            compile(ast.Module(body=[helper], type_ignores=[]), str(SOURCE_PATH), "exec"),
            namespace,
        )
        exceeded = namespace["has_rtk_quality_loss_exceeded"]

        self.assertFalse(exceeded(104.99, 100.0))
        self.assertTrue(exceeded(105.0, 100.0))
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
        self.assertIn("RTK_QUALITY_LOSS_DEBOUNCE = 5.0", source)
        self.assertIn("继续当前导航", timer)


if __name__ == "__main__":
    unittest.main()
