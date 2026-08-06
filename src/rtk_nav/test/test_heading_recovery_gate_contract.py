# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import unittest
from pathlib import Path


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


class HeadingRecoveryGateContractTest(unittest.TestCase):

    def test_stability_parameters_require_30_second_convergence(self):
        source, _ = _source_tree()

        self.assertIn("HEADING_STABILITY_SETTLE_WINDOW = 30.0", source)
        self.assertIn("HEADING_STABILITY_SETTLE_RANGE = 1.0", source)
        self.assertIn("AUTO_HEADING_GATE_TIMEOUT = 180.0", source)

    def test_heading_gate_requires_short_and_settle_windows(self):
        _, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn("tracking_active = self._auto_heading_gate_pending", callback)
        self.assertIn("HEADING_STABILITY_SETTLE_WINDOW", callback)
        self.assertIn("short_range <= HEADING_STABILITY_RANGE", callback)
        self.assertIn("settle_range <= HEADING_STABILITY_SETTLE_RANGE", callback)

    def test_quality_loss_resets_sampling_epoch_and_timeout_start(self):
        _, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn("self._auto_heading_gate_start_time = None", callback)
        self.assertIn("self._auto_heading_gate_start_time = now", callback)

    def test_gate_release_aligns_waypoint_move_before_generator_creation(self):
        source, tree = _source_tree()
        self.assertIn("def _start_auto_heading_gate_path_alignment", source)

        timer = ast.unparse(_function(tree, "rtk_timer_callback"))
        helper = ast.unparse(_function(tree, "_start_auto_heading_gate_path_alignment"))
        alignment_call = timer.index("_start_auto_heading_gate_path_alignment")
        generator_create = timer.index("self.multi_waypoint_generator = self.multi_waypoint_nav_generator")

        self.assertLess(alignment_call, generator_create)
        self.assertIn("self.start_heading_recalibration", helper)
        self.assertIn("self.nav_context['nav_state'] = NavState.WAYPOINT_CALIB", helper)


if __name__ == "__main__":
    unittest.main()
