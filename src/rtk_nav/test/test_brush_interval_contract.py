# Copyright 2026

import ast
import unittest
from pathlib import Path


RTK_SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _function(tree, name):
    return next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


class BrushIntervalContractTest(unittest.TestCase):

    def test_skip_into_active_interval_replays_crossed_start(self):
        """A jump to 145 must apply start 13 and leave stop 484 pending."""
        starts = [13]
        stops = [484]
        active = False
        idx = 145

        while starts or stops:
            next_start = starts[0] if starts else float("inf")
            next_stop = stops[0] if stops else float("inf")
            if min(next_start, next_stop) > idx:
                break
            if next_start <= next_stop:
                starts.pop(0)
                active = True
            else:
                stops.pop(0)
                active = False

        self.assertTrue(active)
        self.assertEqual(starts, [])
        self.assertEqual(stops, [484])

    def test_brush_control_replays_events_in_index_order(self):
        source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        function = ast.unparse(_function(tree, "check_and_control_brush"))

        self.assertIn("next_start", function)
        self.assertIn("next_stop", function)
        self.assertIn("min(next_start, next_stop)", function)
        self.assertIn("self.brush_start_indices.pop(0)", function)
        self.assertIn("self.brush_stop_indices.pop(0)", function)
        self.assertNotIn("stale", function)

    def test_skip_to_area_persists_computed_brush_state_for_auto_resume(self):
        source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        function = ast.unparse(_function(tree, "_apply_skip_to_area"))

        self.assertIn("self.check_and_control_brush()", function)
        self.assertIn("self.nav_context['brush_active'] = self.brush_active", function)


if __name__ == "__main__":
    unittest.main()
