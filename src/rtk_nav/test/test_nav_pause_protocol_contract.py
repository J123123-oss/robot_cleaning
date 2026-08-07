# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import unittest
from pathlib import Path


RTK_SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"
MOTOR_SOURCE_PATH = (
    Path(__file__).parents[2] / "motor_control" / "motor_control" / "motor_control.py"
)


def _function(tree, name):
    return next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


class NavPauseProtocolContractTest(unittest.TestCase):

    def test_rtk_state_contains_structured_pause_metadata_and_sequence(self):
        source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        publisher = ast.unparse(_function(tree, "publish_nav_state"))

        self.assertIn("json.dumps", publisher)
        self.assertIn("'nav_state'", publisher)
        self.assertIn("'pause_reason'", publisher)
        self.assertIn("'auto_resume'", publisher)
        self.assertIn("'seq'", publisher)
        self.assertIn("MANUAL_INTERVENTION_PAUSE_REASONS", publisher)
        self.assertIn("self._nav_state_seq += 1", publisher)

    def test_motor_rejects_stale_state_and_keeps_auto_pause_at_zero_speed(self):
        source = MOTOR_SOURCE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        callback = ast.unparse(_function(tree, "rtk_nav_status_callback"))
        timer = ast.unparse(_function(tree, "timer_callback"))

        self.assertIn("json.loads", callback)
        self.assertIn("seq <= self._last_rtk_nav_state_seq", callback)
        self.assertIn("self._rtk_auto_pause = True", callback)
        self.assertIn("self.set_motors_speed(0.0, 0.0)", callback)
        self.assertIn("self.switch_state('h')", callback)
        self.assertIn("if self._rtk_auto_pause", timer)

    def test_tilt_fault_is_not_published_as_an_rtk_error(self):
        rtk_source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
        motor_source = MOTOR_SOURCE_PATH.read_text(encoding="utf-8")

        self.assertNotIn("ERROR_TILT_FAULT", rtk_source)
        self.assertNotIn("ERROR_TILT_FAULT", motor_source)

    def test_skip_to_area_rejoins_from_current_position(self):
        source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        skip = ast.unparse(_function(tree, "_apply_skip_to_area"))

        self.assertIn("current_rejoin_origin", skip)
        self.assertIn("self.current_gps", skip)
        self.assertIn("self.stanley_path_direction = None", skip)
        self.assertIn("self.last_waypoint_cache = (", skip)


if __name__ == "__main__":
    unittest.main()
