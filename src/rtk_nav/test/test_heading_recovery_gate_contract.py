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
        self.assertIn("AUTO_HEADING_GATE_TIMEOUT = 600.0", source)

    def test_heading_gate_requires_short_and_settle_windows(self):
        _, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn("tracking_active = self._auto_heading_gate_pending", callback)
        self.assertIn("HEADING_STABILITY_SETTLE_WINDOW", callback)
        self.assertIn("short_range <= HEADING_STABILITY_RANGE", callback)
        self.assertIn("settle_range <= HEADING_STABILITY_SETTLE_RANGE", callback)
        self.assertIn("settle_window_dur >= HEADING_STABILITY_SETTLE_WINDOW", callback)

    def test_initial_cleaning_gate_requires_expected_heading_range(self):
        source, tree = _source_tree()
        helper = ast.unparse(_function(tree, "_prepare_auto_cleaning_heading_gate"))
        timer = ast.unparse(_function(tree, "rtk_timer_callback"))

        self.assertIn("INITIAL_HEADING_MIN = -100.0", source)
        self.assertIn("INITIAL_HEADING_MAX = -80.0", source)
        self.assertIn("_auto_heading_gate_initial_direction_required", helper)
        self.assertIn("current_state == NavState.IDLE", helper)
        self.assertIn("not preserve_heading_history", helper)
        callback = ast.unparse(_function(tree, "heading_callback"))
        self.assertIn("INITIAL_HEADING_MIN", callback)
        self.assertIn("INITIAL_HEADING_MAX", callback)
        self.assertIn("_heading_stability_history.clear()", callback)

    def test_quality_loss_resets_sampling_epoch_and_timeout_start(self):
        _, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn("self._auto_heading_gate_start_time = None", callback)
        self.assertIn("self._auto_heading_gate_start_time = now", callback)

    def test_short_quality_gap_bridges_but_long_gap_restarts_qualification(self):
        source, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))
        helper = ast.unparse(_function(tree, "_prepare_auto_cleaning_heading_gate"))

        self.assertIn("HEADING_QUALITY_GAP_MAX = 3.0", source)
        self.assertIn("HEADING_FIXED_CONFIRM_WINDOW = 1.0", source)
        self.assertIn("gap_elapsed > HEADING_QUALITY_GAP_MAX", callback)
        self.assertIn("self._heading_stability_history.clear()", callback)
        self.assertIn("self._auto_heading_gate_start_time = now", callback)
        self.assertIn("preserve_heading_history", helper)
        self.assertIn("force=True", callback)

    def test_float_frames_bridge_without_polluting_stability_window(self):
        _, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn("can_track_heading = current_sample_fixed", callback)
        self.assertIn("self._heading_stability_history.append", callback)

    def test_settle_window_keeps_boundary_margin_and_uses_recent_samples(self):
        source, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn(
            "HEADING_STABILITY_HISTORY_RETENTION = HEADING_STABILITY_SETTLE_WINDOW + 1.0",
            source,
        )
        self.assertIn("settle_history =", callback)
        self.assertIn("len(settle_history) >= 20", callback)

    def test_fixed_recovery_has_confirmation_delay(self):
        _, tree = _source_tree()
        timer = ast.unparse(_function(tree, "rtk_timer_callback"))

        self.assertIn("HEADING_FIXED_CONFIRM_WINDOW", timer)
        self.assertIn("self.publish_stop_speed()", timer)

    def test_heading_gate_is_visible_as_auto_resume_pause(self):
        _, tree = _source_tree()
        helper = ast.unparse(_function(tree, "_prepare_auto_cleaning_heading_gate"))
        timer = ast.unparse(_function(tree, "rtk_timer_callback"))
        recovery = ast.unparse(_function(tree, "heading_callback"))

        self.assertIn("self.nav_context['nav_state'] = NavState.PAUSE", helper)
        self.assertIn("self.nav_context['pause_reason'] = 'auto_heading_gate'", helper)
        self.assertIn("self.publish_nav_state(NavState.PAUSE)", helper)
        self.assertIn("'auto_heading_gate'", timer)
        self.assertIn("保持PAUSE", recovery)

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

    def test_initial_move_does_not_duplicate_heading_gate_inside_generator(self):
        _, tree = _source_tree()
        generator = ast.unparse(_function(tree, "multi_waypoint_nav_generator"))

        self.assertNotIn("_last_heading_stable", generator)
        self.assertNotIn("初始航向校验", generator)
        self.assertIn("current_nav_state = NavState.INITIAL_MOVE", generator)

    def test_waypoint_attitude_change_restarts_auto_heading_gate(self):
        source, tree = _source_tree()
        callback = ast.unparse(_function(tree, "heading_callback"))
        checker = ast.unparse(_function(tree, "_check_waypoint_attitude_change"))

        self.assertIn("WAYPOINT_ATTITUDE_CHANGE_THRESHOLD = 15.0", source)
        self.assertIn("self._check_waypoint_attitude_change(msg, now)", callback)
        self.assertIn("NavState.WAYPOINT_MOVE", checker)
        self.assertIn("msg.angle_x", checker)
        self.assertIn("msg.angle_y", checker)
        self.assertIn("self.publish_stop_speed()", checker)
        self.assertIn("self._prepare_auto_cleaning_heading_gate", checker)
        self.assertIn("self.multi_waypoint_generator = None", checker)


if __name__ == "__main__":
    unittest.main()
