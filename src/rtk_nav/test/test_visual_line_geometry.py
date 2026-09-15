# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import json
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
        "weighted_line_angle",
        "bridge_collinear_line_records",
        "select_minimum_length_lines",
        "wrap_text_to_pixel_width",
        "select_angle_line_candidates",
        "center_band_half_extent_px",
        "lateral_error_sign_for_image_rotation",
        "line_salience_score",
        "select_most_salient_line",
        "select_tracking_candidates",
        "select_center_line_candidates",
        "select_coarse_white_lines",
        "line_normal_offset_at_reference",
        "select_line_for_tracking",
        "update_line_tracking_state",
        "parse_nav_state_message",
        "nav_state_allows_lateral_output",
        "select_boundary_pair",
        "select_reference_line",
    }
    nodes = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "json": json,
        "math": math,
        "WAYPOINT_MOVE_STATE": "WAYPOINT_MOVE",
    }
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE_PATH), "exec"), namespace)
    return namespace


def test_path_angles_use_undirected_180_degree_geometry():
    helpers = _helpers()
    assert helpers["wrap180"](190.0) == -170.0
    assert helpers["wrap180"](-190.0) == 170.0
    assert helpers["undirected_angle_distance"](0.0, 180.0) == 0.0
    assert helpers["undirected_angle_distance"](0.0, 90.0) == 90.0


def test_180_degree_image_rotation_reverses_lateral_error_sign():
    helpers = _helpers()
    sign_for_rotation = helpers["lateral_error_sign_for_image_rotation"]

    assert sign_for_rotation(0) == 1.0
    assert sign_for_rotation(180) == -1.0


def test_single_line_selection_prefers_long_wide_well_supported_line():
    helpers = _helpers()
    shorter = (100, 0, 300, 480, 520.0, 90.0, 200.0, 240.0, 12.0, 0.95)
    strongest = (320, 0, 320, 480, 480.0, 90.0, 320.0, 240.0, 20.0, 0.95)
    weaker_support = (500, 0, 500, 400, 400.0, 90.0, 500.0, 200.0, 20.0, 0.50)

    selected = helpers["select_most_salient_line"](
        [shorter, strongest, weaker_support]
    )

    assert selected == strongest


def test_center_line_candidates_reject_lines_near_image_edges():
    helpers = _helpers()
    center = (300, 0, 300, 480, 480.0, 88.0, 300.0, 240.0, 8.0, 0.90)
    left_edge = (20, 0, 20, 480, 480.0, 45.0, 20.0, 240.0, 8.0, 0.90)
    right_edge = (620, 0, 620, 480, 480.0, -45.0, 620.0, 240.0, 8.0, 0.90)
    invalid = (0, 0, 100, 100, 100.0, float("nan"), 50.0, 50.0, 8.0, 0.90)

    selected = helpers["select_center_line_candidates"](
        [center, left_edge, right_edge, invalid],
        90.0,
        640,
        480,
        center_band_ratio=0.8,
    )

    assert selected == [center]


def test_center_line_candidates_reject_lines_near_top_and_bottom_edges():
    helpers = _helpers()
    center = (320, 0, 320, 480, 480.0, 90.0, 320.0, 240.0, 8.0, 0.90)
    top_edge = (320, 0, 320, 60, 60.0, 90.0, 320.0, 30.0, 8.0, 0.90)
    bottom_edge = (320, 420, 320, 480, 60.0, 90.0, 320.0, 450.0, 8.0, 0.90)

    selected = helpers["select_center_line_candidates"](
        [center, top_edge, bottom_edge],
        90.0,
        640,
        480,
        center_band_ratio=0.8,
    )

    assert selected == [center]


def test_center_line_candidates_keep_multiple_lines_for_angle_average():
    helpers = _helpers()
    first = (285, 0, 300, 480, 480.0, 80.0, 292.5, 240.0, 8.0, 0.90)
    second = (320, 0, 335, 480, 480.0, 82.0, 327.5, 240.0, 8.0, 0.90)
    edge = (20, 0, 20, 480, 480.0, 40.0, 20.0, 240.0, 8.0, 0.90)

    selected = helpers["select_center_line_candidates"](
        [first, second, edge],
        90.0,
        640,
        480,
        center_band_ratio=0.8,
    )
    average_angle = helpers["weighted_line_angle"](selected)

    assert selected == [first, second]
    assert math.isclose(average_angle, 81.0, abs_tol=0.2)


def test_angle_candidates_follow_slanted_grid_not_rtk_axis():
    helpers = _helpers()
    first = (250, 0, 350, 480, 490.3, 78.2, 300.0, 240.0, 2.0, 0.85)
    second = (300, 0, 400, 480, 490.3, 78.2, 350.0, 240.0, 2.0, 0.85)
    horizontal = (0, 120, 640, 120, 640.0, 0.0, 320.0, 120.0, 2.0, 0.9)
    edge = (-30, 0, 70, 480, 490.3, 78.2, 20.0, 240.0, 2.0, 0.85)

    selected = helpers["select_angle_line_candidates"](
        [first, second, horizontal, edge],
        90.0,
        640,
        480,
        center_band_ratio=0.8,
        max_angle_delta_deg=25.0,
    )

    assert selected == [first, second]
    assert math.isclose(
        helpers["weighted_line_angle"](selected), 78.2, abs_tol=0.2
    )


def test_angle_candidates_follow_axis_perpendicular_to_running_direction():
    helpers = _helpers()
    running_axis = 90.0
    perpendicular_line = (
        0,
        240,
        640,
        240,
        640.0,
        0.0,
        320.0,
        240.0,
        2.0,
        0.85,
    )
    running_direction_line = (
        320,
        0,
        320,
        480,
        480.0,
        90.0,
        320.0,
        240.0,
        2.0,
        0.85,
    )

    selected = helpers["select_angle_line_candidates"](
        [perpendicular_line, running_direction_line],
        helpers["undirected_angle"](running_axis + 90.0),
        640,
        480,
        center_band_ratio=0.8,
        max_angle_delta_deg=25.0,
    )

    assert selected == [perpendicular_line]


def test_edge_line_can_still_be_used_by_full_frame_line_tracking():
    helpers = _helpers()
    edge = (20, 0, 20, 480, 480.0, 90.0, 20.0, 240.0, 8.0, 0.90)

    selected = helpers["select_line_for_tracking"](
        [edge],
        90.0,
        640,
        480,
        previous_offset_px=None,
    )

    assert selected is not None
    assert selected[0] == edge


def test_lateral_tracking_uses_only_path_parallel_coarse_lines():
    helpers = _helpers()
    coarse = (320, 0, 320, 480, 480.0, 90.0, 320.0, 240.0, 8.0, 0.9)
    fine = (300, 0, 300, 480, 480.0, 90.0, 300.0, 240.0, 2.0, 0.8)

    selected, source = helpers["select_tracking_candidates"]([coarse], [fine])
    assert selected == [coarse]
    assert source == "coarse"

    selected, source = helpers["select_tracking_candidates"]([], [fine])
    assert selected == []
    assert source == "none"

    selected, source = helpers["select_tracking_candidates"]([], [])
    assert selected == []
    assert source == "none"


def test_center_band_default_is_eighty_percent_without_debug_boundary_overlay():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    assert (
        "self.declare_parameter('angle_average_center_band_ratio', 0.8)"
        in source
    )
    assert "Angle ROI:" not in source
    assert "(255, 0, 255)" not in source


def test_lateral_deviation_label_uses_bgr_red():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    colors = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute) or node.func.attr != "putText":
            continue
        if len(node.args) < 6:
            continue
        label = node.args[1]
        if isinstance(label, (ast.JoinedStr, ast.IfExp)) and any(
            isinstance(value, ast.Constant)
            and "Lat Dev:" in str(value.value)
            for value in ast.walk(label)
        ):
            colors.append(ast.literal_eval(node.args[5]))

    assert colors == [(0, 0, 255)]


def test_collinear_gap_bridge_uses_segment_geometry_and_angle_gate():
    helpers = _helpers()
    first = (40, 0, 40, 100, 100.0, 90.0, 40.0, 50.0, 2.0, 0.9)
    second = (40, 106, 40, 200, 94.0, 90.0, 40.0, 153.0, 2.0, 0.8)
    angle_mismatch = (40, 106, 48, 152, 46.7, 80.0, 44.0, 129.0, 2.0, 0.9)

    bridge = helpers["bridge_collinear_line_records"](
        [first, second], 90.0, 6.0, 5.0, 3.0
    )
    rejected = helpers["bridge_collinear_line_records"](
        [first, angle_mismatch], 90.0, 6.0, 5.0, 3.0
    )

    assert len(bridge) == 1
    assert math.isclose(float(bridge[0][4]), 194.0, abs_tol=1e-6)
    assert len(rejected) == 2


def test_angle_lines_are_filtered_by_length_after_gap_bridging():
    helpers = _helpers()
    connected = (40, 0, 40, 120, 70.0, 90.0, 40.0, 60.0, 2.0, 0.9)
    fragment = (80, 0, 80, 30, 30.0, 88.0, 80.0, 15.0, 2.0, 0.9)

    selected = helpers["select_minimum_length_lines"](
        [connected, fragment], 60.0
    )

    assert selected == [connected]


def test_angle_lines_reject_wide_reflection_candidates():
    helpers = _helpers()
    thin_grid = (40, 0, 40, 120, 120.0, 90.0, 40.0, 60.0, 2.5, 0.9)
    wide_reflection = (
        80,
        0,
        80,
        120,
        120.0,
        90.0,
        80.0,
        60.0,
        14.0,
        0.95,
    )

    selected = helpers["select_coarse_white_lines"](
        [thin_grid, wide_reflection], 12.0, 1.0, 0.20, 6.0
    )

    assert selected == [thin_grid]


def test_debug_text_wraps_to_the_requested_pixel_width():
    helpers = _helpers()

    lines = helpers["wrap_text_to_pixel_width"](
        "Image axis: -5.0 deg [fallback]",
        12,
        lambda text: len(text),
    )

    assert lines == ["Image axis:", "-5.0 deg", "[fallback]"]
    assert all(len(line) <= 12 for line in lines)


def test_gap_bridge_is_not_mask_morphology_fill():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    assert "def bridge_collinear_line_records" in source
    assert "bridge_collinear_line_records" in source
    assert "angle_line_gap_fill_px" in source
    assert "angle_line_hough_gap_px" in source


def test_weighted_line_angle_handles_unequal_line_lengths():
    helpers = _helpers()
    short = (0, 0, 10, 56, 56.9, 70.0, 5.0, 28.0, 8.0, 0.9)
    long = (0, 0, 10, 57, 57.9, 80.0, 5.0, 28.0, 8.0, 0.9)

    average_angle = helpers["weighted_line_angle"]([short, long])

    assert 74.0 < average_angle < 78.0


def test_weighted_line_angle_handles_the_undirected_wrap_boundary():
    helpers = _helpers()
    first = (0, 0, 10, 10, 100.0, 89.0, 5.0, 5.0, 8.0, 0.9)
    second = (0, 0, 10, 10, 100.0, -89.0, 5.0, 5.0, 8.0, 0.9)

    average_angle = helpers["weighted_line_angle"]([first, second])

    assert math.isclose(abs(average_angle), 90.0, abs_tol=1e-9)


def test_detector_uses_center_average_for_heading_and_tracked_line_for_lateral():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    detector = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "GridLineDetector"
    )
    detect_method = next(
        node
        for node in detector.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "detect_and_draw_grid_lines"
    )
    rendered = ast.unparse(detect_method)

    assert "select_center_line_candidates" in rendered
    assert "weighted_line_angle" in rendered
    assert "selected_line_offset" in rendered
    assert "lateral_pixel_error = selected_line_offset" in rendered


def test_single_line_offset_is_measured_from_image_center():
    helpers = _helpers()
    center_line = (320, 0, 320, 480, 480.0, 90.0, 320.0, 240.0, 12.0, 0.95)
    shifted_line = (300, 0, 300, 480, 480.0, 90.0, 300.0, 240.0, 12.0, 0.95)

    center_offset = helpers["line_normal_offset_at_reference"](
        center_line, 90.0, 640, 480, 0.0
    )
    shifted_offset = helpers["line_normal_offset_at_reference"](
        shifted_line, 90.0, 640, 480, 0.0
    )

    assert math.isclose(center_offset, 0.0, abs_tol=1e-9)
    assert math.isclose(shifted_offset, 20.0, abs_tol=1e-9)


def test_tracking_prefers_previous_line_over_a_stronger_adjacent_line():
    helpers = _helpers()
    tracked = (300, 0, 300, 480, 480.0, 90.0, 300.0, 240.0, 5.0, 0.80)
    adjacent = (450, 0, 450, 480, 480.0, 90.0, 450.0, 240.0, 30.0, 1.00)

    selected = helpers["select_line_for_tracking"](
        [tracked, adjacent],
        90.0,
        640,
        480,
        previous_offset_px=20.0,
        max_jump_px=30.0,
    )

    assert selected is not None
    assert selected[0] == tracked
    assert math.isclose(selected[1], 20.0, abs_tol=1e-9)


def test_tracking_rejects_candidates_that_jump_beyond_gate():
    helpers = _helpers()
    line = (450, 0, 450, 480, 480.0, 90.0, 450.0, 240.0, 30.0, 1.00)

    assert (
        helpers["select_line_for_tracking"](
            [line],
            90.0,
            640,
            480,
            previous_offset_px=20.0,
            max_jump_px=30.0,
        )
        is None
    )


def test_tracking_rejects_the_indoor_adjacent_grid_line_jump():
    helpers = _helpers()
    locked = (320, 0, 320, 480, 480.0, 90.0, 320.0, 240.0, 8.0, 0.90)
    adjacent_grid = (398, 0, 398, 480, 480.0, 90.0, 398.0, 240.0, 30.0, 1.00)

    selected, anchor, missed, status = helpers["update_line_tracking_state"](
        [locked],
        90.0,
        640,
        480,
        max_jump_px=30.0,
        max_missed_frames=2,
    )
    assert selected == locked
    assert math.isclose(anchor, 0.0, abs_tol=1e-9)
    assert missed == 0
    assert status == "acquired"

    selected, next_anchor, missed, status = helpers[
        "update_line_tracking_state"
    ](
        [adjacent_grid],
        90.0,
        640,
        480,
        previous_offset_px=anchor,
        max_jump_px=30.0,
        max_missed_frames=2,
    )
    assert selected is None
    assert math.isclose(next_anchor, 0.0, abs_tol=1e-9)
    assert missed == 1
    assert status == "rejected"


def test_tracking_reacquires_only_near_the_last_valid_anchor():
    helpers = _helpers()
    initial = (300, 0, 300, 480, 480.0, 90.0, 300.0, 240.0, 5.0, 0.80)
    adjacent = (450, 0, 450, 480, 480.0, 90.0, 450.0, 240.0, 30.0, 1.00)
    nearby = (305, 0, 305, 480, 480.0, 90.0, 305.0, 240.0, 5.0, 0.80)

    selected, anchor, missed, status = helpers["update_line_tracking_state"](
        [initial], 90.0, 640, 480, max_jump_px=30.0, max_missed_frames=2
    )
    assert selected == initial
    assert math.isclose(anchor, 20.0, abs_tol=1e-9)
    assert missed == 0
    assert status == "acquired"

    selected, anchor, missed, status = helpers["update_line_tracking_state"](
        [adjacent],
        90.0,
        640,
        480,
        previous_offset_px=anchor,
        missed_frames=0,
        max_jump_px=30.0,
        max_missed_frames=2,
    )
    assert selected is None
    assert math.isclose(anchor, 20.0, abs_tol=1e-9)
    assert missed == 1
    assert status == "rejected"

    selected, anchor, missed, status = helpers["update_line_tracking_state"](
        [],
        90.0,
        640,
        480,
        previous_offset_px=anchor,
        missed_frames=missed,
        max_jump_px=30.0,
        max_missed_frames=2,
    )
    assert selected is None
    assert math.isclose(anchor, 20.0, abs_tol=1e-9)
    assert missed == 2
    assert status == "reacquire"

    selected, anchor, missed, status = helpers["update_line_tracking_state"](
        [nearby],
        90.0,
        640,
        480,
        previous_offset_px=anchor,
        missed_frames=missed,
        max_jump_px=30.0,
        max_missed_frames=2,
    )
    assert selected == nearby
    assert math.isclose(anchor, 15.0, abs_tol=1e-9)
    assert missed == 0
    assert status == "reacquired"


def test_nav_state_parser_accepts_rtk_json_and_manual_plain_text():
    helpers = _helpers()
    parse_state = helpers["parse_nav_state_message"]

    assert parse_state('{"nav_state":"WAYPOINT_MOVE","seq":3}') == (
        "WAYPOINT_MOVE"
    )
    assert parse_state(" waypoint_move ") == "WAYPOINT_MOVE"
    assert parse_state('{"nav_state":"PAUSE"}') == "PAUSE"
    assert parse_state('{"pause_reason":"test"}') is None
    assert parse_state("") is None


def test_only_waypoint_move_allows_lateral_output():
    helpers = _helpers()
    allows_lateral = helpers["nav_state_allows_lateral_output"]

    assert allows_lateral("WAYPOINT_MOVE")
    assert allows_lateral("waypoint_move")
    for state in (
        None,
        "IDLE",
        "INITIAL_MOVE",
        "WAYPOINT_CALIB",
        "PAUSE",
        "COMPLETED",
    ):
        assert not allows_lateral(state)


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


def test_single_reference_line_uses_calibrated_target_not_image_center():
    helpers = _helpers()
    line = (300, 0, 300, 480, 480.0, 90.0, 300.0, 240.0)
    selected = helpers["select_reference_line"](
        [line], 90.0, 640, 480, 0.0, 40.0
    )
    assert selected is not None
    _, observed_projection, target_projection = selected
    assert math.isclose(
        observed_projection - target_projection, 20.0, abs_tol=1e-9
    )


def test_reference_line_returns_none_when_no_line_matches_target():
    helpers = _helpers()
    line = (300, 0, 300, 480, 480.0, 90.0, 300.0, 240.0)
    assert (
        helpers["select_reference_line"](
            [line], 90.0, 640, 480, 100.0, 40.0
        )
        is None
    )


def test_reference_line_position_is_invariant_to_visible_segment_range():
    helpers = _helpers()
    first = (250, 0, 370, 480, 494.0, 75.0, 310.0, 240.0)
    second = (275, 100, 370, 480, 391.0, 75.0, 322.5, 290.0)
    first_offset = helpers["line_normal_offset_at_reference"](
        first, 90.0, 640, 480, 0.0
    )
    second_offset = helpers["line_normal_offset_at_reference"](
        second, 90.0, 640, 480, 0.0
    )
    assert math.isclose(first_offset, second_offset, abs_tol=1e-9)


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
                (0, 100, 480, 100, 480.0, 0.0, 240.0, 100.0),
                (0, 340, 480, 340, 480.0, 0.0, 240.0, 340.0),
            ]

        return [
            (230, 0, 230, 480, 480.0, 90.0, 230.0, 240.0),
            (470, 0, 470, 480, 480.0, 90.0, 470.0, 240.0),
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
        self.assertAlmostEqual(errors[0.0], -20.0, delta=1e-9)
        self.assertAlmostEqual(errors[180.0], 20.0, delta=1e-9)
