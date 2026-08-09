# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"
PLANNER_PATH = Path(__file__).parents[1] / "rtk_nav" / "full_path_planner_dense.py"
BATCH_PLANNER_PATH = Path(__file__).parents[3] / "batch_generate_paths.py"
BRIDGE_CONFIG_PATH = Path(__file__).parents[1] / "rtk_nav" / "config" / "001-E1-E8.yaml"


def _source_tree():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    return source, ast.parse(source)


def _function(tree, name):
    return next(node for node in ast.walk(tree)
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name == name)


def _call_lines(function, callee):
    return [node.lineno for node in ast.walk(function)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == callee]


def _has_boundary_guard(node):
    return (
        isinstance(node, ast.If)
        and (
            "boundary_correct_locked" in ast.unparse(node.test)
            or "_is_calibration_boundary_active" in ast.unparse(node.test)
        )
    )


def test_boundary_back_duration_is_two_seconds():
    source, _ = _source_tree()
    assert "self.BOUNDARY_BACK_DURATION = 2.0" in source


def test_calibration_handlers_guard_generator_before_next():
    _, tree = _source_tree()
    waypoint_calib = _function(tree, "multi_waypoint_nav_generator")
    helper = _function(tree, "_calibrate_with_boundary_retreat")

    for function in (waypoint_calib, helper):
        next_lines = _call_lines(function, "next")
        assert next_lines
        guarded_lines = [node.lineno for node in ast.walk(function)
                         if _has_boundary_guard(node)]
        assert guarded_lines
        assert any(
            any(guard_line < next_line for guard_line in guarded_lines)
            for next_line in next_lines
        )


def test_heading_abnormal_count_is_frozen_while_boundary_is_locked():
    source, _ = _source_tree()
    assert (
        'if not in_bearing_mode and not self.boundary_correct_locked:'
        in source
    )
    assert source.count(
        'if not in_bearing_mode and not self.boundary_correct_locked:'
    ) >= 2


def test_boundary_cycle_pause_reason_is_manual_intervention():
    source, _ = _source_tree()
    assert '"boundary_cycle_exhausted"' in source
    assert 'self.nav_context["pause_reason"] = "boundary_cycle_exhausted"' in source


def test_geometric_p1_rejects_stop_iteration_while_sensor_active():
    _, tree = _source_tree()
    retreat = _function(tree, "_retreat_to_waypoint")
    stop_handlers = [
        node for node in ast.walk(retreat)
        if isinstance(node, ast.ExceptHandler)
        and isinstance(node.type, ast.Name)
        and node.type.id == "StopIteration"
    ]
    assert any(
        "_is_retreat_turn_blocked(target_heading)" in ast.unparse(handler)
        and "P1_SENSOR_BLOCKED" in ast.unparse(handler)
        for handler in stop_handlers
    )


def test_geometric_p1_checks_boundary_before_advancing_generator():
    _, tree = _source_tree()
    retreat = _function(tree, "_retreat_to_waypoint")
    p1_loops = [
        node for node in ast.walk(retreat)
        if isinstance(node, ast.While)
        and any(
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "next"
            and "calib_gen" in ast.unparse(call)
            for call in ast.walk(node)
        )
    ]
    assert p1_loops
    for loop in p1_loops:
        next_lines = [
            call.lineno for call in ast.walk(loop)
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "next"
            and "calib_gen" in ast.unparse(call)
        ]
        guard_lines = [
            node.lineno for node in ast.walk(loop)
            if isinstance(node, ast.If)
            and "_is_retreat_turn_blocked(target_heading)" in ast.unparse(node.test)
        ]
        assert guard_lines and min(guard_lines) < min(next_lines)


def test_retreat_pause_preserves_real_navigation_state():
    _, tree = _source_tree()
    pause = _function(tree, "_pause_boundary_retreat_timeout")
    source = ast.unparse(pause)
    assert "pre_pause_state = self.nav_context.get('nav_state')" in source
    assert "self.nav_context['pre_pause_state'] = pre_pause_state" in source


def test_boundary_cycle_prefers_locked_sensor_signature():
    _, tree = _source_tree()
    recorder = _function(tree, "_record_boundary_cycle")
    source = ast.unparse(recorder)
    assert "sensor_signature = self.boundary_cycle_sensor_signature" in source
    assert "if not sensor_signature" in source


def test_geometric_retry_does_not_use_ordinary_boundary_correction():
    source, _ = _source_tree()
    assert 'retry_count >= 2 and not self.nav_context.get("retreat_enabled")' in source
    assert "几何回退来源跳过普通边界后退" in source


def test_heading_recalibration_does_not_rewrite_navigation_state():
    _, tree = _source_tree()
    recalibration = _function(tree, "start_heading_recalibration")
    assignments = [
        node for node in ast.walk(recalibration)
        if isinstance(node, ast.Assign)
        and "nav_context" in ast.unparse(node.targets[0])
        and "nav_state" in ast.unparse(node.targets[0])
    ]
    assert not assignments


def test_heading_recalibration_enables_geometric_retreat():
    _, tree = _source_tree()
    recalibration = _function(tree, "start_heading_recalibration")
    assert any(
        "retreat_enabled" in ast.unparse(node.targets[0])
        and isinstance(node.value, ast.Constant)
        and node.value.value is True
        for node in ast.walk(recalibration)
        if isinstance(node, ast.Assign)
    )


def test_geometric_retreat_is_enabled_for_heading_recovery_sources():
    _, tree = _source_tree()
    helper = _function(tree, "_calibrate_with_boundary_retreat")
    tuple_values = [
        node.attr
        for node in ast.walk(helper)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "CalibOrigin"
    ]
    assert "HEADING_RECOVERY" in tuple_values
    assert "FORCE_BEARING" in tuple_values


def test_anchor_sensor_trigger_has_bounded_escape_phase():
    source, tree = _source_tree()
    retreat = _function(tree, "_retreat_to_waypoint")
    retreat_source = ast.unparse(retreat)
    assert "P1_ESCAPE" in source
    assert "_select_escape_plan" in source
    assert "RETREAT_ESCAPE_DURATION" in source
    assert "retreat_escape_start_time" in retreat_source


def test_anchor_escape_stops_without_safe_linear_direction():
    source, tree = _source_tree()
    selector = _function(tree, "_select_escape_plan")
    selector_source = ast.unparse(selector)
    assert "FORWARD" in selector_source
    assert "BACKWARD" in selector_source
    assert "return False" in selector_source
    assert "P1_NO_SAFE_CANDIDATE" in source


def test_geometric_escape_does_not_delegate_to_boundary_correction():
    _, tree = _source_tree()
    retreat = _function(tree, "_retreat_to_waypoint")
    assert "get_boundary_correct_speed" not in ast.unparse(retreat)


def test_bridge_suppression_covers_move_and_calibration_with_one_area_list():
    _, tree = _source_tree()
    gate = _function(tree, "_is_ultrasonic_suppression_allowed")
    source = ast.unparse(gate)
    assert "NavState.WAYPOINT_MOVE" in source
    assert "NavState.WAYPOINT_CALIB" in source
    assert "ultrasonic_suppression_areas" in source
    assert "configured_areas" not in source
    assert "_rtk_ready_for_ultrasonic_suppression" in source
    rtk_gate = _function(tree, "_rtk_ready_for_ultrasonic_suppression")
    rtk_source = ast.unparse(rtk_gate)
    assert "last_wtrtk_time" in rtk_source
    assert "rtk_data_timed_out" in rtk_source
    assert "RTK_DATA_TIMEOUT" in rtk_source


def test_bridge_suppression_covers_immediate_calibration_sensor_guard():
    _, tree = _source_tree()
    boundary_guard = _function(tree, "_is_calibration_boundary_active")
    source = ast.unparse(boundary_guard)
    assert "_is_ultrasonic_suppression_allowed" in source
    assert "not suppression_active" in source
    assert "confirmed_sensors" in source
    assert "mid_left" in source


def test_bridge_suppression_has_no_configured_time_or_distance_limit():
    _, tree = _source_tree()
    gate = _function(tree, "_is_ultrasonic_suppression_allowed")
    source = ast.unparse(gate)
    assert "ultrasonic_suppression_max_duration_s" not in source
    assert "ultrasonic_suppression_max_distance_m" not in source


def test_bridge_suppression_exits_for_retreat_and_non_auto_states():
    source, tree = _source_tree()
    gate = _function(tree, "_is_ultrasonic_suppression_allowed")
    gate_source = ast.unparse(gate)
    assert "retreat_active" in gate_source
    assert "boundary_correct_locked" in gate_source
    assert "ControlMode.AUTO_CLEANING" in gate_source
    assert "_suppression_area_identity" in gate_source
    assert "ultrasonic_suppression_active" in source


def test_bridge_suppression_session_resets_on_pause_and_auto_reentry():
    _, tree = _source_tree()
    publish_state = _function(tree, "publish_nav_state")
    state_callback = _function(tree, "state_callback")
    mode_callback = _function(tree, "mode_callback")
    publish_source = ast.unparse(publish_state)
    assert "NavState.PAUSE" in publish_source
    assert "_exit_ultrasonic_suppression" in publish_source
    assert "enter_auto_cleaning" in ast.unparse(state_callback)
    assert "enter_auto_cleaning" in ast.unparse(mode_callback)


def test_bridge_suppression_metadata_is_persisted_in_generated_path():
    planner_source = PLANNER_PATH.read_text(encoding="utf-8")
    batch_source = BATCH_PLANNER_PATH.read_text(encoding="utf-8")
    config_source = BRIDGE_CONFIG_PATH.read_text(encoding="utf-8")
    assert "ultrasonic_suppression" in planner_source
    assert "json.dumps(suppression_meta" in planner_source
    assert "json.dumps(suppression_meta" in batch_source
    assert "suppression_areas" in batch_source
    assert '"bridge_E3-E3out2"' in config_source
    assert '"bridge_E4out1-E4"' in config_source
    assert "calibration_areas" not in config_source
    assert "max_duration_s" not in config_source
    assert "max_distance_m" not in config_source
