# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


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
