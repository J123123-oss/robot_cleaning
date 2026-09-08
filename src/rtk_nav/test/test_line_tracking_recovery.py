# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "line_detector_node.py"


def _recovery_helper():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "select_line_tracking_recovery_output"
    )
    namespace = {}
    exec(
        compile(
            ast.Module(body=[function], type_ignores=[]),
            str(SOURCE_PATH),
            "exec",
        ),
        namespace,
    )
    return namespace["select_line_tracking_recovery_output"]


def _detector_method():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    detector_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "GridLineDetector"
    )
    return next(
        node
        for node in detector_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "detect_and_draw_grid_lines"
    )


VALID_RESULT = (
    2.0,
    0.04,
    True,
    0.85,
    True,
    True,
    0.85,
    0.85,
)


def test_short_line_loss_keeps_last_valid_output_within_window():
    recover = _recovery_helper()

    assert recover(VALID_RESULT, "lost", 1, 2, True) == VALID_RESULT
    assert recover(VALID_RESULT, "rejected", 2, 2, True) == VALID_RESULT


def test_line_loss_beyond_window_returns_no_recovery_output():
    recover = _recovery_helper()

    assert recover(VALID_RESULT, "reacquire", 3, 2, True) is None


def test_reacquisition_keeps_last_output_until_new_line_is_confirmed():
    recover = _recovery_helper()

    assert recover(VALID_RESULT, "reacquired", 0, 2, True) == VALID_RESULT
    assert recover(VALID_RESULT, "locked", 0, 2, True) == VALID_RESULT


def test_recovery_never_invents_motion_without_previous_valid_output():
    recover = _recovery_helper()

    assert recover(None, "lost", 1, 2, True) is None
    assert recover(VALID_RESULT, "lost", 1, 2, False) is None
    assert recover(VALID_RESULT, "disabled", 0, 2, True) is None


def test_detector_integrates_recovery_before_publishing_invalid_output():
    detector = ast.unparse(_detector_method())

    assert "self.last_valid_visual_result = current_result" in detector
    assert detector.count("self.get_line_tracking_recovery_output") >= 4
    assert "recovery_result is not None" in detector
