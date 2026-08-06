# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _function(tree, name):
    return next(node for node in ast.walk(tree)
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name == name)


def _source_tree():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    return source, ast.parse(source)


def test_heading_callback_tracks_continuous_zero_angle_frames():
    _, tree = _source_tree()
    callback = ast.unparse(_function(tree, "heading_callback"))

    assert "zero_angle_start_time" in callback
    assert "msg.angle_x == 0.0 and msg.angle_y == 0.0" in callback


def test_rtk_timeout_handler_consumes_zero_angle_timeout():
    _, tree = _source_tree()
    handler = ast.unparse(_function(tree, "handle_rtk_data_timeout"))

    assert "zero_angle_start_time" in handler
    assert "RTK_DATA_TIMEOUT" in handler
    assert "rtk_timeout" in handler
