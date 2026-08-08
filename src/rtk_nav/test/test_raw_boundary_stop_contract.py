# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _function(tree, name):
    return next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_raw_boundary_trigger_stops_before_confirmation_and_p1():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    service_source = ast.unparse(_function(tree, "_service_calibration_boundary"))
    move_source = ast.unparse(_function(tree, "multi_waypoint_nav_generator"))

    assert "not self.confirmed_sensors" in service_source
    assert "self.publish_stop_speed()" in service_source
    assert "return 'wait'" in service_source
    assert "raw_boundary_stop" in move_source
    assert "not self.confirmed_sensors" in move_source
    assert "not self.nav_context.get('retreat_active')" in move_source
    assert "not suppression_active" in move_source
