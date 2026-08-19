# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).parents[3]
LAUNCH_SOURCE_PATH = Path(__file__).parents[1] / "launch" / "run.launch.py"
RTK_SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"
LINE_DETECTOR_SOURCE_PATH = REPO_ROOT / "line_detector_node.py"


def _function(tree, name):
    return next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_launch_declares_visual_correction_default_off_and_passes_it_to_rtk():
    source = LAUNCH_SOURCE_PATH.read_text(encoding="utf-8")

    assert '"enable_visual_correction"' in source
    assert 'default_value=TextSubstitution(text="false")' in source
    assert 'LaunchConfiguration("enable_visual_correction")' in source
    assert "'enable_visual_correction': LaunchConfiguration(\"enable_visual_correction\")" in source


def test_rtk_declares_and_reports_visual_correction_switch():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    context_publisher = ast.unparse(_function(tree, "publish_nav_context"))

    assert "self.declare_parameter('enable_visual_correction', False)" in initializer
    assert "self.enable_visual_correction" in initializer
    assert "'enable_visual_correction'" in context_publisher


def test_line_detector_defaults_disabled_and_publishes_invalid_output_when_disabled():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    image_callback = ast.unparse(_function(tree, "image_callback"))

    assert "enable_visual_correction" in initializer
    assert "False" in initializer
    assert "self.enable_visual_correction" in image_callback
    assert "self.angle_deviation.z = 0.0" in image_callback
    assert "self.confidence.data = 0.0" in image_callback
