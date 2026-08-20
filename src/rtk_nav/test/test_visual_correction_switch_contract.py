# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import math
import time
from pathlib import Path


REPO_ROOT = Path(__file__).parents[3]
LAUNCH_SOURCE_PATH = Path(__file__).parents[1] / "launch" / "run.launch.py"
RTK_SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"
SETUP_SOURCE_PATH = Path(__file__).parents[1] / "setup.py"
CAMERA_PUBLISHER_SOURCE_PATH = (
    Path(__file__).parents[1] / "rtk_nav" / "camera_publisher_node.py"
)
LINE_DETECTOR_SOURCE_PATH = (
    Path(__file__).parents[1] / "rtk_nav" / "line_detector_node.py"
)


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
    assert "value_type=bool" in source
    assert "executable='line_detector_node'" in source
    assert "IfCondition(LaunchConfiguration('enable_visual_correction'))" in source
    assert "'visual_heading_gain': ParameterValue" in source
    assert "'visual_lateral_gain': ParameterValue" in source
    assert "'visual_max_steering_deg': ParameterValue" in source
    assert "'visual_confidence_threshold': ParameterValue" in source
    assert "'visual_timeout_sec': ParameterValue" in source


def test_launch_exposes_independent_rtk_and_visual_tuning_parameters():
    source = LAUNCH_SOURCE_PATH.read_text(encoding="utf-8")

    for name, default in (
        ("stanley_k_path", "0.45"),
        ("stanley_k_near_target", "0.42"),
        ("visual_heading_gain", "0.2"),
        ("visual_lateral_gain", "10.0"),
        ("visual_max_steering_deg", "3.0"),
        ("visual_confidence_threshold", "0.5"),
        ("visual_timeout_sec", "0.5"),
    ):
        assert f'"{name}"' in source
        assert f'default_value=TextSubstitution(text="{default}")' in source
        assert f"'{name}': ParameterValue(" in source
        assert f'LaunchConfiguration("{name}")' in source
    assert source.count("value_type=float") == 7


def test_camera_publisher_is_registered_and_enabled_with_visual_correction():
    setup_source = SETUP_SOURCE_PATH.read_text(encoding="utf-8")
    launch_source = LAUNCH_SOURCE_PATH.read_text(encoding="utf-8")
    camera_source = CAMERA_PUBLISHER_SOURCE_PATH.read_text(encoding="utf-8")
    package_source = (Path(__file__).parents[1] / "package.xml").read_text(
        encoding="utf-8"
    )

    assert "camera_publisher_node = rtk_nav.camera_publisher_node:main" in setup_source
    assert "executable='camera_publisher_node'" in launch_source
    assert "name='camera_publisher'" in launch_source
    assert "self.create_publisher(Image, \"/camera/color/image_raw\", 10)" in camera_source
    assert "<exec_depend>python3-opencv</exec_depend>" in package_source
    assert "node = None" in camera_source
    assert "if node is not None:" in camera_source
    assert launch_source.count(
        "condition=IfCondition(LaunchConfiguration('enable_visual_correction'))"
    ) >= 2


def test_camera_publisher_loads_static_images_as_color_frames():
    source = CAMERA_PUBLISHER_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    loader = _function(tree, "load_static_image")

    class FakeCv2:
        IMREAD_COLOR = 1

        @staticmethod
        def imread(path, mode):
            assert path == "image.png"
            assert mode == FakeCv2.IMREAD_COLOR
            return "bgr-frame"

    namespace = {"cv2": FakeCv2}
    exec(
        compile(ast.Module(body=[loader], type_ignores=[]), str(CAMERA_PUBLISHER_SOURCE_PATH), "exec"),
        namespace,
    )

    assert namespace["load_static_image"]("image.png") == "bgr-frame"


def test_rtk_declares_and_reports_visual_correction_switch():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    context_publisher = ast.unparse(_function(tree, "publish_nav_context"))
    visual_context = ast.unparse(_function(tree, "publish_visual_path_context"))

    assert "self.declare_parameter('enable_visual_correction', False)" in initializer
    assert "self.enable_visual_correction" in initializer
    assert "'enable_visual_correction'" in context_publisher
    assert "self.visual_path_context_pub.publish" in visual_context
    assert "self.rtk_solution_ready" in visual_context
    assert "msg.z = 0.0" in visual_context


def test_line_detector_defaults_disabled_and_publishes_invalid_output_when_disabled():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    image_callback = ast.unparse(_function(tree, "image_callback"))

    assert "enable_visual_correction" in initializer
    assert "False" in initializer
    assert "self.enable_visual_correction" in image_callback
    assert "publish_invalid" in image_callback
    publish_invalid = ast.unparse(_function(tree, "publish_invalid"))
    assert "result.z = 0.0" in publish_invalid
    assert "confidence.data = 0.0" in publish_invalid


def test_line_detector_implements_path_axis_groups_and_reacquisition():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    callback = ast.unparse(_function(tree, "path_context_callback"))
    detector = ast.unparse(_function(tree, "detect_and_draw_grid_lines"))

    assert "wrap180" in callback
    assert "reset_reacquisition" in callback
    assert "parallel_group" in detector
    assert "perpendicular_group" in detector
    assert "select_boundary_pair" in detector
    assert "self.reacquire_frames" in detector


def test_rtk_stanley_consumes_fresh_visual_correction():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    visual_callback = ast.unparse(_function(tree, "visual_angle_callback"))
    visual_correction = ast.unparse(
        _function(tree, "get_visual_steering_correction")
    )
    stanley = ast.unparse(_function(tree, "stanley_steering_control"))

    assert "'/grid_line/angle_deviation'" in initializer
    assert "'/grid_line/detection_confidence'" in initializer
    assert "visual_heading_gain" in initializer
    assert "visual_lateral_gain" in initializer
    assert "visual_max_steering_deg" in initializer
    assert "visual_confidence_threshold" in initializer
    assert "visual_timeout_sec" in initializer
    assert "self.visual_angle_callback" in initializer
    assert "self.visual_detected" in visual_callback
    assert "time.monotonic()" in visual_callback
    assert "self.rtk_solution_ready" in visual_correction
    assert "self.current_control_mode" in visual_correction
    assert "NavState.INITIAL_MOVE" in visual_correction
    assert "NavState.WAYPOINT_MOVE" in visual_correction
    assert "self.boundary_correct_locked" in visual_correction
    assert "self.visual_max_steering_deg" in visual_correction
    assert "visual_correction" in stanley
    assert "total_steering = steering_correction - heading_error + visual_correction" in stanley


def test_rtk_stanley_gain_parameters_select_normal_and_near_target_values():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    helper = _function(tree, "get_adaptive_stanley_k")

    assert "self.declare_parameter('stanley_k_path', 0.45)" in initializer
    assert "self.declare_parameter('stanley_k_near_target', 0.42)" in initializer
    assert "self.stanley_k_path" in initializer
    assert "self.stanley_k_near_target" in initializer

    namespace = {}
    exec(
        compile(
            ast.Module(body=[helper], type_ignores=[]),
            str(RTK_SOURCE_PATH),
            "exec",
        ),
        namespace,
    )

    class State:
        stanley_k_path = 0.31
        stanley_k_near_target = 0.27

    state = State()
    select_gain = namespace["get_adaptive_stanley_k"]
    assert select_gain(state, 0.0, 0.4) == 0.27
    assert select_gain(state, 0.0, 1.3) == 0.31


def test_visual_steering_correction_requires_fresh_confident_motion_sample():
    tree = ast.parse(RTK_SOURCE_PATH.read_text(encoding="utf-8"))
    helper = _function(tree, "get_visual_steering_correction")

    class ControlModeConstants:
        AUTO_CLEANING = "AUTO_CLEANING"

    class NavStateConstants:
        INITIAL_MOVE = "INITIAL_MOVE"
        WAYPOINT_MOVE = "WAYPOINT_MOVE"

    namespace = {
        "math": math,
        "time": time,
        "ControlMode": ControlModeConstants,
        "NavState": NavStateConstants,
    }
    exec(
        compile(ast.Module(body=[helper], type_ignores=[]), str(RTK_SOURCE_PATH), "exec"),
        namespace,
    )

    class VisualState:
        enable_visual_correction = True
        rtk_solution_ready = True
        current_control_mode = ControlModeConstants.AUTO_CLEANING
        nav_context = {
            "nav_state": NavStateConstants.WAYPOINT_MOVE,
            "calibration_active": False,
            "retreat_active": False,
            "force_bearing_mode": False,
        }
        boundary_correct_locked = False
        visual_detected = True
        visual_confidence = 0.8
        visual_confidence_threshold = 0.5
        visual_heading_error_deg = 1.0
        visual_lateral_error_m = 0.2
        visual_heading_gain = 1.0
        visual_lateral_gain = 10.0
        visual_max_steering_deg = 10.0
        visual_timeout_sec = 0.5
        last_visual_angle_time = time.monotonic()

    state = VisualState()
    correction = namespace["get_visual_steering_correction"](state)
    assert math.isclose(correction, 1.0, abs_tol=1e-9)

    state.visual_confidence = 0.1
    assert namespace["get_visual_steering_correction"](state) == 0.0
    state.visual_confidence = 0.8
    state.last_visual_angle_time = time.monotonic() - 1.0
    assert namespace["get_visual_steering_correction"](state) == 0.0
    state.last_visual_angle_time = time.monotonic()
    state.nav_context["nav_state"] = "PAUSE"
    assert namespace["get_visual_steering_correction"](state) == 0.0
