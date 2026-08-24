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


def test_launch_declares_visual_correction_default_on_and_passes_visual_gates():
    source = LAUNCH_SOURCE_PATH.read_text(encoding="utf-8")

    assert '"enable_visual_correction"' in source
    assert 'default_value=TextSubstitution(text="true")' in source
    assert '"bypass_path_context_gate"' in source
    assert 'default_value=TextSubstitution(text="true")' in source
    assert 'LaunchConfiguration("bypass_path_context_gate")' in source
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
        ("visual_confidence_threshold", "0.75"),
        ("visual_timeout_sec", "0.5"),
        ("target_line_offset_m", "nan"),
        ("target_line_match_tolerance_m", "0.5"),
        ("reference_axis_offset_px", "0.0"),
    ):
        assert f'"{name}"' in source
        assert f'default_value=TextSubstitution(text="{default}")' in source
        assert f"'{name}': ParameterValue(" in source
        assert f'LaunchConfiguration("{name}")' in source
    assert source.count("value_type=float") == 10


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


def test_line_detector_uses_single_rendered_argument_for_logger_calls():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)

    invalid_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr not in {"debug", "info", "warn", "warning", "error", "fatal"}:
            continue
        if isinstance(node.func.value, ast.Call):
            if (
                isinstance(node.func.value.func, ast.Attribute)
                and node.func.value.func.attr == "get_logger"
                and len(node.args) > 1
            ):
                invalid_calls.append(node.lineno)

    assert invalid_calls == []


def test_line_detector_logs_parallel_and_perpendicular_group_counts():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    detector = ast.unparse(_function(tree, "detect_and_draw_grid_lines"))

    assert "self.get_logger().info" in detector
    assert "P:{len(parallel_group)} C:{len(perpendicular_group)}" in detector
    assert "P:0 C:0" in detector


def test_line_detector_logs_geometry_gate_state_for_offset_diagnostics():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    detector = ast.unparse(_function(tree, "detect_and_draw_grid_lines"))

    assert "pair={pair is not None}" in detector
    assert "geometry={valid_geometry}" in detector
    assert "streak={self.valid_streak}/{self.reacquire_frames}" in detector


def test_rtk_declares_and_reports_visual_correction_switch():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    context_publisher = ast.unparse(_function(tree, "publish_nav_context"))
    visual_context = ast.unparse(_function(tree, "publish_visual_path_context"))

    assert "self.declare_parameter('enable_visual_correction', True)" in initializer
    assert "self.enable_visual_correction" in initializer
    assert "'enable_visual_correction'" in context_publisher
    assert "self.visual_path_context_pub.publish" in visual_context
    assert "self.rtk_solution_ready" in visual_context
    assert "msg.z = 0.0" in visual_context


def test_rtk_publishes_current_path_reference_with_validity_and_projection():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    reference = ast.unparse(_function(tree, "publish_visual_path_reference"))

    assert "'/rtk/visual_path_reference'" in initializer
    assert "self.visual_path_reference_pub" in initializer
    assert "self.calculate_lateral_error" in reference
    assert "self._get_projection_ratio" in reference
    assert "msg.x" in reference
    assert "msg.y" in reference
    assert "msg.z = 0.0" in reference


def test_line_detector_exposes_independent_visual_status_and_path_reference():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    callback = ast.unparse(_function(tree, "path_reference_callback"))
    publish_invalid = ast.unparse(_function(tree, "publish_invalid"))

    assert "'/rtk/visual_path_reference'" in initializer
    assert "'/grid_line/heading_valid'" in initializer
    assert "'/grid_line/lateral_valid'" in initializer
    assert "'/grid_line/heading_confidence'" in initializer
    assert "'/grid_line/lateral_confidence'" in initializer
    assert "self.path_reference_valid" in callback
    assert "self.path_reference_lateral_m" in callback
    assert "self.path_reference_projection_ratio" in callback
    assert "self.heading_valid_pub.publish" in publish_invalid
    assert "self.lateral_valid_pub.publish" in publish_invalid
    assert "self.heading_confidence_pub.publish" in publish_invalid
    assert "self.lateral_confidence_pub.publish" in publish_invalid


def test_line_detector_uses_calibrated_reference_line_for_lateral_offset():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    detector = ast.unparse(_function(tree, "detect_and_draw_grid_lines"))
    reference_selector = ast.unparse(_function(tree, "select_reference_line"))

    assert "target_line_offset_m" in initializer
    assert "target_line_match_tolerance_m" in initializer
    assert "reference_axis_offset_px" in initializer
    assert "select_reference_line" in detector
    assert "line_normal_offset_at_reference" in reference_selector
    assert "target_line_offset_m" in detector
    assert "reference_line is not None" in detector
    assert "(left_projection + right_projection) / 2.0" not in detector


def test_rtk_consumes_independent_visual_components_with_separate_gates():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    correction = ast.unparse(_function(tree, "get_visual_steering_correction"))

    assert "self.visual_heading_valid_callback" in initializer
    assert "self.visual_lateral_valid_callback" in initializer
    assert "self.visual_heading_confidence_callback" in initializer
    assert "self.visual_lateral_confidence_callback" in initializer
    assert "self.visual_heading_valid" in correction
    assert "self.visual_lateral_valid" in correction
    assert "self.visual_heading_confidence" in correction
    assert "self.visual_lateral_confidence" in correction
    assert "self.visual_heading_valid" in correction
    assert "self.visual_lateral_valid" in correction


def test_line_detector_defaults_enabled_and_publishes_invalid_output_when_disabled():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    image_callback = ast.unparse(_function(tree, "image_callback"))

    assert "enable_visual_correction" in initializer
    assert "True" in initializer
    assert "self.enable_visual_correction" in image_callback
    assert "publish_invalid" in image_callback
    publish_invalid = ast.unparse(_function(tree, "publish_invalid"))
    assert "result.z = 0.0" in publish_invalid
    assert "confidence.data = 0.0" in publish_invalid


def test_line_detector_path_context_bypass_defaults_on_and_preserves_visual_switch():
    source = LINE_DETECTOR_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    initializer = ast.unparse(_function(tree, "__init__"))
    image_callback = ast.unparse(_function(tree, "image_callback"))

    assert "self.declare_parameter('bypass_path_context_gate', True)" in initializer
    assert "self.bypass_path_context_gate" in initializer
    assert "if not self.enable_visual_correction:" in image_callback
    assert "not self.bypass_path_context_gate" in image_callback
    assert "not self.path_context_valid" in image_callback
    assert "self.path_context_timeout_sec" in image_callback


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


def test_visual_steering_correction_uses_only_independently_valid_components():
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
        nav_context = {"nav_state": NavStateConstants.WAYPOINT_MOVE}
        boundary_correct_locked = False
        visual_heading_valid = True
        visual_lateral_valid = False
        visual_heading_confidence = 0.8
        visual_lateral_confidence = 0.0
        last_visual_heading_time = time.monotonic()
        last_visual_lateral_time = 0.0
        visual_heading_error_deg = 2.0
        visual_lateral_error_m = 100.0
        visual_heading_gain = 1.0
        visual_lateral_gain = 10.0
        visual_max_steering_deg = 20.0
        visual_confidence_threshold = 0.5
        visual_timeout_sec = 0.5

    state = VisualState()
    assert math.isclose(
        namespace["get_visual_steering_correction"](state), -2.0, abs_tol=1e-9
    )

    state.visual_heading_valid = False
    state.visual_lateral_valid = True
    state.visual_heading_confidence = 0.0
    state.visual_lateral_confidence = 0.8
    state.last_visual_heading_time = 0.0
    state.last_visual_lateral_time = time.monotonic()
    assert math.isclose(
        namespace["get_visual_steering_correction"](state), 20.0, abs_tol=1e-9
    )
