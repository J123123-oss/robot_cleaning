"""用真实导航生成器复现日志中的纠偏误停和首点切换残留。"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, Generator, List, Optional, Tuple
from unittest.mock import Mock

import pytest


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _load_navigation():
    """只隔离 ROS 接口，执行源码中的控制方法与生成器。"""
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    method_names = {
        "normalize_angle", "get_path_heading", "get_adaptive_stanley_k",
        "stanley_steering_control", "_stanley_lateral_angle",
        "_tracking_heading_guard_error", "_reset_waypoint_tracking",
        "_record_force_bearing_progress", "_force_bearing_diverging",
        "start_heading_recalibration", "multi_waypoint_nav_generator",
        "move_to_first_waypoint",
    }
    node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                      and n.name == "RTKNavControlNode")
    body = [n for n in tree.body if isinstance(n, ast.Assign)]
    body += [n for n in tree.body if isinstance(n, ast.ClassDef)
             and n.name in {"NavState", "ControlMode", "CalibOrigin"}]
    body += [n for n in tree.body if isinstance(n, ast.FunctionDef)
             and n.name in {"combine_stanley_and_visual_correction",
                            "calculate_uncommanded_heading_error"}]
    body.append(ast.ClassDef(
        name="Navigation", bases=[], keywords=[], decorator_list=[],
        body=[n for n in node_class.body if isinstance(n, ast.FunctionDef)
              and n.name in method_names],
    ))
    namespace = {
        "math": math, "time": SimpleNamespace(monotonic=lambda: 10.0),
        "rclpy": SimpleNamespace(ok=lambda: True),
        "Dict": Dict, "Generator": Generator, "List": List,
        "Optional": Optional, "Tuple": Tuple,
    }
    module = ast.fix_missing_locations(ast.Module(body=body, type_ignores=[]))
    exec(compile(module, str(SOURCE_PATH), "exec"), namespace)
    return namespace


def _calibrate():
    yield (3.0, 3.0)
    yield (0.0, 0.0)
    return True


def _finished_initial_move():
    if False:
        yield
    return True


def _make_node():
    ns = _load_navigation()
    node = ns["Navigation"]()
    node.waypoints = [
        (120.06973751, 30.32021544, 351.80),
        (120.06972614, 30.32028353, 261.72),
        (120.06972100, 30.32028289, 171.80),
    ]
    node.current_waypoint_idx = 1
    node.current_gps = node.waypoints[0][:2]
    node.imu_yaw = 9.3
    node.imu_calibration_offset = 0.0
    node.nav_running = True
    node.brush_active = False
    node.boundary_correct_locked = False
    node.blocked_directions = set()
    node.heading_abnormal_start_time = None
    node.heading_timed_out = False
    node.last_waypoint_cache = node.waypoints[0]
    node.cross_file_last_waypoint = None
    node.stanley_path_start = node.waypoints[0][:2]
    node.stanley_path_direction = -6.8
    node.stanley_k_path = 0.45
    node.stanley_k_near_target = 0.42
    node.rtk_max_correction = ns["RTK_MAX_CORRECTION_SPEED"]
    node.visual_max_correction = 1.5
    node.rtk_correction_ratio = node.visual_correction_ratio = 1.0
    node.visual_sample_heading_error_deg = 0.0
    node._last_visual_heading_correction = 0.0
    node._last_visual_lateral_correction = 0.0
    node._last_stanley_log_time = 0.0
    node.nav_context = {
        "nav_state": ns["NavState"].WAYPOINT_MOVE,
        "target_waypoint": node.waypoints[1],
        "last_distance": 0.0, "last_target_heading": 0.0,
        "angle_abnormal_count": 0, "waypoint_recalib_count": 0,
        "force_bearing_mode": False, "force_bearing_target": None,
        "force_bearing_min_distance": float("inf"),
        "force_bearing_progress_distance": float("inf"),
        "force_bearing_recalib_count": 0, "distance_increase_count": 0,
    }
    node.get_logger = Mock(return_value=Mock())
    node.check_control_mode = Mock(return_value=True)
    node.publish_nav_state = Mock()
    node.publish_stop_speed = Mock()
    node.set_rtk_error_bits = Mock()
    node.check_and_control_brush = Mock()
    node._reset_boundary_cycle_tracking = Mock()
    node._clear_retreat_context = Mock()
    node._is_ultrasonic_suppression_allowed = Mock(return_value=False)
    node._has_raw_boundary_trigger = Mock(return_value=False)
    node._is_calibration_boundary_active = Mock(return_value=False)
    node._is_motion_blocked = Mock(return_value=False)
    node.get_boundary_correct_speed = Mock(return_value=(0.0, 0.0))
    node.get_target_waypoint = lambda idx: node.waypoints[idx]
    node.calc_distance_to_waypoint = Mock(return_value=7.0)
    node.calculate_lateral_error = Mock(return_value=-0.273)
    node.calculate_path_bearing = Mock(return_value=-6.8)
    node.calculate_bearing = Mock(return_value=-6.8)
    node._get_projection_ratio = Mock(return_value=0.2)
    node.get_visual_steering_correction = Mock(return_value=0.0)
    node.calibrate_heading_at_waypoint = Mock(side_effect=lambda *_: _calibrate())
    node._calibrate_with_boundary_retreat = Mock(side_effect=lambda *_: _calibrate())
    node.move_to_first_waypoint = _finished_initial_move
    return node, ns


def test_logged_stanley_recovery_is_not_interrupted_after_five_frames():
    node, ns = _make_node()
    generator = node.multi_waypoint_nav_generator()

    outputs = [next(generator) for _ in range(5)]

    assert node.nav_context["nav_state"] == ns["NavState"].WAYPOINT_MOVE
    assert node.nav_context["waypoint_recalib_count"] == 0
    assert all(left < 0 < right for left, right in outputs)
    node.calibrate_heading_at_waypoint.assert_not_called()


def test_uncommanded_turn_still_stops_and_requests_recalibration():
    node, ns = _make_node()
    node.imu_yaw = -30.0  # 向横向修正所需方向的反侧偏离。
    generator = node.multi_waypoint_nav_generator()

    outputs = [next(generator) for _ in range(5)]

    assert outputs[-1] == (0.0, 0.0)
    assert node.nav_context["calibration_active"]
    assert node.nav_context["calib_target_heading"] == -6.8
    warnings = [c.args[0] for c in node.get_logger().warn.call_args_list]
    assert any("持续5帧" in message for message in warnings)
    assert all("持续0帧" not in message for message in warnings)


def test_initial_completion_does_not_compare_new_target_to_old_minimum():
    node, ns = _make_node()
    node.current_waypoint_idx = 0
    node.imu_yaw = -6.8
    node.calculate_lateral_error.return_value = 0.0
    node.nav_context.update(
        nav_state=ns["NavState"].INITIAL_MOVE,
        force_bearing_mode=True, force_bearing_min_distance=0.04,
        distance_increase_count=9, force_bearing_recalib_count=3,
    )
    generator = node.multi_waypoint_nav_generator()

    left, right = next(generator)

    assert node.current_waypoint_idx == 1
    assert node.nav_context["nav_state"] == ns["NavState"].WAYPOINT_MOVE
    assert not node.nav_context["force_bearing_mode"]
    assert node.nav_context["force_bearing_min_distance"] == float("inf")
    assert node.nav_context["distance_increase_count"] == 0
    assert node.nav_context["force_bearing_recalib_count"] == 0
    assert left < 0 < right


def test_force_bearing_progress_prevents_false_fourth_alignment_stop():
    node, ns = _make_node()
    node.imu_yaw = 0.0
    node.calculate_bearing.return_value = 30.0
    node.calc_distance_to_waypoint.return_value = 1.0
    node.nav_context.update(
        force_bearing_mode=True, force_bearing_progress_distance=1.5,
        force_bearing_recalib_count=3,
    )

    output = next(node.multi_waypoint_nav_generator())

    assert output == (3.0, 3.0)
    assert node.nav_context["nav_state"] == ns["NavState"].WAYPOINT_MOVE
    assert node.nav_context["force_bearing_recalib_count"] == 1


def test_force_bearing_without_progress_retains_fourth_alignment_stop():
    node, ns = _make_node()
    node.imu_yaw = 0.0
    node.calculate_bearing.return_value = 30.0
    node.calc_distance_to_waypoint.return_value = 1.0
    node.nav_context.update(
        force_bearing_mode=True, force_bearing_progress_distance=1.0,
        force_bearing_recalib_count=3,
    )

    assert next(node.multi_waypoint_nav_generator()) == (0.0, 0.0)
    assert node.nav_context["nav_state"] == ns["NavState"].PAUSE
    assert node.nav_context["pause_reason"] == "force_bearing_limit_cycle"
    node.publish_stop_speed.assert_called()


def test_arrival_stops_before_requesting_the_planned_right_angle_heading():
    node, ns = _make_node()
    node.calc_distance_to_waypoint.return_value = 0.05

    assert next(node.multi_waypoint_nav_generator()) == (0.0, 0.0)
    assert node.nav_context["nav_state"] == ns["NavState"].WAYPOINT_CALIB
    assert node.nav_context["calib_target_heading"] == pytest.approx(-98.28)
    node.calibrate_heading_at_waypoint.assert_called_once_with(pytest.approx(-98.28))


def test_real_backtracking_still_triggers_divergence_protection():
    node, _ = _make_node()
    node.nav_context["force_bearing_min_distance"] = 1.0

    assert not any(node._force_bearing_diverging(1.6) for _ in range(9))
    assert node._force_bearing_diverging(1.6)


@pytest.mark.parametrize("heading,lateral,expected", [
    (-16.1, -24.3, 0.0), (16.1, 24.3, 0.0),
    (0.0, -24.3, 0.0), (-40.4, -24.3, -16.1),
    (16.1, -24.3, 16.1), (-16.1, 24.3, -16.1),
    (float("nan"), -24.3, float("inf")),
])
def test_heading_guard_accepts_only_the_commanded_recovery_corridor(
    heading, lateral, expected,
):
    ns = _load_navigation()

    assert ns["calculate_uncommanded_heading_error"](heading, lateral) == pytest.approx(expected)


@pytest.mark.parametrize("disabled_setting", ["rtk_max_correction", "rtk_correction_ratio"])
def test_disabled_rtk_correction_does_not_allow_a_recovery_turn(disabled_setting):
    node, _ = _make_node()
    setattr(node, disabled_setting, 0.0)

    error = node._tracking_heading_guard_error(-16.1, (0, 0), (0, 0), (1, 0), 52, 7)

    assert error == -16.1


def test_centimeter_position_noise_does_not_clear_no_progress_count():
    node, _ = _make_node()
    node.nav_context.update(force_bearing_progress_distance=1.0, force_bearing_recalib_count=3)

    for distance in (0.99, 1.01, 0.98, 0.99):
        node._record_force_bearing_progress(distance)
        assert node.nav_context["force_bearing_recalib_count"] == 3
        assert node.nav_context["force_bearing_progress_distance"] == 1.0

    node._record_force_bearing_progress(0.89)
    assert node.nav_context["force_bearing_recalib_count"] == 0
    assert node.nav_context["force_bearing_progress_distance"] == 0.89
    node.nav_context["force_bearing_recalib_count"] = 1
    for distance in (0.91, 0.90, 0.88):
        node._record_force_bearing_progress(distance)
    assert node.nav_context["force_bearing_recalib_count"] == 1


@pytest.mark.parametrize("lateral,heading_error,expected_diff", [
    (-0.273, -10.8, -3.005), (-0.338, -0.4, -6.415),
])
def test_shared_geometry_preserves_logged_wheel_commands(lateral, heading_error, expected_diff):
    node, _ = _make_node()
    node.calculate_lateral_error.return_value = lateral

    left, right = node.stanley_steering_control(
        (0, 0), -heading_error, (0, 0), (1, 0), 0.0, 52.0, 7.0,
    )

    assert left == pytest.approx(-52.0 + expected_diff, abs=0.001)
    assert right == pytest.approx(52.0 + expected_diff, abs=0.001)


def test_normal_turn_advances_only_after_calibration_completes():
    node, ns = _make_node()
    node.calc_distance_to_waypoint.return_value = 0.05
    node.nav_context.update(
        force_bearing_mode=True, force_bearing_min_distance=0.05,
        force_bearing_progress_distance=0.16, force_bearing_recalib_count=3,
        distance_increase_count=9,
    )
    generator = node.multi_waypoint_nav_generator()

    assert next(generator) == (0.0, 0.0)  # 到点先停车。
    assert next(generator) == (3.0, 3.0)  # 原地旋转期间不切换目标。
    assert node.current_waypoint_idx == 1
    assert next(generator) == (0.0, 0.0)
    assert node.current_waypoint_idx == 1
    assert next(generator) == (0.0, 0.0)  # 校准成功才允许切换。
    assert node.current_waypoint_idx == 2
    assert node.nav_context["target_waypoint"] == node.waypoints[2]
    assert node.nav_context["nav_state"] == ns["NavState"].WAYPOINT_MOVE
    assert not node.nav_context["force_bearing_mode"]
    assert node.nav_context["force_bearing_min_distance"] == float("inf")
    assert node.nav_context["force_bearing_progress_distance"] == float("inf")
    assert node.nav_context["force_bearing_recalib_count"] == 0
    assert node.nav_context["distance_increase_count"] == 0


@pytest.mark.parametrize("yaw,expected_calibrations", [(9.3, 1), (-30.0, 2)])
def test_first_waypoint_uses_same_recovery_guard_after_initial_alignment(yaw, expected_calibrations):
    node, ns = _make_node()
    node.current_waypoint_idx = 0
    node.imu_initialized = True
    node.gps_update_sequence = 1
    node.get_clock = Mock()
    node.imu_yaw = yaw
    generator = ns["Navigation"].move_to_first_waypoint(node)

    assert next(generator) == (3.0, 3.0)  # 初始对准。
    assert next(generator) == (0.0, 0.0)
    outputs = [next(generator) for _ in range(5)]

    assert node._calibrate_with_boundary_retreat.call_count == expected_calibrations
    if expected_calibrations == 1:
        assert all(left < 0 < right for left, right in outputs)
        assert node.nav_context["waypoint_recalib_count"] == 0
    else:
        assert outputs[-1] == (3.0, 3.0)
        assert node.nav_context["waypoint_recalib_count"] == 1
