# Copyright 2026

import ast
import math
import os
from bisect import bisect_right
from pathlib import Path
from types import SimpleNamespace
from typing import Optional, Tuple
from unittest.mock import Mock

import pytest


RTK_SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _function(tree, name):
    return next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


class MotionCommand(SimpleNamespace):
    DISABLE = 0
    HOLD_ZERO = 1
    RUN = 2


@pytest.fixture
def navigator():
    tree = ast.parse(RTK_SOURCE_PATH.read_text(encoding="utf-8"))
    node_class = next(item for item in tree.body
                      if isinstance(item, ast.ClassDef) and item.name == "RTKNavControlNode")
    method_names = {
        "_brush_interval_active", "_motion_brush_action", "_publish_motor_speed",
        "publish_stop_speed", "publish_brush_speed", "check_and_control_brush",
        "get_target_waypoint", "finish_navigation_task", "load_waypoints_from_file",
        "resume_progress_callback", "_apply_skip_to_area", "_publish_imu_odom_fallback_speed",
        "_get_motion_direction_from_speed",
    }
    node_class.bases = []
    node_class.body = [item for item in node_class.body
                       if isinstance(item, ast.FunctionDef) and item.name in method_names]
    body = [item for item in tree.body
            if isinstance(item, ast.Assign)
            or isinstance(item, ast.ClassDef)
            and item.name in {"ControlMode", "NavState"}]
    namespace = {
        "Optional": Optional, "Tuple": Tuple, "String": SimpleNamespace,
        "Vector3": SimpleNamespace, "Float32": SimpleNamespace,
        "RtkMotionCommand": MotionCommand, "bisect_right": bisect_right, "math": math, "os": os,
        "calculate_heading_hold_wheel_speeds": Mock(return_value=(12.0, -10.0)),
        "normalize_heading_error": Mock(return_value=0.0),
    }
    exec(compile(ast.Module(body=[*body, node_class], type_ignores=[]),
                 str(RTK_SOURCE_PATH), "exec"), namespace)
    node = namespace["RTKNavControlNode"]()
    node.waypoints = [(120.0, 30.0, 0.0)] * 10
    node.waypoint_areas = ["work"] * 10
    node.loading_waypoint = (120.1, 30.0, 0.0)
    node.pending_next_path_file = None
    node.return_to_loading_added = False
    node.current_waypoint_idx = 3
    node.current_control_mode = "AUTO_CLEANING"
    node.nav_context = {"nav_state": "WAYPOINT_MOVE"}
    node.brush_start_indices = [2, 6]
    node.brush_stop_indices = [4, 8]
    node.brush_speed_rpm = 70.0
    node.brush_active = True
    node.get_logger = Mock()
    node.get_next_path_file = Mock(return_value=None)
    node.get_clock = Mock()
    node.get_clock().now().nanoseconds = 10_000_000_023
    node.get_clock().now().to_msg.return_value = SimpleNamespace(sec=10, nanosec=23)
    node.motor_speed_pub = Mock()
    node.motion_command_pub = Mock()
    node._boundary_stop_stamp_ns = None
    node._boundary_motion_inhibited = Mock(return_value=False)
    node._boundary_blocks_motion = Mock(return_value=False)
    node.rtk_error_code = 0
    node.update_rtk_error_status = Mock()
    node.geofence_latched = False
    node.geofence_outside_count = 0
    node.geofence_recovery_active = False
    node.geofence_recovery = None
    node.publish_geofence_recovery_ready = Mock()
    node._publish_geofence_recovery_active = Mock()
    namespace["time"] = Mock()
    return node, namespace


def last_message(publisher):
    return publisher.publish.call_args.args[0]


@pytest.mark.parametrize("index, legacy_on, action", [
    (0, False, MotionCommand.DISABLE),
    (2, True, MotionCommand.DISABLE),
    (3, True, MotionCommand.RUN),
    (4, False, MotionCommand.DISABLE),
    (6, True, MotionCommand.RUN),
    (9, False, MotionCommand.DISABLE),
])
def test_dual_publish_tracks_route_markers(navigator, index, legacy_on, action):
    node, _ = navigator
    node.current_waypoint_idx = index
    node.check_and_control_brush()
    node._publish_motor_speed(20.0, -18.0)

    legacy = last_message(node.motor_speed_pub)
    command = last_message(node.motion_command_pub)
    assert (legacy.x, legacy.y, legacy.z) == (20.0, -18.0, 70.0 if legacy_on else 0.0)
    assert (command.left_rpm, command.right_rpm) == (20.0, -18.0)
    assert command.stamp == SimpleNamespace(sec=10, nanosec=23)
    assert command.brush_action == action
    assert command.brush_rpm == (70.0 if action == MotionCommand.RUN else 0.0)


@pytest.mark.parametrize("state, previous_state, reason, action", [
    ("INITIAL_MOVE", None, None, MotionCommand.DISABLE),
    ("WAYPOINT_MOVE", None, None, MotionCommand.HOLD_ZERO),
    ("WAYPOINT_CALIB", None, None, MotionCommand.HOLD_ZERO),
    ("PAUSE", "INITIAL_MOVE", "rtk_not_fixed", MotionCommand.DISABLE),
    ("PAUSE", "WAYPOINT_MOVE", "rtk_not_fixed", MotionCommand.HOLD_ZERO),
    ("PAUSE", "WAYPOINT_CALIB", "imu_odom_fallback_limit_reached", MotionCommand.HOLD_ZERO),
    ("PAUSE", "WAYPOINT_MOVE", "auto_heading_gate_timeout", MotionCommand.DISABLE),
    ("PAUSE", "WAYPOINT_MOVE", "geofence_stop", MotionCommand.DISABLE),
    ("COMPLETED", None, None, MotionCommand.DISABLE),
])
def test_stop_action_matches_navigation_state(navigator, state, previous_state, reason, action):
    node, _ = navigator
    node.nav_context.update(nav_state=state, pre_pause_state=previous_state, pause_reason=reason)
    node.publish_stop_speed()

    assert vars(last_message(node.motor_speed_pub)) == dict(x=0.0, y=0.0, z=0.0)
    command = last_message(node.motion_command_pub)
    assert command.brush_action == action
    assert (command.left_rpm, command.right_rpm, command.brush_rpm) == (0.0, 0.0, 0.0)


def test_zero_yield_after_arrival_stop_does_not_restart_brush(navigator):
    node, _ = navigator
    node.publish_stop_speed()
    node._publish_motor_speed(0.0, 0.0)

    assert last_message(node.motor_speed_pub).z == 70.0
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.HOLD_ZERO


def test_initial_waypoint_brush_stays_disabled_through_final_calibration(navigator):
    node, _ = navigator
    node.brush_start_indices = [0]
    node.current_waypoint_idx = 0
    for state in ("INITIAL_MOVE", "WAYPOINT_CALIB"):
        node.nav_context["nav_state"] = state
        node._publish_motor_speed(12.0, -10.0)
        assert last_message(node.motion_command_pub).brush_action == MotionCommand.DISABLE
    node.current_waypoint_idx = 1
    node.nav_context["nav_state"] = "WAYPOINT_MOVE"
    node._publish_motor_speed(12.0, -10.0)
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.RUN


def test_active_interval_runs_during_moving_calibration_and_disables_outside_auto(navigator):
    node, _ = navigator
    node.nav_context["nav_state"] = "WAYPOINT_CALIB"
    node._publish_motor_speed(12.0, 12.0)
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.RUN

    node.current_control_mode = "NORMAL"
    node.publish_stop_speed()
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.DISABLE


def test_pause_suppresses_both_motion_topics_even_with_nonzero_generator_output(navigator):
    node, _ = navigator
    node.nav_context.update(nav_state="PAUSE", pre_pause_state="WAYPOINT_CALIB",
                            pause_reason="rtk_timeout")
    node._publish_motor_speed(12.0, -10.0)

    assert vars(last_message(node.motor_speed_pub)) == dict(x=0.0, y=0.0, z=0.0)
    command = last_message(node.motion_command_pub)
    assert command.brush_action == MotionCommand.HOLD_ZERO
    assert (command.left_rpm, command.right_rpm, command.brush_rpm) == (0.0, 0.0, 0.0)


def test_return_append_and_completion_publish_disable(navigator):
    node, _ = navigator
    node.brush_stop_indices.clear()
    node.current_waypoint_idx = len(node.waypoints)
    assert node.get_target_waypoint() == node.loading_waypoint
    node.motor_speed_pub.publish.assert_not_called()
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.DISABLE
    node._publish_motor_speed(12.0, -10.0)
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.DISABLE
    assert last_message(node.motion_command_pub).left_rpm == 12.0

    node.publish_nav_state = Mock()
    node.reset_nav_context = Mock()
    node._clear_progress_snapshot = Mock()
    node.update_cleaning_area = Mock()
    node.current_waypoint_idx = 3
    node.finish_navigation_task()
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.DISABLE


def test_fallback_uses_dual_publisher(navigator):
    node, _ = navigator
    node.imu_yaw = 90.0
    node.velocity_pub = Mock()
    node._publish_imu_odom_fallback_speed({
        "base_rpm": 20.0, "direction": "forward", "heading_target_deg": 90.0,
    })
    assert vars(last_message(node.motor_speed_pub)) == dict(x=12.0, y=-10.0, z=70.0)
    command = last_message(node.motion_command_pub)
    assert (command.left_rpm, command.right_rpm, command.brush_action) == (
        12.0, -10.0, MotionCommand.RUN,
    )


def test_route_jumps_rederive_interval_in_both_directions(navigator):
    node, _ = navigator
    node.current_gps = None
    for method in ("_clear_retreat_context", "_reset_boundary_correction",
                   "_reset_boundary_cycle_tracking", "clear_rtk_error_bits",
                   "_persist_progress", "publish_nav_state", "publish_current_route_id",
                   "update_cleaning_area"):
        setattr(node, method, Mock())

    for index, active in [(7, True), (9, False), (3, True), (1, False), (6, True)]:
        node._apply_skip_to_area(index, "work", "work", "exact")
        assert node.nav_context["brush_active"] is active
        node._publish_motor_speed(12.0, -10.0)
        assert last_message(node.motion_command_pub).brush_action == (
            MotionCommand.RUN if active else MotionCommand.DISABLE
        )
    assert node.brush_start_indices == [2, 6]
    assert node.brush_stop_indices == [4, 8]


def test_progress_restore_loads_markers_from_the_saved_route(navigator, tmp_path):
    node, namespace = navigator
    route = tmp_path / "saved.txt"
    route.write_text("header\n0,120,30,0\n#start\n1,120,30,0\n2,120,30,0\n"
                     "#stop\n3,120,30,0\n", encoding="utf-8")
    node.rtk_path_file = str(tmp_path / "different.txt")
    node.progress_path = str(tmp_path / "progress.json")
    node.enable_geofence = False
    node.waiting_for_next_unloading = False
    node.last_state = "DISABLE"
    node.current_control_mode = "NORMAL"
    for method in ("_reset_ultrasonic_suppression_runtime", "load_geofence_sidecar",
                   "update_cleaning_area", "_persist_progress", "publish_nav_state",
                   "publish_current_route_id"):
        setattr(node, method, Mock())
    namespace["load_progress"] = Mock(return_value={
        "route_path": str(route), "route_id": "saved",
        "waypoint_idx": 2, "nav_state": "WAYPOINT_MOVE",
    })

    node.resume_progress_callback(SimpleNamespace(data="RESUME"))

    assert node.current_waypoint_idx == 2
    assert node.nav_context["brush_active"] is True
    assert node.brush_start_indices == [1]
    assert node.brush_stop_indices == [3]
    node.motion_command_pub.publish.assert_not_called()
    node.current_control_mode = "AUTO_CLEANING"
    node._publish_motor_speed(12.0, -10.0)
    assert last_message(node.motion_command_pub).brush_action == MotionCommand.RUN


@pytest.mark.parametrize("saved_state", ["INITIAL_MOVE", "WAYPOINT_CALIB", "PAUSE"])
def test_progress_restore_rejoins_current_waypoint_from_current_position(
    navigator, tmp_path, saved_state
):
    node, namespace = navigator
    route = tmp_path / "saved.txt"
    route.write_text("header\n0,120,30,0\n1,120.1,30,0\n", encoding="utf-8")
    node.rtk_path_file = str(route)
    node.progress_path = str(tmp_path / "progress.json")
    node.enable_geofence = False
    node.waiting_for_next_unloading = False
    node.last_state = "DISABLE"
    node.current_control_mode = "NORMAL"
    for method in ("_reset_ultrasonic_suppression_runtime", "load_geofence_sidecar",
                   "update_cleaning_area", "_persist_progress", "publish_nav_state",
                   "publish_current_route_id"):
        setattr(node, method, Mock())
    namespace["load_progress"] = Mock(return_value={
        "route_path": str(route), "route_id": "saved",
        "waypoint_idx": 0, "nav_state": saved_state,
    })

    node.resume_progress_callback(SimpleNamespace(data="RESUME"))

    assert node.current_waypoint_idx == 0
    assert node.nav_context["target_waypoint"] == node.waypoints[0]
    assert node.nav_context["nav_state"] == "WAYPOINT_MOVE"
    assert node.nav_running is False
    node.motion_command_pub.publish.assert_not_called()


@pytest.mark.parametrize("motor_state", ["DISABLE", "ENABLE"])
def test_progress_resume_is_allowed_only_in_disable_or_enable(
    navigator, tmp_path, motor_state
):
    node, namespace = navigator
    route = tmp_path / "saved.txt"
    route.write_text("header\n0,120,30,0\n1,120.1,30,0\n", encoding="utf-8")
    node.rtk_path_file = str(route)
    node.progress_path = str(tmp_path / "progress.json")
    node.enable_geofence = False
    node.waiting_for_next_unloading = False
    node.last_state = motor_state
    node.current_control_mode = "NORMAL"
    for method in ("_reset_ultrasonic_suppression_runtime", "load_geofence_sidecar",
                   "update_cleaning_area", "_persist_progress", "publish_nav_state",
                   "publish_current_route_id"):
        setattr(node, method, Mock())
    namespace["load_progress"] = Mock(return_value={
        "route_path": str(route), "route_id": "saved",
        "waypoint_idx": 0, "nav_state": "WAYPOINT_MOVE",
    })

    node.resume_progress_callback(SimpleNamespace(data="RESUME"))

    namespace["load_progress"].assert_called_once_with(node.progress_path)
    assert node.current_waypoint_idx == 0
    assert node.nav_context["nav_state"] == "WAYPOINT_MOVE"


@pytest.mark.parametrize(
    "motor_state,current_control_mode",
    [
        ("HOLD", "NORMAL"),
        ("START", "NORMAL"),
        ("LOADING", "NORMAL"),
        ("UNLOADING", "NORMAL"),
        ("AUTO_CLEANING", "AUTO_CLEANING"),
        (None, "NORMAL"),
    ],
)
def test_progress_resume_is_rejected_outside_disable_or_enable(
    navigator, tmp_path, motor_state, current_control_mode
):
    node, namespace = navigator
    node.progress_path = str(tmp_path / "progress.json")
    node.last_state = motor_state
    node.current_control_mode = current_control_mode
    node.waiting_for_next_unloading = False
    namespace["load_progress"] = Mock()

    node.resume_progress_callback(SimpleNamespace(data="RESUME"))

    namespace["load_progress"].assert_not_called()
    assert node.current_waypoint_idx == 3
    node.get_logger().warning.assert_not_called()


def test_publisher_depth_and_all_speed_paths_share_one_entrance():
    tree = ast.parse(RTK_SOURCE_PATH.read_text(encoding="utf-8"))
    init = ast.unparse(_function(tree, "__init__"))
    assert "create_publisher(RtkMotionCommand, '/rtk/motion_command', 1)" in init
    for publisher in ("motor_speed_pub", "motion_command_pub"):
        methods = [
            method.name for method in ast.walk(tree) if isinstance(method, ast.FunctionDef)
            and f"self.{publisher}.publish(" in ast.unparse(method)
        ]
        assert methods == ["_publish_motor_speed"]
