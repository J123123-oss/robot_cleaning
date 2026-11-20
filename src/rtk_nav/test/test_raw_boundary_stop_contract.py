# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Generator, Optional, Tuple
from unittest.mock import Mock

import pytest


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"

class MotionCommand(SimpleNamespace):
    RUN = 2


@pytest.fixture
def boundary_node():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    source_class = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "RTKNavControlNode"
    )
    methods = {
        "_apply_travel_boundary", "_get_motion_direction_from_speed",
        "_raw_boundary_sensor_names", "_boundary_sensors_for_motion", "_blocked_boundary_actions",
        "_active_boundary_sensor_names", "_get_live_blocked_directions",
        "_is_retreat_turn_blocked", "_is_calibration_boundary_active",
        "_service_calibration_boundary", "_select_retreat_plan", "_select_escape_plan",
        "_retreat_to_waypoint", "_clear_retreat_context", "_start_boundary_correction",
        "get_boundary_correct_speed", "_reset_boundary_correction", "_reset_boundary_cycle_tracking",
        "_fallback_entry_block_reason", "_fallback_age", "_process_imu_odom_fallback",
        "_update_blocked_directions", "io_data_rtk_callback", "_boundary_motion_inhibited",
        "_boundary_chassis_stopped", "_stop_on_boundary_io_loss", "_publish_motor_speed",
        "_boundary_blocks_motion",
        "_reset_boundary_debounce", "drive_twist_callback", "rtk_timer_callback",
    }
    body = [item for item in source_class.body
            if isinstance(item, ast.FunctionDef) and item.name in methods]
    node_class = ast.ClassDef("BoundaryNode", [], [], body, [])
    constants = [item for item in tree.body if isinstance(item, ast.Assign)]
    classes = [item for item in tree.body if isinstance(item, ast.ClassDef)
               and item.name in {"NavState", "ControlMode", "BoundaryCorrectState",
                                 "RetreatPhase", "RetreatResult"}]
    clock = SimpleNamespace(monotonic=Mock(return_value=10.0))
    scope = {"Optional": Optional, "Generator": Generator, "Tuple": Tuple, "math": math,
             "time": clock, "UInt8": SimpleNamespace, "Vector3": SimpleNamespace, "TwistStamped": SimpleNamespace,
             "RtkMotionCommand": MotionCommand, "fallback_budget_exhausted": Mock(return_value=False)}
    exec(compile(ast.fix_missing_locations(ast.Module(constants + classes + [node_class], [])),
                 str(SOURCE_PATH), "exec"), scope)
    node = scope["BoundaryNode"]()
    node.mid_left = node.mid_right = node.back_left = node.back_right = False
    node.confirmed_sensors = set()
    node.blocked_directions = set()
    node.clock = clock
    node.imu_yaw = 0.0
    node.boundary_cycle_count = 0
    node.boundary_cycle_sensor_signature = ()
    node._reset_boundary_correction()
    node.BOUNDARY_BACK_DURATION = 2.0
    node.correct_speed_scale = 0.4
    node.current_gps = (120.1, 30.1)
    node.nav_context = {}
    node.get_logger = Mock()
    node._is_ultrasonic_suppression_allowed = Mock(return_value=False)
    node.current_control_mode = "AUTO_CLEANING"
    node._last_motor_left = node._last_motor_right = 0.0
    node._boundary_stop_until = None
    node._boundary_stop_stamp_ns = None
    node._boundary_drive_stamp_ns = 0
    node._boundary_drive_velocity = (0.0, 0.0)
    node.last_io_time = 10.0
    node._arrival_confirmation = Mock()
    node.get_clock = Mock()
    node.get_clock().now.side_effect = lambda: SimpleNamespace(
        nanoseconds=int(clock.monotonic() * 1e9), to_msg=lambda: None,
    )
    node.motor_speed_pub = Mock()
    node.motion_command_pub = Mock()
    node._motion_brush_action = Mock(side_effect=lambda moving: 2 if moving else 1)
    node.brush_active = True
    node.brush_speed_rpm = 70.0
    node.publish_stop_speed = Mock(side_effect=lambda: node._publish_motor_speed(0, 0, stop=True))
    node._pause_boundary_retreat_timeout = Mock()
    return node


def sample_stopped_chassis(node, now):
    node.clock.monotonic.return_value = now
    node.last_io_time = now
    sec, nanosec = divmod(int(now * 1e9), 1_000_000_000)
    node.drive_twist_callback(SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        twist=SimpleNamespace(linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0)),
    ))


@pytest.mark.parametrize("speed,sensor,should_stop", [
    ((-10.0, 11.0), "mid_left", True),
    ((-10.0, 11.0), "back_left", False),
    ((10.0, -11.0), "back_right", True),
    ((10.0, -11.0), "mid_right", False),
])
def test_raw_io_stops_only_the_planned_translation(boundary_node, speed, sensor, should_stop):
    setattr(boundary_node, sensor, True)
    output = boundary_node._apply_travel_boundary(*speed)
    assert output == ((0.0, 0.0) if should_stop else speed)
    assert boundary_node.publish_stop_speed.called is should_stop
    assert not boundary_node.boundary_correct_locked


def test_confirmed_rear_does_not_mask_fresh_front_trigger(boundary_node):
    boundary_node.confirmed_sensors = {"back_left"}
    boundary_node.mid_left = boundary_node.back_left = True
    assert boundary_node._apply_travel_boundary(-10.0, 10.0) == (0.0, 0.0)
    boundary_node.publish_stop_speed.assert_called_once()
    assert not boundary_node.boundary_correct_locked


def test_confirmed_front_starts_straight_escape_until_rear_triggers(boundary_node):
    boundary_node.mid_left = True
    boundary_node.confirmed_sensors = {"mid_left"}
    assert boundary_node._apply_travel_boundary(-10.0, 10.0) == (0.0, 0.0)
    assert boundary_node.boundary_correct_locked
    assert boundary_node.boundary_correct_start_time is None
    assert boundary_node.get_boundary_correct_speed() == (0.0, 0.0)
    assert boundary_node.boundary_correct_start_time is None

    boundary_node.clock.monotonic.return_value = 11.0
    boundary_node.last_io_time = 11.0
    boundary_node._boundary_drive_stamp_ns = 11_000_000_000
    boundary_node._boundary_drive_velocity = (0.0, 0.2)
    assert boundary_node.get_boundary_correct_speed() == (0.0, 0.0)
    boundary_node._boundary_drive_velocity = (0.0, 0.0)
    left, right = boundary_node.get_boundary_correct_speed()
    assert left > 0 > right
    assert boundary_node.boundary_correct_start_time == 11.0

    boundary_node.back_right = True
    assert boundary_node.get_boundary_correct_speed() == (0.0, 0.0)
    boundary_node._pause_boundary_retreat_timeout.assert_called_once()


def test_ordinary_correction_waits_for_all_sensors_before_return_turn(boundary_node):
    node = boundary_node
    node.mid_right = True
    node.confirmed_sensors = {"mid_right"}
    node._apply_travel_boundary(-10.0, 10.0)
    sample_stopped_chassis(node, 10.1)
    node.get_boundary_correct_speed()
    sample_stopped_chassis(node, 12.11)
    assert node.get_boundary_correct_speed() == (0.0, 0.0)
    node.get_heading_error = lambda _: -15.0
    assert node.get_boundary_correct_speed() == (0.0, 0.0)

    node.current_control_mode = "AUTO_CLEANING"
    node.io_data_rtk_callback(SimpleNamespace(data=0b1111))
    sample_stopped_chassis(node, 12.32)
    node.io_data_rtk_callback(SimpleNamespace(data=0b1111))
    node.get_adaptive_turn_speed = lambda _: 5.0
    node.get_speed_correction = lambda _: 0.0
    assert node.get_boundary_correct_speed() == (5.0, 5.0)


@pytest.mark.parametrize("rear_trigger_during_turn", [False, True])
def test_geometric_retreat_uses_straight_escape_when_turn_is_blocked(
    boundary_node, rear_trigger_during_turn
):
    boundary_node.back_right = not rear_trigger_during_turn
    boundary_node.calc_distance_to_waypoint = lambda _: 1.0
    boundary_node.calculate_bearing = lambda *_: 15.0
    boundary_node.normalize_angle = lambda value: (value + 180) % 360 - 180
    boundary_node.get_heading_error = lambda target: target
    boundary_node.calibrate_heading_at_waypoint = Mock(return_value=iter([(4.0, 4.0)]))
    retreat = boundary_node._retreat_to_waypoint((120.0, 30.0, 90.0))
    assert next(retreat) == (0.0, 0.0)
    boundary_node.publish_stop_speed()
    sample_stopped_chassis(boundary_node, 10.1)

    if rear_trigger_during_turn:
        assert next(retreat) == (4.0, 4.0)
        boundary_node._publish_motor_speed(4.0, 4.0)
        boundary_node.back_right = True
    assert next(retreat) == (0.0, 0.0)
    boundary_node.publish_stop_speed()
    if not rear_trigger_during_turn:
        boundary_node.calibrate_heading_at_waypoint.assert_not_called()
    assert boundary_node.nav_context["retreat_escape_start_time"] is None
    sample_stopped_chassis(boundary_node, 10.2)
    left, right = next(retreat)
    assert left < 0 < right
    boundary_node.mid_left = True
    assert next(retreat) == (0.0, 0.0)
    with pytest.raises(StopIteration) as completed:
        next(retreat)
    assert completed.value.value == "p1_no_safe_candidate"


def test_fallback_filters_sensors_on_entry_callback_and_timer(boundary_node):
    node = boundary_node
    node.current_control_mode = node.last_state = "AUTO_CLEANING"
    node.nav_context = {"nav_state": "WAYPOINT_MOVE", "target_waypoint": (120.0, 30.0, 0.0)}
    node.current_waypoint_idx = 1
    node.waiting_for_next_unloading = False
    node.stanley_path_direction = 0.0
    node.odom_linear_velocity_mps = 0.1
    node.last_wtrtk_time = node.last_odom_time = node.last_io_time = 10.0
    node._last_motor_left, node._last_motor_right = (-10.0, 10.0)
    node._fallback_candidate = {"anchor_xy": (0.0, 0.0), "estimator_valid": True}
    node.enable_imu_odom_fallback = True
    node._fallback_lockout_reason = None
    node._pause_imu_odom_fallback = Mock()
    node._advance_imu_odom_recovery = Mock()
    node._fallback_segment_end_reached = Mock(return_value=False)
    node._publish_imu_odom_fallback_speed = Mock()
    node._fallback_episode = {
        "active": True, "direction": "FORWARD", "estimator_valid": True,
        "waypoint_idx": 1, "target_waypoint": node.nav_context["target_waypoint"],
        "path_direction_deg": 0.0, "fallback_elapsed_sec": 0.0, "fallback_distance_m": 0.0,
    }
    node.io_data_rtk_callback(SimpleNamespace(data=0b1011))  # 仅后左触发。
    assert node.sensors_status == 0b0100
    assert node._fallback_entry_block_reason(10.0) is None
    assert node._process_imu_odom_fallback(10.0)
    node._pause_imu_odom_fallback.assert_not_called()
    node._publish_imu_odom_fallback_speed.assert_called_once()

    node.io_data_rtk_callback(SimpleNamespace(data=0b1010))  # 新触发前左。
    assert node._fallback_entry_block_reason(10.0) == "current_motion_not_straight"
    assert node.motion_command_pub.publish.call_args.args[0].left_rpm == 0.0
    node._pause_imu_odom_fallback.assert_called_once_with(
        "imu_odom_fallback_boundary_blocked", 10.0, lockout=True
    )
    node._pause_imu_odom_fallback.reset_mock()
    assert node._process_imu_odom_fallback(10.0)
    node._pause_imu_odom_fallback.assert_called_once_with(
        "imu_odom_fallback_boundary_blocked", 10.0, lockout=True
    )
    node._publish_imu_odom_fallback_speed.assert_called_once()


@pytest.mark.parametrize("turn_error", [-15.0, 15.0])
@pytest.mark.parametrize("sensor", ["mid_left", "mid_right", "back_left", "back_right"])
def test_retreat_turn_blocks_on_each_of_the_four_sensors(boundary_node, sensor, turn_error):
    setattr(boundary_node, sensor, True)
    boundary_node.get_heading_error = lambda _: turn_error
    assert boundary_node._is_retreat_turn_blocked(0.0)


def test_bridge_suppression_still_applies_to_new_move_and_calibration_io(boundary_node):
    boundary_node.mid_left = True
    boundary_node._is_ultrasonic_suppression_allowed.return_value = True
    assert boundary_node._apply_travel_boundary(-10.0, 10.0) == (-10.0, 10.0)
    assert not boundary_node._is_calibration_boundary_active()
    boundary_node.confirmed_sensors = {"mid_left"}
    assert boundary_node._is_calibration_boundary_active()


def test_bridge_suppression_allows_new_raw_hit_without_disabling_io_watchdog(boundary_node):
    node = boundary_node
    node._is_ultrasonic_suppression_allowed.return_value = True
    node._publish_motor_speed(-10, 10)
    node.io_data_rtk_callback(SimpleNamespace(data=0b1110))
    node.publish_stop_speed.assert_not_called()
    node.clock.monotonic.return_value = 10.3
    node._publish_motor_speed(-10, 10)
    assert node.motor_speed_pub.publish.call_args.args[0].x == 0


def test_io_timeout_stops_without_advancing_navigation_generator(boundary_node):
    node = boundary_node
    node._fallback_diagnostics_active = Mock(return_value=False)
    node._last_nav_context_publish = 10.0
    node.update_cleaning_area = Mock()
    node.ultrasonic_suppression_active = False
    node.geofence_latched = False
    node._position_guard = SimpleNamespace(blocked=False)
    node.enforce_geofence = Mock(return_value=False)
    node.update_rtk_error_status = Mock()
    node.rtk_error_code = 0
    node._is_manual_intervention_pause = Mock(return_value=False)
    node.waiting_for_next_unloading = False
    node.enable_imu_odom_fallback = False
    node.multi_waypoint_generator = iter([(-10, 10)])
    node.nav_running = True
    node.clock.monotonic.return_value = 10.3
    node.rtk_timer_callback()
    node.publish_stop_speed.assert_called_once()
    assert next(node.multi_waypoint_generator) == (-10, 10)


@pytest.mark.parametrize("io_time", [None, 9.7])
def test_missing_or_stale_io_blocks_both_motion_topics(boundary_node, io_time):
    node = boundary_node
    node.last_io_time = io_time
    node._publish_motor_speed(-10, 10)
    legacy = node.motor_speed_pub.publish.call_args.args[0]
    stamped = node.motion_command_pub.publish.call_args.args[0]
    assert (legacy.x, legacy.y, legacy.z) == (0, 0, 0)
    assert (stamped.left_rpm, stamped.right_rpm, stamped.brush_rpm) == (0, 0, 0)


def test_raw_pulse_stops_immediately_and_survives_a_clear_frame(boundary_node):
    node = boundary_node
    node._publish_motor_speed(-10, 10)
    node.io_data_rtk_callback(SimpleNamespace(data=0b1110))
    node.publish_stop_speed.assert_called_once()
    assert node.motion_command_pub.publish.call_args.args[0].left_rpm == 0
    node.io_data_rtk_callback(SimpleNamespace(data=0b1111))
    node._publish_motor_speed(-10, 10)
    assert node.motor_speed_pub.publish.call_args.args[0].x == 0
    assert not node.boundary_correct_locked
    sample_stopped_chassis(node, 10.21)
    node._publish_motor_speed(-10, 10)
    assert node.motor_speed_pub.publish.call_args.args[0].x == -10


@pytest.mark.parametrize("speed,sensor", [
    ((-0.01, 20.0), "back_left"), ((20.0, -0.01), "mid_right"),
])
def test_tight_arc_checks_opposite_end(boundary_node, speed, sensor):
    setattr(boundary_node, sensor, True)
    assert boundary_node._apply_travel_boundary(*speed) == (0.0, 0.0)


def test_io_gap_during_escape_pauses_even_if_the_next_frame_is_clear(boundary_node):
    node = boundary_node
    node.mid_left = True
    node.confirmed_sensors = {"mid_left"}
    node._apply_travel_boundary(-10.0, 10.0)
    sample_stopped_chassis(node, 10.1)
    node._publish_motor_speed(*node.get_boundary_correct_speed())
    node.clock.monotonic.return_value = 10.4
    node.io_data_rtk_callback(SimpleNamespace(data=0b1111))
    assert not node.boundary_correct_locked
    node._pause_boundary_retreat_timeout.assert_called_once()
