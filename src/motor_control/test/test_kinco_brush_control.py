import ast
import json
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from motor_control.kinco_brush_control import BrushAction, KincoBrushControl


def wait_until(predicate):
    deadline = time.monotonic() + 2.0
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(0.005)


@pytest.fixture(params=[(3,), (3, 4)])
def brush(request):
    driver = Mock()
    driver.read_motor_status_word.return_value = 0x0021
    driver.read_brush_velocity.return_value = 0.0
    controller = KincoBrushControl(driver, request.param, True, lambda source: True)
    try:
        yield driver, controller, request.param
    finally:
        controller.close()


def test_pause_holds_enable_then_resume_and_disable(brush):
    driver, controller, motor_ids = brush
    controller.request(BrushAction.HOLD_ZERO)
    controller.wait_idle()
    driver.motor_enable.assert_not_called()
    controller.request(BrushAction.RUN, 90, source="rtk")
    wait_until(lambda: controller.speed == 90)
    controller.request(BrushAction.HOLD_ZERO)
    wait_until(lambda: controller.speed == 0)
    driver.motor_disable.assert_not_called()
    controller.request(BrushAction.RUN, 90, source="rtk")
    wait_until(lambda: controller.speed == 90)
    assert driver.motor_enable.call_count == len(motor_ids)
    controller.request(BrushAction.DISABLE)
    controller.wait_idle()
    assert driver.motor_disable.call_count == len(motor_ids)
    assert driver.read_brush_velocity.call_count == len(motor_ids)


def test_stop_during_enable_cancels_nonzero_speed(brush):
    driver, controller, _ = brush
    enabling = threading.Event()
    release = threading.Event()

    def enable(motor_id):
        enabling.set()
        return release.wait(1.0)

    driver.motor_enable.side_effect = enable
    controller.request(BrushAction.RUN, 90, source="rtk")
    try:
        assert enabling.wait(1.0)
        assert controller.wheel_motion_blocked
        controller.request(BrushAction.DISABLE)
    finally:
        release.set()
    controller.wait_idle()
    assert all(call.args[1] == 0 for call in driver.motor_set_speed.call_args_list)


def test_queued_nonzero_frame_is_cancelled_when_stop_arrives(brush):
    driver, controller, _ = brush
    queued = threading.Event()
    release = threading.Event()
    sent = []

    def write(motor_id, speed, *, send_allowed=None):
        if send_allowed is not None:
            queued.set()
            assert release.wait(1.0)
            if not send_allowed():
                return False
        sent.append((motor_id, speed))
        return True

    driver.motor_set_speed.side_effect = write
    controller.request(BrushAction.RUN, 90, source="rtk")
    try:
        assert queued.wait(1.0)
        controller.request(BrushAction.HOLD_ZERO)
    finally:
        release.set()
    controller.wait_idle()
    assert sent and all(speed == 0 for _, speed in sent)
    assert controller.fault is None
    driver.motor_disable.assert_not_called()


def test_partial_failure_disables_every_brush_and_latches_fault(brush):
    driver, controller, motor_ids = brush
    driver.motor_enable.side_effect = lambda motor_id: motor_id != motor_ids[-1]
    controller.request(BrushAction.RUN, 90, source="rtk")
    wait_until(lambda: controller.fault is not None)
    controller.request(BrushAction.RUN, 90, source="rtk")
    controller.wait_idle()
    assert driver.motor_enable.call_count == len(motor_ids)
    assert driver.motor_disable.call_count == len(motor_ids)
    assert all(call.args[1] == 0 for call in driver.motor_set_speed.call_args_list)


def test_stop_feedback_timeout_still_disables_every_brush(brush):
    driver, controller, motor_ids = brush
    driver.read_brush_velocity.return_value = None
    controller.request(BrushAction.RUN, 90, source="rtk")
    wait_until(lambda: controller.speed == 90)
    controller.request(BrushAction.DISABLE)
    wait_until(lambda: controller.fault is not None)
    assert driver.motor_disable.call_count == len(motor_ids)


def test_command_expiry_disables_without_a_new_message(brush):
    driver, controller, motor_ids = brush
    controller.request(BrushAction.RUN, 90, source="rtk", valid_for=0.15)
    wait_until(lambda: controller.speed == 90)
    wait_until(lambda: driver.motor_disable.call_count == len(motor_ids))
    assert controller.speed == 0


def test_wheels_are_blocked_until_brush_disable_is_confirmed(brush):
    driver, controller, _ = brush
    confirming = threading.Event()
    release = threading.Event()

    def wait_status(*args):
        confirming.set()
        return release.wait(1.0)

    driver.wait_motor_status.side_effect = wait_status
    controller.request(BrushAction.RUN, 90, source="rtk")
    wait_until(lambda: controller.speed == 90)
    controller.request(BrushAction.DISABLE)
    try:
        assert confirming.wait(1.0)
        assert controller.wheel_motion_blocked
    finally:
        release.set()
    controller.wait_idle()
    assert not controller.wheel_motion_blocked


@pytest.mark.parametrize("paused, action, expected", [
    (False, BrushAction.RUN, (14.0, -15.0, BrushAction.RUN, 95.0)),
    (True, BrushAction.RUN, (0.0, 0.0, BrushAction.HOLD_ZERO, 0.0)),
    (True, BrushAction.DISABLE, (0.0, 0.0, BrushAction.DISABLE, 0.0)),
    (False, BrushAction.DISABLE, (14.0, -15.0, BrushAction.DISABLE, 0.0)),
])
def test_motion_callback_validates_freshness_and_keeps_pause_precedence(paused, action, expected):
    path = Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    method = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "rtk_motion_callback"
    )
    namespace = dict(
        RtkMotionCommand=SimpleNamespace, BrushAction=BrushAction, math=math, time=time,
        RTK_MOTION_MAX_AGE=0.5, MIN_SPEED=-60.0, MAX_SPEED=60.0,
        limit_kinco_brush_speed=lambda speed, limit: min(speed, limit),
    )
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
    callback = namespace["rtk_motion_callback"]
    controller = SimpleNamespace(
        current_status="AUTO_CLEANING", current_control_mode="AUTO_CLEANING",
        web_control_active=False, rc_control=False, is_in_bin_process=False,
        current_stall_fault=False, geofence_fault_latched=False,
        _rtk_motion_stamp=-1, _rtk_auto_pause=paused,
        get_clock=Mock(), _kinco_brush=Mock(), kinco_brush_speed_rpm=95.0,
    )
    controller.get_clock().now().nanoseconds = 10_100_000_000
    command = SimpleNamespace(
        stamp=SimpleNamespace(sec=9, nanosec=0),
        left_rpm=14.0, right_rpm=-15.0, brush_rpm=120.0, brush_action=action,
    )
    callback(controller, command)
    controller._kinco_brush.request.assert_not_called()
    command.stamp.sec = 10
    callback(controller, command)
    callback(controller, command)
    assert controller._kinco_brush.request.call_count == 1
    assert (controller.rtk_left_speed, controller.rtk_right_speed,
            *controller._kinco_brush.request.call_args.args) == expected


@pytest.mark.parametrize("overrides", [
    {"enable_brush_test": False},
    {"_brush_test_armed": False, "current_status": "FORWARD"},
    {"current_status": "HOLD"},
    {"current_status": "DISABLE"},
    {"current_control_mode": "AUTO_CLEANING"},
    {"rc_control": True, "current_status": "RC_ENABLE", "rc_previous_status": "DISABLE"},
    {"rc_control": True, "current_status": "RC_ENABLE", "rc_previous_status": "HOLD"},
    {"current_status": "FORWARD", "blocked_directions": {"FORWARD"}},
    {"boundary_test_control_active": True},
    {"is_in_bin_process": True},
    {"current_stall_fault": True},
    {"geofence_fault_latched": True},
])
def test_enable_brush_test_uses_separate_kinco_source(overrides):
    controller = make_enable_brush_test_controller(**overrides)
    controller._service_enable_brush_test()
    assert not controller._kinco_brush_can_run("enable_test")
    controller._kinco_brush.request.assert_not_called()
    controller.set_motors_speed.assert_not_called()


def make_enable_brush_test_controller(**overrides):
    path = Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names = {
        "_enable_brush_test_can_run", "_service_enable_brush_test",
        "_kinco_brush_can_run", "set_brush_speed", "limit_kinco_brush_speed",
        "status_callback", "switch_state", "rtk_nav_status_callback", "timer_callback",
    }
    methods = [node for node in ast.walk(tree)
               if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = dict(
        BrushAction=BrushAction, math=math, time=time, json=json, String=SimpleNamespace,
        STATE_DICT={"h": "HOLD", "w": "FORWARD", "x": "ENABLE", "r": "AUTO_CLEANING"},
        MANUAL_TURN_SPEED_SCALE=0.25,
        KINCO_DEFAULT_BRUSH_SPEED_RPM=100.0, AIMOTOR_DEFAULT_BRUSH_SPEED_RPM=70.0,
        ENABLE_BRUSH_TEST_MAX_AGE=0.5,
    )
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), namespace)
    controller = SimpleNamespace(
        enable_brush_test=True, _enable_brush_test_active=False, _brush_test_armed=True,
        current_status="ENABLE", current_control_mode="NORMAL",
        web_control_active=False, rc_control=False, boundary_test_control_active=False,
        rc_previous_status="ENABLE", _brush_test_wheel_hold=False, blocked_directions=set(),
        is_in_bin_process=False, current_stall_fault=False, geofence_fault_latched=False,
        brush_motor_ids=[3], brush_use_kinco=True, kinco_brush_speed_rpm=25.0,
        web_rc_last_update=0.0, web_rc_timeout=1.0, _kinco_brush=Mock(),
        web_rc_left_speed=0.0, web_rc_right_speed=0.0,
        _wheel_command_blocked_by_sensors=Mock(return_value=False),
        set_motors_speed=Mock(), motor_ctrl=Mock(), brush_direction_mode="same",
        motor_auto_enable=True, get_logger=Mock(), publish_state=Mock(), state_pub=Mock(),
        _cancel_kinco_motion=Mock(), _check_auto_cleaning_entry=Mock(return_value=True),
        _clear_current_stall_fault=Mock(), _initialize_control_motors=Mock(),
        direction_timer=None, create_timer=Mock(), mqtt_control_speed=20.0,
        manual_drive_speed_scale=1.0, _last_rtk_nav_state_seq=-1,
        brush_speed=0.0, status_list=["ENABLE", "HOLD", "FORWARD", "AUTO_CLEANING"],
    )
    controller.motor_ctrl.motors = [{"id": 1}, {"id": 2}, {"id": 3}]
    for name in names - {"limit_kinco_brush_speed"}:
        setattr(controller, name, namespace[name].__get__(controller))
    vars(controller).update(overrides)
    return controller


@pytest.mark.parametrize("overrides", [
    {},
    {"rc_control": True, "current_status": "RC_ENABLE"},
    {"web_control_active": True, "current_control_mode": "WEB_REMOTE"},
    {"current_status": "FORWARD"},
    {"current_status": "HOLD", "_brush_test_wheel_hold": True},
])
def test_manual_wheel_control_keeps_configured_brush(overrides):
    controller = make_enable_brush_test_controller(**overrides)
    assert controller._service_enable_brush_test()
    assert controller._kinco_brush_can_run("enable_test")
    controller.set_motors_speed.assert_not_called()
    controller._kinco_brush.request.assert_called_once_with(
        BrushAction.RUN, 25.0, source="enable_test", valid_for=0.5,
    )
    controller.set_brush_speed(25.0)
    controller._kinco_brush.request.assert_called_with(BrushAction.DISABLE)

    controller.brush_use_kinco = False
    controller.motor_ctrl.motor_set_speed.return_value = True
    controller._service_enable_brush_test()
    controller.motor_ctrl.motor_set_speed.assert_called_with(3, 70.0)
    controller.current_control_mode = "AUTO_CLEANING"
    controller._service_enable_brush_test()
    controller.motor_ctrl.motor_set_speed.assert_called_with(3, 0.0)


def test_mqtt_remote_and_wheel_hold_do_not_cancel_brush_until_explicit_stop():
    controller = make_enable_brush_test_controller()
    assert controller._service_enable_brush_test()

    for command, expected_status in (
        ("RC_ENABLE", "RC_ENABLE"), ("RC_DISABLE", "ENABLE"),
        ("RC_ENABLE", "RC_ENABLE"), ("FORWARD", "FORWARD"), ("HOLD", "HOLD"),
        ("FORWARD", "FORWARD"), ("HOLD", "HOLD"),
    ):
        controller.status_callback(SimpleNamespace(data=json.dumps({"command": command})))
        assert controller.current_status == expected_status
        assert controller._service_enable_brush_test()

    controller._cancel_kinco_motion.assert_not_called()
    controller._initialize_control_motors.assert_not_called()
    assert all(call.args[0] == BrushAction.RUN for call in controller._kinco_brush.request.call_args_list)
    assert all(call.args[0] != 3 for call in controller.motor_ctrl.motor_set_speed.call_args_list)

    pause = SimpleNamespace(data=json.dumps({"nav_state": "PAUSE", "seq": 1, "auto_resume": True}))
    controller.rtk_nav_status_callback(pause)
    assert controller._kinco_brush.request.call_args.args[0] == BrushAction.RUN

    controller.switch_state("h")
    assert not controller._service_enable_brush_test()
    controller._cancel_kinco_motion.assert_called_once()
    controller._kinco_brush.request.assert_called_with(BrushAction.DISABLE)


def test_rtk_mode_ends_manual_brush_test():
    controller = make_enable_brush_test_controller()
    assert controller._service_enable_brush_test()
    controller.switch_state("r")
    assert controller.current_control_mode == "AUTO_CLEANING"
    assert not controller._service_enable_brush_test()
    controller._cancel_kinco_motion.assert_called_once()


@pytest.mark.parametrize("web_active", [False, True])
@pytest.mark.parametrize("fresh", [False, True])
def test_remote_timer_controls_wheels_and_keeps_launch_brush_speed(web_active, fresh):
    controller = make_enable_brush_test_controller(
        web_control_active=web_active, rc_control=not web_active,
        current_status="ENABLE" if web_active else "RC_ENABLE",
        web_rc_last_update=time.time() if fresh else 0.0,
        web_rc_left_speed=-8.0, web_rc_right_speed=8.0, web_rc_brush_speed=0.0,
        current_left_speed=0.0, current_right_speed=0.0,
        _service_kinco_motion=Mock(), _check_current_protection=Mock(),
        is_laser_timeout=Mock(return_value=False), mode_pub=Mock(),
    )
    controller.timer_callback()

    expected_wheels = (-8.0, 8.0) if fresh else (0.0, 0.0)
    controller.set_motors_speed.assert_called_once_with(*expected_wheels)
    controller._kinco_brush.request.assert_called_once_with(
        BrushAction.RUN, 25.0, source="enable_test", valid_for=0.5,
    )
