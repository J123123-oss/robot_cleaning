import ast
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
