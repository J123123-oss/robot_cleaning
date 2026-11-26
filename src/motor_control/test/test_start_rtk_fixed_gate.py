"""回归验证 START 启动前的 RTK 固定解门控。"""

import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).parents[1] / "motor_control" / "motor_control.py"


def _load_methods():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    node_class = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "MotorControlNode"
    )
    method_names = {"gps_callback", "wtrtk_callback", "switch_state"}
    methods = [
        item for item in node_class.body
        if isinstance(item, ast.FunctionDef) and item.name in method_names
    ]
    namespace = {
        "WTRTK": object,
        "NavSatFix": object,
        "String": SimpleNamespace,
        "time": SimpleNamespace(sleep=lambda _: None),
        "STATE_DICT": {"u": "START"},
        "ERROR_RTK_NOT_FIXED": 4,
        "UNLOADING_MIN_BATTERY": 90.0,
    }
    exec(
        compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace


def _make_controller(namespace):
    controller = SimpleNamespace(
        current_status="DISABLE",
        status_list=["DISABLE", "ENABLE", "START"],
        current_stall_fault=False,
        blocked_directions=set(),
        is_in_bin_process=False,
        current_control_mode="NORMAL",
        magnetic_test_unloading_active=False,
        _magnetic_unloading_request_pending=False,
        battery_remaining=100.0,
        cleaning_low_battery_threshold=90.0,
        low_battery_warning=False,
        rtk_error_code=0,
        rtk_position_status=0,
        wtrtk_position_status=0,
        rtk_orientation_status=0,
        rtk_position_data_valid=False,
        current_lon=0.0,
        current_lat=0.0,
        enable_brush_test=False,
        brush_use_kinco=False,
        _rtk_motion_deadline=None,
        rtk_left_speed=0.0,
        rtk_right_speed=0.0,
        _brush_test_wheel_hold=False,
        _magnetic_enabled_dock_count=0,
        complete_state=False,
        state_publish_timer=Mock(),
        direction_timer=None,
        get_logger=Mock(return_value=Mock()),
        publish_state=Mock(),
        _cancel_bin_process_timers=Mock(),
        _initialize_control_motors=Mock(return_value=True),
        create_timer=Mock(return_value=Mock()),
        handle_unloading_step=Mock(),
        state_pub=Mock(),
    )
    controller.wtrtk_callback = (
        lambda msg: namespace["wtrtk_callback"](controller, msg)
    )
    controller.gps_callback = (
        lambda msg: namespace["gps_callback"](controller, msg)
    )
    controller.switch_state = (
        lambda key: namespace["switch_state"](controller, key)
    )
    return controller


def _wtrtk_message(position_status, orientation_status, position_data_valid):
    return SimpleNamespace(
        angle_x=0.0,
        angle_y=0.0,
        position_status=position_status,
        fix_status=orientation_status,
        position_data_valid=position_data_valid,
    )


def _gps_message(position_status):
    return SimpleNamespace(
        status=SimpleNamespace(status=position_status),
        longitude=120.0,
        latitude=30.0,
    )


@pytest.mark.parametrize(
    "position_status,orientation_status,position_data_valid",
    [
        (5, 4, True),
        (4, 5, True),
        (4, 4, False),
    ],
)
def test_start_rejects_when_rtk_is_not_dual_fixed_or_invalid(
    position_status, orientation_status, position_data_valid
):
    namespace = _load_methods()
    controller = _make_controller(namespace)
    controller.wtrtk_callback(
        _wtrtk_message(position_status, orientation_status, position_data_valid)
    )

    controller.switch_state("u")

    assert controller.current_status == "DISABLE"
    assert controller.rtk_error_code & 4
    controller.publish_state.assert_called_once_with()
    controller._initialize_control_motors.assert_not_called()


def test_wtrtk_callback_caches_position_data_validity():
    namespace = _load_methods()
    controller = _make_controller(namespace)

    controller.wtrtk_callback(_wtrtk_message(4, 4, True))

    assert controller.rtk_position_status == 4
    assert controller.wtrtk_position_status == 4
    assert controller.rtk_orientation_status == 4
    assert controller.rtk_position_data_valid is True


def test_start_reaches_existing_path_for_valid_dual_fixed_rtk():
    namespace = _load_methods()
    controller = _make_controller(namespace)
    controller.wtrtk_callback(_wtrtk_message(4, 4, True))

    controller.switch_state("u")

    assert controller.current_status == "START"
    controller._initialize_control_motors.assert_called_once_with()
    controller.create_timer.assert_called()


def test_pending_magnetic_unloading_bypasses_start_rtk_gate():
    namespace = _load_methods()
    controller = _make_controller(namespace)
    controller._magnetic_unloading_request_pending = True
    controller.wtrtk_callback(_wtrtk_message(5, 5, False))

    controller.switch_state("u")

    assert controller.current_status == "START"
    controller._initialize_control_motors.assert_called_once_with()


def test_start_uses_wtrtk_position_status_after_stale_gps_update():
    namespace = _load_methods()
    controller = _make_controller(namespace)
    controller.wtrtk_callback(_wtrtk_message(5, 4, True))
    controller.gps_callback(_gps_message(4))

    controller.switch_state("u")

    assert controller.current_status == "DISABLE"
    assert controller.rtk_position_status == 4
    assert controller.wtrtk_position_status == 5
    assert controller.rtk_error_code & 4
    controller._initialize_control_motors.assert_not_called()
