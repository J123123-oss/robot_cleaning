"""Tests for brush motor topology configuration."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Tuple
from unittest.mock import Mock

import pytest


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "motor_control"
    / "motor_control.py"
)


def test_direction_selection_uses_kinco_parameters_only_for_configured_brushes():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    initializer = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                       and item.name == "__init__")
    assignment = next(
        item for item in initializer.body if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "motor_directions"
                for target in item.targets)
    )
    values = {
        "motor_direction_1": -1, "motor_direction_2": -1,
        "motor_direction_3": 1, "motor_direction_4": -1,
        "kinco_motor_direction_3": -1, "kinco_motor_direction_4": 1,
    }
    for use_kinco in (False, True):
        for motor_ids in ((1, 2), (1, 2, 3), (1, 2, 3, 4)):
            controller = SimpleNamespace(
                brush_use_kinco=use_kinco,
                get_parameter=lambda name: SimpleNamespace(value=values[name]),
            )
            namespace = {"self": controller, "motor_ids": motor_ids}
            exec(compile(ast.Module(body=[assignment], type_ignores=[]),
                         str(SOURCE_PATH), "exec"), namespace)
            assert namespace["motor_directions"] == {
                motor_id: values[
                    f"kinco_motor_direction_{motor_id}"
                    if use_kinco and motor_id in (3, 4)
                    else f"motor_direction_{motor_id}"
                ]
                for motor_id in motor_ids
            }


def test_kinco_direction_parameters_are_declared_for_both_brush_ids():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    declarations = [
        item for item in ast.walk(tree) if isinstance(item, ast.For)
        and any(isinstance(child, ast.Constant)
                and child.value == "kinco_motor_direction_"
                for child in ast.walk(item))
    ]
    assert len(declarations) == 1
    declared = {}
    namespace = {
        "self": SimpleNamespace(
            declare_parameter=lambda name, default: declared.update({name: default})
        ),
    }
    exec(compile(ast.Module(body=declarations, type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    assert declared == {"kinco_motor_direction_3": -1, "kinco_motor_direction_4": -1}


def test_startup_configures_kinco_without_enabling_brushes():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    methods = [
        item for item in node.body if isinstance(item, ast.FunctionDef)
        and item.name in (
            "auto_enable_startup_motors", "_configure_enabled_state",
            "_initialize_control_motors",
        )
    ]
    namespace = {}
    exec(compile(ast.Module(body=methods, type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    previous_state_timer = Mock()
    previous_direction_timer = Mock()
    controller = SimpleNamespace(
        motor_auto_enable=True, brush_use_kinco=True,
        _kinco_brush=None,
        current_status="ENABLE", complete_state=True,
        state_publish_timer=previous_state_timer,
        direction_timer=previous_direction_timer,
        _disable_publish_countdown=3,
        create_timer=Mock(return_value=Mock()),
        publish_state=Mock(),
        motor_ctrl=Mock(motor_ids=(1, 2, 3, 4)),
        get_logger=Mock(return_value=Mock()),
    )
    controller._configure_enabled_state = (
        lambda: namespace["_configure_enabled_state"](controller)
    )
    controller._initialize_control_motors = (
        lambda **options: namespace["_initialize_control_motors"](controller, **options)
    )
    controller.switch_state = Mock(
        side_effect=lambda key, *, urgent: setattr(controller, "current_status", "DISABLE")
    )
    controller.motor_ctrl.initialize_motors.return_value = True
    namespace["auto_enable_startup_motors"](controller)
    controller.motor_ctrl.initialize_motors.assert_called_once_with(
        motor_ids=(1, 2, 3, 4), enable_kinco=False
    )
    assert controller.current_status == "ENABLE"
    assert not controller.complete_state
    assert controller.direction_timer is None
    assert controller._disable_publish_countdown == 0
    previous_state_timer.cancel.assert_called_once()
    previous_direction_timer.cancel.assert_called_once()
    controller.create_timer.assert_called_once_with(0.5, controller.publish_state)
    controller.publish_state.assert_called_once_with()
    controller.motor_ctrl.motor_set_speed.assert_not_called()
    controller.motor_ctrl.motor_enable.assert_not_called()
    controller.create_timer.reset_mock()
    controller.publish_state.reset_mock()
    controller.motor_ctrl.initialize_motors.return_value = False
    namespace["auto_enable_startup_motors"](controller)
    controller.switch_state.assert_called_once_with("z", urgent=True)
    assert controller.current_status == "DISABLE"
    controller.create_timer.assert_not_called()
    controller.publish_state.assert_not_called()
    controller.motor_ctrl.reset_mock()
    controller.motor_auto_enable = False
    namespace["auto_enable_startup_motors"](controller)
    controller.motor_ctrl.initialize_motors.assert_not_called()
    controller.create_timer.assert_not_called()


def test_enable_state_settings_create_publisher_timer_when_none_exists():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == "_configure_enabled_state")
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    controller = SimpleNamespace(
        complete_state=True, direction_timer=None, state_publish_timer=None,
        _disable_publish_countdown=3, create_timer=Mock(return_value=Mock()),
        publish_state=Mock(), get_logger=Mock(return_value=Mock()),
    )

    namespace["_configure_enabled_state"](controller)

    assert not controller.complete_state
    assert controller._disable_publish_countdown == 0
    assert controller.state_publish_timer is controller.create_timer.return_value
    controller.create_timer.assert_called_once_with(0.5, controller.publish_state)


def test_fault_callback_does_not_reset_enabled_motors_for_stale_startup_faults():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == "motor_fault_callback")
    namespace = {"Float32MultiArray": SimpleNamespace}
    exec(compile(ast.Module(body=[method], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    controller = SimpleNamespace(
        motor_ctrl=Mock(motors=[{"id": motor_id} for motor_id in (1, 2, 3, 4)]),
        motor_fault_codes=[0, 0, 0, 0], current_status="ENABLE",
        get_logger=Mock(return_value=Mock()), publish_state=Mock(),
        switch_state=Mock(),
    )
    controller.motor_ctrl.read_motor_status_word.return_value = 0x4637

    namespace["motor_fault_callback"](
        controller, SimpleNamespace(data=[0.0, 0.0, 33024.0, 33024.0])
    )

    controller.motor_ctrl.motor_clear_fault.assert_not_called()
    controller.switch_state.assert_not_called()
    assert controller.motor_fault_codes == [0, 0, 0, 0]


def test_fault_callback_disables_on_live_fault_and_resets_only_faulted_nodes():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == "motor_fault_callback")
    namespace = {"Float32MultiArray": SimpleNamespace}
    exec(compile(ast.Module(body=[method], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    for status in (0x0008, None):
        controller = SimpleNamespace(
            motor_ctrl=Mock(motors=[{"id": motor_id} for motor_id in (1, 2, 3, 4)]),
            motor_fault_codes=[0, 0, 0, 0], current_status="ENABLE",
            get_logger=Mock(return_value=Mock()), publish_state=Mock(),
        )
        events = []
        controller.switch_state = lambda key, *, urgent: events.append(("state", key, urgent))
        controller.motor_ctrl.motor_clear_fault.side_effect = (
            lambda motor_id: events.append(("reset", motor_id)) or True
        )
        controller.motor_ctrl.read_motor_status_word.return_value = status

        namespace["motor_fault_callback"](
            controller, SimpleNamespace(data=[0.0, 0.0, 33024.0, 0.0])
        )

        assert events == ([("state", "z", True), ("reset", 3)]
                          if status is not None else [("state", "z", True)])
        assert controller.motor_fault_codes == [0, 0, 33024, 0]
        controller.publish_state.assert_called()


def test_manual_enable_applies_same_settings_without_starting_motors():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    methods = [
        item for item in node.body if isinstance(item, ast.FunctionDef)
        and item.name in ("switch_state", "_configure_enabled_state", "_initialize_control_motors")
    ]
    namespace = {
        "STATE_DICT": {"x": "ENABLE"}, "String": SimpleNamespace,
        "time": SimpleNamespace(sleep=Mock()),
    }
    exec(compile(ast.Module(body=methods, type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    previous_state_timer = Mock()
    previous_direction_timer = Mock()
    controller = SimpleNamespace(
        current_status="HOLD", status_list=["HOLD", "ENABLE"],
        brush_use_kinco=False, _kinco_brush=None,
        current_stall_fault=False, blocked_directions=set(), is_in_bin_process=False,
        complete_state=True, direction_timer=previous_direction_timer,
        state_publish_timer=previous_state_timer, _disable_publish_countdown=3,
        motor_ctrl=Mock(), state_pub=Mock(), publish_state=Mock(),
        create_timer=Mock(return_value=Mock()), get_logger=Mock(return_value=Mock()),
        _clear_current_stall_fault=Mock(), set_brush_speed=Mock(),
        set_motors_speed=Mock(),
    )
    controller._configure_enabled_state = (
        lambda: namespace["_configure_enabled_state"](controller)
    )
    controller._initialize_control_motors = (
        lambda **options: namespace["_initialize_control_motors"](controller, **options)
    )
    controller.motor_ctrl.initialize_motors.return_value = True

    namespace["switch_state"](controller, "x")

    assert controller.current_status == "ENABLE"
    assert not controller.complete_state
    assert controller.direction_timer is None
    assert controller._disable_publish_countdown == 0
    controller.motor_ctrl.initialize_motors.assert_called_once_with()
    previous_direction_timer.cancel.assert_called_once()
    previous_state_timer.cancel.assert_called_once()
    controller.create_timer.assert_called_once_with(0.5, controller.publish_state)
    controller.set_brush_speed.assert_not_called()
    controller.set_motors_speed.assert_not_called()
    assert controller.state_pub.publish.call_args.args[0].data == "ENABLE"


@pytest.mark.parametrize("brush_configured", [False, True])
def test_control_main_preserves_driver_shutdown_policy_on_normal_exit_only(brush_configured):
    from test_motor_driver_aimotor import _load_module

    implementation = _load_module().rclpy._test_implementation
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    main = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                and item.name == "main")
    external_shutdown = type("ExternalShutdownException", (Exception,), {})
    cases = (
        (KeyboardInterrupt(), False, False),
        (external_shutdown(), False, False),
        (RuntimeError("control failed"), False, True),
        (implementation.RCLError("publisher's context is invalid"), False, False),
        (implementation.InvalidHandle("destruction was requested"), False, False),
        (implementation.RCLError("publish failed"), True, True),
        (implementation.InvalidHandle("invalid handle"), True, True),
    )
    for exit_reason, context_active, should_disable in cases:
        controller = Mock(_kinco_brush=Mock() if brush_configured else None)
        controller.motor_ctrl.drive_motor_ids = (1, 2)
        ros = SimpleNamespace(
            init=Mock(), spin=Mock(side_effect=exit_reason),
            ok=Mock(return_value=context_active), shutdown=Mock(),
        )
        namespace = {
            "rclpy": ros, "MotorControlNode": Mock(return_value=controller),
            "_rclpy": implementation,
            "ExternalShutdownException": external_shutdown,
            "traceback": SimpleNamespace(format_exc=Mock(return_value="trace")),
        }
        exec(compile(ast.Module(body=[main], type_ignores=[]),
                     str(SOURCE_PATH), "exec"), namespace)

        namespace["main"]()

        controller.motor_ctrl.destroy_node.assert_called_once()
        controller.motor_ctrl.motor_disable.assert_not_called()
        if brush_configured:
            assert [call.args for call in controller.motor_ctrl.motor_set_speed.call_args_list] == [
                (1, 0.0), (2, 0.0)
            ]
            controller._kinco_brush.close.assert_called_once_with(urgent=should_disable)
        if should_disable:
            controller.motor_ctrl.stop_all_motors.assert_called_once_with()
        else:
            controller.motor_ctrl.stop_all_motors.assert_not_called()
        assert ros.shutdown.call_count == int(context_active)


def test_initial_state_is_enable_before_startup_initialization():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    initializer = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                       and item.name == "__init__")
    assignment = next(
        item for item in initializer.body if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Attribute) and target.attr == "current_status"
                for target in item.targets)
    )
    controller = SimpleNamespace(motor_auto_enable=True, status_list=["HOLD"])
    exec(compile(ast.Module(body=[assignment], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), {"self": controller})
    assert controller.current_status == "ENABLE"


def test_enabled_dock_detection_requires_three_consecutive_six_bit_samples():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == "_disable_if_docked_while_enabled")
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    controller = SimpleNamespace(
        enable_magnetic_navigation=True, current_status="ENABLE",
        magnetic_loading_end_active_bits=6, magnetic_loading_end_count_required=3,
        _magnetic_enabled_dock_count=0, finish_loading_process=Mock(),
    )
    detect = namespace["_disable_if_docked_while_enabled"]
    for mask in (0xFC, 0xFC, 0, 0xFC, 0xFC):
        assert not detect(controller, mask)
    controller.finish_loading_process.assert_not_called()
    assert detect(controller, 0xFC)
    controller.finish_loading_process.assert_called_once()
    controller.current_status = "START"
    assert not detect(controller, 0xFF)
    assert controller._magnetic_enabled_dock_count == 0


def test_confirmed_dock_reuses_zero_speed_disable_and_charging_flow():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == "finish_loading_process")
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    controller = SimpleNamespace(
        current_status="ENABLE", loading_timer=None,
        set_motors_speed=Mock(), get_logger=Mock(return_value=Mock()),
        _request_charge_if_docked=Mock(),
    )
    controller.switch_state = lambda key: setattr(controller, "current_status", "DISABLE")
    namespace["finish_loading_process"](controller)
    assert controller.current_status == "DISABLE"
    assert controller.complete_state
    controller.set_motors_speed.assert_called_once_with(0.0, 0.0)
    controller._request_charge_if_docked.assert_called_once()


def _load_brush_motor_ids_function():
    """Load the topology helper without requiring a ROS installation."""
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "brush_motor_ids_for_count"
    )
    namespace = {"Tuple": Tuple}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    return namespace["brush_motor_ids_for_count"]


def _load_kinco_speed_limiter():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "limit_kinco_brush_speed"
    )
    namespace = {"math": math}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    return namespace["limit_kinco_brush_speed"]


def test_brush_motor_count_maps_to_configured_node_ids():
    """Zero, one, and two brushes select the expected CANopen nodes."""
    brush_motor_ids_for_count = _load_brush_motor_ids_function()

    assert brush_motor_ids_for_count(0) == ()
    assert brush_motor_ids_for_count(1) == (3,)
    assert brush_motor_ids_for_count(2) == (3, 4)


def test_brush_motor_count_rejects_unsupported_values():
    """The topology contract rejects negative and larger brush counts."""
    brush_motor_ids_for_count = _load_brush_motor_ids_function()

    for value in (-1, 3):
        try:
            brush_motor_ids_for_count(value)
        except ValueError:
            pass
        else:
            raise AssertionError("unsupported brush count was accepted")


def test_kinco_brush_speed_limit_preserves_direction_and_zero_means_unlimited():
    limit_kinco_brush_speed = _load_kinco_speed_limiter()

    assert limit_kinco_brush_speed(25.0, 18.0) == 18.0
    assert limit_kinco_brush_speed(-25.0, 18.0) == -18.0
    assert limit_kinco_brush_speed(12.0, 18.0) == 12.0
    assert limit_kinco_brush_speed(-25.0, 0.0) == -25.0


def test_kinco_brush_speed_limit_rejects_invalid_values():
    limit_kinco_brush_speed = _load_kinco_speed_limiter()

    for speed, maximum in (("bad", 18.0), (10.0, -1.0), (10.0, "bad")):
        try:
            limit_kinco_brush_speed(speed, maximum)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid Kinco brush speed was accepted")
