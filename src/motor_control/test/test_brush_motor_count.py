"""Tests for brush motor topology configuration."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Tuple
from unittest.mock import Mock


SOURCE_PATH = (
    Path(__file__).parents[1]
    / "motor_control"
    / "motor_control.py"
)


def test_startup_auto_enable_selects_aimotor_and_kinco_and_keeps_zero_speed():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == "auto_enable_startup_motors")
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]),
                 str(SOURCE_PATH), "exec"), namespace)
    controller = SimpleNamespace(
        motor_auto_enable=True, brush_use_kinco=True,
        current_status="ENABLE",
        motor_ctrl=Mock(motor_ids=(1, 2, 3, 4)),
        get_logger=Mock(return_value=Mock()),
    )
    controller.switch_state = lambda key: setattr(controller, "current_status", "DISABLE")
    controller.motor_ctrl.initialize_motors.return_value = True
    namespace["auto_enable_startup_motors"](controller)
    controller.motor_ctrl.initialize_motors.assert_called_once_with(motor_ids=(1, 2, 3, 4))
    assert controller.current_status == "ENABLE"
    controller.motor_ctrl.initialize_motors.return_value = False
    namespace["auto_enable_startup_motors"](controller)
    assert controller.current_status == "DISABLE"
    controller.motor_ctrl.reset_mock()
    controller.motor_auto_enable = False
    namespace["auto_enable_startup_motors"](controller)
    controller.motor_ctrl.initialize_motors.assert_not_called()


def test_control_main_preserves_driver_shutdown_policy_on_normal_exit_only():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    main = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                and item.name == "main")
    external_shutdown = type("ExternalShutdownException", (Exception,), {})
    for exit_reason in (KeyboardInterrupt(), external_shutdown(),
                        RuntimeError("control failed")):
        controller = Mock()
        ros = SimpleNamespace(
            init=Mock(), spin=Mock(side_effect=exit_reason),
            ok=Mock(return_value=False), shutdown=Mock(),
        )
        namespace = {
            "rclpy": ros, "MotorControlNode": Mock(return_value=controller),
            "ExternalShutdownException": external_shutdown,
            "traceback": SimpleNamespace(format_exc=Mock(return_value="trace")),
        }
        exec(compile(ast.Module(body=[main], type_ignores=[]),
                     str(SOURCE_PATH), "exec"), namespace)

        namespace["main"]()

        controller.motor_ctrl.destroy_node.assert_called_once()
        controller.motor_ctrl.motor_disable.assert_not_called()
        if isinstance(exit_reason, RuntimeError):
            controller.motor_ctrl.stop_all_motors.assert_called_once_with()
        else:
            controller.motor_ctrl.stop_all_motors.assert_not_called()
        ros.shutdown.assert_not_called()


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
