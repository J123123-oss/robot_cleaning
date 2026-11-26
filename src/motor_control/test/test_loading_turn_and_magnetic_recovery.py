import ast
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).parents[1]))

from motor_control.magnetic_navigation import calculate_magnetic_error


SOURCE = Path(__file__).parents[1] / "motor_control" / "motor_control.py"


def _load_methods(names, namespace=None):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "MotorControlNode"
    )
    methods = [
        node for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    scope = {"time": SimpleNamespace(time=lambda: 10.1, monotonic=lambda: 10.1)}
    if namespace:
        scope.update(namespace)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), "exec"), scope)
    return scope


def test_magnetic_mask_does_not_interrupt_recovery_phase():
    scope = _load_methods({"magnetic_mask_callback"}, {"UInt16": object})
    logger = Mock()
    node = SimpleNamespace(
        current_status="LOADING",
        is_in_bin_process=True,
        bin_process_paused=False,
        loading_countdown_started=True,
        magnetic_raw_mask=0,
        magnetic_mask_update_time=10.0,
        magnetic_gps_exit_count=0,
        magnetic_last_nonzero_mask=0x80,
        magnetic_last_nonzero_time=10.0,
        magnetic_mask=0,
        magnetic_loading_end_active_bits=6,
        magnetic_loading_end_count=0,
        magnetic_loading_end_count_required=3,
        magnetic_center_count=0,
        magnetic_center_lost_count=0,
        magnetic_center_lost_limit=3,
        magnetic_tracking_ready=False,
        magnetic_center_ready_time=None,
        magnetic_precise_error_threshold=0.4,
        magnetic_precise_required=3,
        magnetic_precise_count=0,
        magnetic_precise_ready=False,
        magnetic_precise_ready_time=None,
        magnetic_precise_lost_count=0,
        magnetic_precise_lost_limit=3,
        magnetic_search_phase="RIGHT_RECOVER",
        magnetic_search_phase_start=10.0,
        magnetic_search_reference_yaw=0.0,
        magnetic_search_recover_target_yaw=0.0,
        magnetic_search_active_duration=4.0,
        imu_yaw_deg=0.0,
        magnetic_last_log_time=0.0,
        is_charge_paused=False,
        last_charge_stop_time=0.0,
        _update_loading_gps_magnetic_count=Mock(),
        _disable_if_docked_while_enabled=Mock(return_value=False),
        _magnetic_center_candidate=lambda mask: (False, None),
        _magnetic_precise_candidate=lambda mask, error: False,
        _magnetic_search_phase_from_mask=lambda mask: "RIGHT_SEARCH",
        _magnetic_search_duration_from_mask=lambda mask: 4.0,
        get_logger=Mock(return_value=logger),
    )
    scope["magnetic_mask_callback"](node, SimpleNamespace(data=0x80))

    assert node.magnetic_search_phase == "RIGHT_RECOVER"


def _make_loading_turn_controller(scope):
    clock = scope["time"]
    node = SimpleNamespace(
        current_status="LOADING",
        is_in_bin_process=True,
        bin_process_paused=False,
        loading_phase="LOADING_TURN",
        loading_turn_target_deg=0.0,
        loading_start_time=0.0,
        loading_turn_time=60.0,
        loading_turn_slow_started_at=None,
        loading_turn_slow_reference_yaw=None,
        loading_turn_speed_boosted=False,
        loading_turn_slow_timeout=5.0,
        loading_turn_min_yaw_change=2.0,
        loading_direct_start_time=None,
        loading_countdown_started=False,
        imu_yaw_deg=-5.0,
        last_imu_update_time=0.0,
        yaw_diff_min=0.3,
        yaw_stable_count=0,
        current_lon=0.0,
        current_lat=0.0,
        get_speed_correction=Mock(return_value=0.0),
        get_heading_error=lambda target: target - node.imu_yaw_deg,
        get_adaptive_turn_speed=Mock(return_value=20.0),
        get_motor_feedback_snapshot=Mock(
            return_value=[{"actual_velocity": 0.0}, {"actual_velocity": 0.0}]
        ),
        set_motors_speed=Mock(),
        get_logger=Mock(return_value=Mock()),
        _handoff_to_magnetic_loading_if_ready=Mock(return_value=False),
    )
    node.handle_loading_step = scope["handle_loading_step"].__get__(node)
    node._handle_loading_step_inner = scope["_handle_loading_step_inner"].__get__(node)
    return node, clock


def test_slow_loading_turn_boosts_after_five_seconds_without_yaw_change():
    scope = _load_methods(
        {"handle_loading_step", "_handle_loading_step_inner"},
        {"calculate_magnetic_error": calculate_magnetic_error},
    )
    node, clock = _make_loading_turn_controller(scope)

    clock.time = lambda: 0.0
    node.handle_loading_step()
    node.set_motors_speed.assert_called_with(-10.0, -10.0)

    clock.time = lambda: 5.1
    node.handle_loading_step()

    node.set_motors_speed.assert_called_with(-20.0, -20.0)
    assert node.loading_turn_speed_boosted
