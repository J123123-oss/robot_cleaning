"""验证进仓 GPS 阶段检测磁条后切换到磁导航。"""

import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def _load_methods():
    path = Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "MotorControlNode"
    )
    names = {
        "_update_loading_gps_magnetic_count",
        "_handoff_to_magnetic_loading_if_ready",
        "handle_loading_step",
    }
    methods = [
        node for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {"time": SimpleNamespace(time=lambda: 20.1, monotonic=lambda: 20.1)}
    exec(
        compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"),
        namespace,
    )
    return namespace


@pytest.fixture
def controller():
    namespace = _load_methods()
    node = SimpleNamespace(
        current_status="LOADING",
        is_in_bin_process=True,
        bin_process_paused=False,
        enable_magnetic_navigation=True,
        magnetic_mask_update_time=20.0,
        magnetic_raw_mask=0x18,
        loading_phase="LOADING_TURN",
        magnetic_gps_exit_count=0,
        magnetic_gps_exit_required=2,
        set_motors_speed=Mock(),
        loading_direct_start_time=None,
        loading_direct_elapsed=4.0,
        loading_direct_last_update_time=None,
        loading_direct_motion_active=True,
        magnetic_search_phase="LEFT_SEARCH",
        magnetic_search_phase_start=12.0,
        magnetic_search_reference_yaw=1.0,
        magnetic_search_recover_target_yaw=2.0,
        imu_yaw_deg=-72.0,
        get_logger=Mock(return_value=Mock()),
    )
    for name in (
        "_update_loading_gps_magnetic_count",
        "_handoff_to_magnetic_loading_if_ready",
        "handle_loading_step",
    ):
        method = namespace.get(name)
        if method is None:
            continue
        setattr(node, name, method.__get__(node))
    return node


def test_two_magnetic_samples_handoff_from_loading_turn(controller):
    controller._update_loading_gps_magnetic_count(0x08)
    assert controller.magnetic_gps_exit_count == 1
    assert not controller._handoff_to_magnetic_loading_if_ready(20.0)

    controller._update_loading_gps_magnetic_count(0x18)

    controller.handle_loading_step()

    assert controller.loading_phase == "LOADING_DIRECT_BACKWARD"
    assert controller.loading_direct_start_time == 20.1
    assert controller.loading_direct_elapsed == 0.0
    assert controller.loading_direct_last_update_time == 20.1
    assert not controller.loading_direct_motion_active
    assert controller.magnetic_search_phase == "RIGHT_SEARCH"
    controller.set_motors_speed.assert_called_once_with(0.0, 0.0)


def test_zero_sample_resets_loading_gps_magnetic_sequence(controller):
    controller._update_loading_gps_magnetic_count(0x08)
    controller._update_loading_gps_magnetic_count(0)

    assert controller.magnetic_gps_exit_count == 0
    assert not controller._handoff_to_magnetic_loading_if_ready(20.0)
    assert controller.loading_phase == "LOADING_TURN"


@pytest.mark.parametrize("override", [
    {"magnetic_mask_update_time": 19.0},
    {"magnetic_mask_update_time": None},
    {"magnetic_raw_mask": 0},
    {"enable_magnetic_navigation": False},
    {"bin_process_paused": True},
])
def test_stale_disabled_or_paused_never_handoffs(controller, override):
    controller.magnetic_gps_exit_count = 2
    for name, value in override.items():
        setattr(controller, name, value)
    assert not controller._handoff_to_magnetic_loading_if_ready(20.1)
    assert controller.loading_phase == "LOADING_TURN"


@pytest.mark.parametrize("phase", [
    "LOADING_DIRECT_BACKWARD",
    "COMPLETE",
    None,
])
def test_other_loading_phases_do_not_accumulate_gps_handoff_samples(controller, phase):
    controller.loading_phase = phase

    controller._update_loading_gps_magnetic_count(0x08)
    controller._update_loading_gps_magnetic_count(0x18)

    assert controller.magnetic_gps_exit_count == 0
