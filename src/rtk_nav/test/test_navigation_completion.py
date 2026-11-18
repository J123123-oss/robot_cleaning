import ast
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE_ROOT = Path(__file__).parents[2]


def load_callback(relative_path, name, **namespace):
    source = (SOURCE_ROOT / relative_path).read_text(encoding="utf-8")
    tree = ast.parse(source)
    method = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    module = ast.Module(body=[method], type_ignores=[])
    namespace.update(String=SimpleNamespace, json=json)
    exec(compile(module, str(relative_path), "exec"), namespace)
    return namespace[name]


@pytest.mark.parametrize("structured", [False, True])
@pytest.mark.parametrize("auto_loading, expected_key", [(False, "h"), (True, "l")])
def test_completed_only_finishes_once_per_auto_cleaning_run(
    structured, auto_loading, expected_key
):
    callback = load_callback(
        "motor_control/motor_control/motor_control.py", "rtk_nav_status_callback"
    )
    logger = Mock()
    node = SimpleNamespace(
        _last_rtk_nav_state_seq=-1,
        current_control_mode="AUTO_CLEANING",
        nav_status="WAYPOINT_CALIB",
        is_in_bin_process=False,
        auto_cleaning_finish_to_loading=auto_loading,
        switch_state=Mock(),
        get_logger=lambda: logger,
    )
    for sequence in range(1, 11):
        payload = (
            json.dumps({"nav_state": "COMPLETED", "seq": sequence})
            if structured else "COMPLETED"
        )
        callback(node, SimpleNamespace(data=payload))

    node.switch_state.assert_called_once_with(expected_key)
    assert logger.info.call_count == 1
    assert node.current_control_mode == "NORMAL"

    node.current_control_mode = "AUTO_CLEANING"
    payload = (
        json.dumps({"nav_state": "COMPLETED", "seq": 11})
        if structured else "COMPLETED"
    )
    callback(node, SimpleNamespace(data=payload))
    assert node.switch_state.call_count == 2


@pytest.mark.parametrize("mode", ["NORMAL", "REMOTE", "WEB_REMOTE"])
def test_completed_after_manual_takeover_does_not_switch_state(mode):
    callback = load_callback(
        "motor_control/motor_control/motor_control.py", "rtk_nav_status_callback"
    )
    node = SimpleNamespace(
        _last_rtk_nav_state_seq=-1,
        current_control_mode=mode,
        is_in_bin_process=False,
        auto_cleaning_finish_to_loading=False,
        switch_state=Mock(),
        get_logger=Mock(),
    )
    callback(node, SimpleNamespace(data="COMPLETED"))
    node.switch_state.assert_not_called()
    assert node.current_control_mode == mode


@pytest.mark.parametrize("waypoints", [[], [(120.0, 30.0, 0.0)]])
def test_repeated_hold_mode_keeps_stopping_without_repeated_context_reset(waypoints):
    callback = load_callback(
        "rtk_nav/rtk_nav/rtk_nav.py",
        "mode_callback",
        ControlMode=SimpleNamespace(AUTO_CLEANING="AUTO_CLEANING"),
    )
    logger = Mock()
    node = SimpleNamespace(
        _last_mode_msg="AUTO_CLEANING",
        _fallback_episode=False,
        _fallback_candidate=False,
        _disarm_auto_cleaning_heading_gate=Mock(),
        _exit_ultrasonic_suppression=Mock(),
        _is_manual_intervention_pause=lambda: False,
        multi_waypoint_generator=object(),
        nav_running=True,
        nav_context={"nav_state": "IDLE", "calib_retry_count": 0},
        waypoints=waypoints,
        publish_stop_speed=Mock(),
        reset_nav_context=Mock(),
        get_logger=lambda: logger,
    )
    for _ in range(10):
        callback(node, SimpleNamespace(data="HOLD"))

    assert node.publish_stop_speed.call_count == 10
    assert node.reset_nav_context.call_count == (0 if waypoints else 1)
    assert logger.info.call_count == (0 if waypoints else 1)
    assert node.multi_waypoint_generator is None
    assert node.nav_running is False
