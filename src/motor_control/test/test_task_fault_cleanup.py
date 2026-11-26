import ast
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


SOURCE = Path(__file__).parents[1] / "motor_control" / "motor_control.py"


def _load_callback():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    method = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "rtk_nav_status_callback"
    )
    namespace = {"String": SimpleNamespace, "json": json}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["rtk_nav_status_callback"]


def test_completed_clears_task_fault_latches_before_switching_state():
    callback = _load_callback()
    node = SimpleNamespace(
        _last_rtk_nav_state_seq=-1,
        current_control_mode="AUTO_CLEANING",
        nav_status="WAYPOINT_MOVE",
        is_in_bin_process=False,
        auto_cleaning_finish_to_loading=False,
        brush_use_kinco=False,
        rtk_error_code=4 | 8 | 64 | 128,
        low_battery_warning=True,
        auto_cleaning_low_battery_fault=True,
        loading_timeout_error=True,
        geofence_fault_latched=True,
        geofence_recovery_ready=True,
        geofence_recovery_active=True,
        switch_state=Mock(),
        publish_state=Mock(),
        get_logger=Mock(),
    )

    callback(node, SimpleNamespace(
        data=json.dumps({"nav_state": "COMPLETED", "seq": 1})
    ))

    assert node.rtk_error_code == 0
    assert not node.low_battery_warning
    assert not node.auto_cleaning_low_battery_fault
    assert not node.loading_timeout_error
    assert not node.geofence_fault_latched
    assert not node.geofence_recovery_ready
    assert not node.geofence_recovery_active
    node.switch_state.assert_called_once_with("h")
