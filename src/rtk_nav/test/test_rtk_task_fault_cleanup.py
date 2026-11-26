import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


SOURCE = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def _load_finish():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    method = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "finish_navigation_task"
    )
    namespace = {
        "NavState": SimpleNamespace(COMPLETED="COMPLETED", IDLE="IDLE"),
    }
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["finish_navigation_task"]


def test_finish_navigation_task_clears_rtk_and_geofence_fault_state():
    finish_navigation_task = _load_finish()
    node = SimpleNamespace(
        pending_next_path_file=None,
        nav_context={"nav_state": "WAYPOINT_MOVE"},
        rtk_error_code=4 | 8 | 64 | 128,
        geofence_latched=True,
        geofence_outside_count=3,
        geofence_recovery_active=True,
        geofence_recovery=object(),
        publish_nav_state=Mock(),
        publish_stop_speed=Mock(),
        reset_nav_context=Mock(),
        _clear_progress_snapshot=Mock(),
        update_cleaning_area=Mock(),
        update_rtk_error_status=Mock(),
        publish_geofence_recovery_ready=Mock(),
        _publish_geofence_recovery_active=Mock(),
        waypoints=[(120.0, 30.0, 0.0)],
        waypoint_areas=["work"],
        current_cleaning_area="work",
        waiting_for_next_unloading=False,
        get_logger=Mock(),
    )

    finish_navigation_task(node)

    assert node.rtk_error_code == 0
    assert not node.geofence_latched
    assert node.geofence_outside_count == 0
    assert not node.geofence_recovery_active
    assert node.geofence_recovery is None
    node.update_rtk_error_status.assert_called_once_with(0, force=True)
