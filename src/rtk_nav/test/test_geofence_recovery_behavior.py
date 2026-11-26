import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

PACKAGE = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE))
from rtk_nav.geofence_geometry import (
    nearest_recovery_target, point_allowed_by_payload, segment_allowed_by_payload,
)


@pytest.fixture
def navigator():
    source = PACKAGE / "rtk_nav" / "rtk_nav.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    names = {"_begin_geofence_recovery", "_service_geofence_recovery"}
    methods = [item for item in ast.walk(tree)
               if isinstance(item, ast.FunctionDef) and item.name in names]
    selector = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                    and item.name == "select_geofence_recovery_heading")
    scope = {"math": math, "Tuple": tuple, "time": SimpleNamespace(monotonic=lambda: 10.0),
             "NavState": SimpleNamespace(INITIAL_MOVE="INITIAL_MOVE", WAYPOINT_MOVE="WAYPOINT_MOVE"),
             "BOUNDARY_IO_MAX_AGE": 0.25, "TURN_SPEED_SLOW": 2.673803,
             "nearest_recovery_target": nearest_recovery_target,
             "segment_allowed_by_payload": lambda *_: True}
    exec(compile(ast.Module(body=[selector, *methods], type_ignores=[]), str(source), "exec"), scope)
    node = SimpleNamespace(
        geofence_latched=True, geofence_recovery_active=False, geofence_recovery=None,
        geofence_entry_payload=None, geofence_entry_pending=False,
        gps_update_sequence=1, brush_active=True, _arrival_confirmation=Mock(),
        publish_stop_speed=Mock(), publish_geofence_recovery_ready=Mock(),
        get_logger=Mock(), geofence_payload={"areas": [], "bridges": [
            {"centerline_utm": [[0, 0], [0, 10]]}], "expansion_radius_m": 0.2},
        waypoints=[(1, 2, 0)], current_waypoint_idx=0, _position_guard=SimpleNamespace(blocked=False),
        rtk_solution_ready=True, _position_sample_trusted=True, _position_trusted_time=10.0,
        last_wtrtk_time=10.0, last_io_time=10.0, _auto_heading_gate_pending=False,
        _boundary_stop_until=None, current_gps=(1.0, 5.0), imu_yaw=90.0,
        _fallback_age=lambda now, stamp: now - stamp, _boundary_chassis_stopped=Mock(return_value=True),
        _boundary_motion_inhibited=Mock(return_value=False),
        latlon_to_utm=lambda lat, lon: (lon, lat), check_geofence_position=Mock(return_value=False),
        nav_context={"nav_state": "PAUSE", "pause_reason": "geofence_stop"},
        publish_nav_state=Mock(), _boundary_blocks_motion=Mock(return_value=False),
        get_heading_error=lambda target: (target - node.imu_yaw + 180) % 360 - 180,
        _publish_geofence_recovery_active=Mock(), _publish_motor_speed=Mock(),
        trip_geofence_stop=Mock(), geofence_recovery_complete_pub=Mock(),
    )
    scope["Bool"] = SimpleNamespace
    scope["_begin_geofence_recovery"](node)
    return node, scope["_service_geofence_recovery"]


def test_recovery_drives_backward_with_smaller_rotation_and_disables_brush(navigator):
    node, service = navigator
    service(node, 10.0)
    assert node.geofence_recovery["direction"] == "BACKWARD"
    assert node.brush_active is False
    assert node.geofence_latched
    service(node, 10.1)
    assert node._publish_motor_speed.call_args.args == (6.0, -6.0)


def test_three_new_inside_samples_and_stopped_feedback_required(navigator):
    node, service = navigator
    node.check_geofence_position.return_value = True
    node._boundary_chassis_stopped.return_value = False
    for sequence in (2, 2, 3, 4):
        node.gps_update_sequence = sequence
        service(node, 10.0)
    assert node.geofence_latched
    node._boundary_chassis_stopped.return_value = True
    service(node, 10.0)
    assert not node.geofence_latched
    assert node.nav_context["nav_state"] == "INITIAL_MOVE"


def test_stale_localization_during_movement_locks_again(navigator):
    node, service = navigator
    service(node, 10.0)
    node.geofence_recovery["phase"] = "DRIVE"
    service(node, 10.6)
    node.trip_geofence_stop.assert_called_with("recovery_data_unavailable")


def test_boundary_trigger_during_recovery_locks_again(navigator):
    node, service = navigator
    service(node, 10.0)
    node._boundary_motion_inhibited.return_value = True
    service(node, 10.1)
    node.trip_geofence_stop.assert_called_with("recovery_motion_inhibited")


def test_recovery_waits_for_stop_before_turning(navigator):
    node, service = navigator
    node._boundary_chassis_stopped.return_value = False
    service(node, 10.0)
    node._publish_motor_speed.assert_not_called()
    assert node.geofence_recovery["phase"] == "WAIT"


def test_recovery_waits_for_fresh_io(navigator):
    node, service = navigator
    node.last_io_time = 9.0
    service(node, 10.0)
    node._publish_motor_speed.assert_not_called()
    assert node.geofence_recovery["phase"] == "WAIT"


def test_target_for_large_polygon_is_near_edge():
    fence = {"areas": [{"boundary_utm": [[0, 0], [100, 0], [100, 100], [0, 100]]}], "bridges": []}
    target = nearest_recovery_target((-0.5, 50), fence)
    assert point_allowed_by_payload(target, fence)
    assert math.dist(target, (-0.5, 50)) < 1.0


def test_recovery_target_can_be_in_common_entry_area():
    fence = {
        "areas": [{"boundary_utm": [[0, 0], [10, 0], [10, 10], [0, 10]]}],
        "bridges": [],
    }
    target = nearest_recovery_target((-0.5, 5), fence)
    assert target is not None
    assert point_allowed_by_payload(target, fence)
