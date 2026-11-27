import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, Generator, List, Optional, Tuple

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from rtk_nav.navigation_position_safety import ArrivalConfirmation, PositionJumpGuard


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def load_navigation():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    method_names = {
        "normalize_angle", "latlon_to_utm", "calculate_path_bearing",
        "calculate_bearing", "get_path_heading", "_get_resume_path_direction",
        "_is_bridge_area_name", "_is_bridge_entry_waypoint",
        "_select_bridge_drive_direction", "_is_bridge_approach_waypoint",
        "_get_bridge_approach_bearing", "_get_waypoint_calibration_heading",
        "_get_effective_path_heading", "_ensure_bridge_entry_direction",
        "_reset_waypoint_tracking",
        "multi_waypoint_nav_generator", "_start_auto_heading_gate_path_alignment",
        "_confirm_waypoint_arrival",
        "move_to_first_waypoint",
    }
    node_class = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "RTKNavControlNode"
    )
    body = [item for item in tree.body if isinstance(item, ast.Assign)]
    body += [item for item in tree.body if isinstance(item, ast.AnnAssign)]
    body += [
        item for item in tree.body
        if isinstance(item, ast.ClassDef)
        and item.name in {"NavState", "ControlMode", "CalibOrigin"}
    ]
    body += [
        item for item in tree.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "select_geofence_recovery_heading"
    ]
    body.append(ast.ClassDef(
        name="Navigation", bases=[], keywords=[], decorator_list=[],
        body=[
            item for item in node_class.body
            if isinstance(item, ast.FunctionDef) and item.name in method_names
        ],
    ))
    namespace = {
        "math": math, "time": SimpleNamespace(monotonic=lambda: 10.0),
        "Dict": Dict, "Generator": Generator, "List": List,
        "Optional": Optional, "Tuple": Tuple,
        "rclpy": SimpleNamespace(ok=lambda: True),
    }
    module = ast.fix_missing_locations(ast.Module(body=body, type_ignores=[]))
    exec(compile(module, str(SOURCE_PATH), "exec"), namespace)
    return namespace


def make_node():
    namespace = load_navigation()
    node = namespace["Navigation"]()
    node.waypoints = [(120.0, 30.0, 0.0), (120.00001, 30.0, 90.0)]
    node.current_waypoint_idx = 1
    node.current_gps = (120.0, 30.000001)
    node.imu_yaw = 88.5
    node.imu_calibration_offset = 0.0
    node.stanley_path_start = None
    node.stanley_path_direction = -6.269
    node.cross_file_last_waypoint = None
    node.last_waypoint_cache = (119.0, 29.0, 0.0)
    node.heading_abnormal_start_time = None
    node.heading_timed_out = False
    node.brush_active = False
    node.boundary_correct_locked = False
    node.blocked_directions = set()
    node._arrival_confirmation = ArrivalConfirmation()
    node._position_guard = PositionJumpGuard()
    node._position_sample_trusted = True
    node.gps_update_sequence = 1
    node.current_control_mode = namespace["ControlMode"].AUTO_CLEANING
    node._auto_heading_gate_path_alignment_pending = True
    node.nav_context = {
        "nav_state": namespace["NavState"].WAYPOINT_MOVE,
        "target_waypoint": node.waypoints[1],
        "last_distance": 0.0,
    }
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_: None, warn=lambda *_: None, error=lambda *_: None,
    )
    node.check_and_control_brush = lambda: None
    node.calc_distance_to_waypoint = lambda *_: 1.0
    node.check_control_mode = lambda: False
    node.publish_nav_state = lambda *_: None
    node.publish_stop_speed = lambda: None
    node.start_heading_recalibration = lambda direction, *_: node.nav_context.update(
        calib_target_heading=direction, calibration_active=True,
    )
    return node, namespace


def test_resume_after_switch_does_not_rotate_back_to_previous_segment():
    node, namespace = make_node()

    assert next(node.multi_waypoint_nav_generator(resume=True)) == (0.0, 0.0)

    assert not node.nav_context.get("calibration_active", False)
    assert node.nav_context["nav_state"] == namespace["NavState"].WAYPOINT_MOVE


def test_auto_gate_alignment_after_switch_uses_new_segment_direction():
    node, namespace = make_node()

    assert node._start_auto_heading_gate_path_alignment()

    assert 91.0 < node.nav_context["calib_target_heading"] < 92.0
    assert node.nav_context["nav_state"] == namespace["NavState"].WAYPOINT_CALIB


def test_resume_retains_valid_segment_heading_when_robot_is_off_path():
    node, _ = make_node()
    node.stanley_path_start = node.waypoints[0][:2]
    node.stanley_path_direction = 88.5
    node.current_gps = (120.000009, 30.00001)

    assert next(node.multi_waypoint_nav_generator(resume=True)) == (0.0, 0.0)

    assert not node.nav_context.get("calibration_active", False)


def test_completed_turn_invalidates_both_path_cache_fields_before_next_tick():
    node, namespace = make_node()
    node.waypoints.append((120.00001, 30.00001, 0.0))
    node.stanley_path_start = node.waypoints[0][:2]
    node.stanley_path_direction = 88.5
    node.check_control_mode = lambda: True
    node.get_target_waypoint = lambda index: node.waypoints[index]
    node._is_calibration_boundary_active = lambda: False
    node._reset_boundary_cycle_tracking = lambda: None
    node._persist_progress = lambda: None
    node._clear_retreat_context = lambda: None

    def completed_calibration():
        if False:
            yield
        return True

    node.nav_context.update(
        nav_state=namespace["NavState"].WAYPOINT_CALIB,
        calib_generator=completed_calibration(),
    )

    assert next(node.multi_waypoint_nav_generator()) == (0.0, 0.0)

    assert node.current_waypoint_idx == 2
    assert node.stanley_path_start is None
    assert node.stanley_path_direction is None
    assert node.nav_context["target_waypoint"] == node.waypoints[2]


@pytest.mark.parametrize("cross_file", [False, True])
def test_first_point_resume_uses_cross_file_anchor_or_current_position(cross_file):
    node, _ = make_node()
    node.current_waypoint_idx = 0
    node.waypoints = [(120.00001, 30.0, 90.0)]
    node.current_gps = (120.0, 30.0)
    node.nav_context["target_waypoint"] = node.waypoints[0]
    if cross_file:
        node.current_gps = (120.00001, 29.99999)
        node.cross_file_last_waypoint = (120.0, 30.0, 90.0)

    assert node._start_auto_heading_gate_path_alignment()

    assert 91.0 < node.nav_context["calib_target_heading"] < 92.0


def test_bridge_entry_is_only_the_first_waypoint_of_a_bridge_area():
    node, _ = make_node()
    node.waypoint_areas = ["cleaning", "bridge-2-3", "bridge-2-3", "cleaning"]

    assert node._is_bridge_area_name(" bridge-2-3 ")
    assert not node._is_bridge_area_name("cleaning")
    assert node._is_bridge_entry_waypoint(1)
    assert not node._is_bridge_entry_waypoint(2)
    assert not node._is_bridge_entry_waypoint(3)


def test_cleaning_endpoint_calibrates_for_reverse_bridge_approach():
    node, _ = make_node()
    node.waypoints = [
        (120.0, 30.0, 180.0),
        (120.00001, 30.0, 180.0),
    ]
    node.waypoint_areas = ["cleaning", "bridge-2-3"]
    node.current_waypoint_idx = 0
    node.imu_yaw = 200.0

    target_heading = node._get_waypoint_calibration_heading(node.waypoints[0])

    assert node.nav_context["bridge_entry_direction"] == "BACKWARD"
    assert target_heading == pytest.approx(-88.5, abs=0.1)


def test_reverse_direction_survives_tracking_reset_at_bridge_entry():
    node, _ = make_node()
    node.waypoints = [
        (120.0, 30.0, 180.0),
        (120.00001, 30.0, 180.0),
        (120.00002, 30.0, 180.0),
    ]
    node.waypoint_areas = ["cleaning", "bridge-2-3", "bridge-2-3"]
    node.current_waypoint_idx = 1
    node.nav_context["bridge_entry_direction"] = "BACKWARD"

    node._reset_waypoint_tracking()

    assert node.nav_context["bridge_entry_direction"] == "BACKWARD"


def test_bridge_approach_direction_stays_locked_during_calibration_retry():
    node, _ = make_node()
    node.waypoints = [
        (120.0, 30.0, 180.0),
        (120.00001, 30.0, 180.0),
    ]
    node.waypoint_areas = ["cleaning", "bridge-2-3"]
    node.current_waypoint_idx = 0
    node.imu_yaw = 200.0
    node._get_waypoint_calibration_heading(node.waypoints[0])

    node.imu_yaw = 20.0
    target_heading = node._get_waypoint_calibration_heading(node.waypoints[0])

    assert node.nav_context["bridge_entry_direction"] == "BACKWARD"
    assert target_heading == pytest.approx(-88.5, abs=0.1)


@pytest.mark.parametrize(
    ("imu_yaw", "path_bearing", "expected"),
    [
        (90.0, 0.0, "FORWARD"),
        (180.0, 0.0, "BACKWARD"),
    ],
)
def test_bridge_entry_uses_reverse_only_when_forward_turn_exceeds_90_degrees(
    imu_yaw, path_bearing, expected
):
    node, _ = make_node()
    node.imu_yaw = imu_yaw

    assert node._select_bridge_drive_direction(path_bearing) == expected
