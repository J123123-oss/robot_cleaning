import ast
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Optional, Tuple

import pytest


sys.path.insert(0, str(Path(__file__).parents[1]))
from rtk_nav.navigation_position_safety import ArrivalConfirmation, PositionJumpGuard
from rtk_nav.imu_odom_fallback import position_residual_m


def test_guard_allows_valid_motion_and_centimeter_noise():
    guard = PositionJumpGuard()
    assert guard.update((0.0, 0.0), 1_000_000_000, active=True)
    assert guard.update((0.05, 0.0), 1_100_000_000, active=True)
    assert guard.update((0.08, 0.01), 1_200_000_000, active=True)
    assert not guard.blocked


@pytest.mark.parametrize("distance, accepted", [
    (0.142, True),
    (0.300, True),
    (0.301, False),
])
def test_rtk_jump_limit_at_rest(distance, accepted):
    guard = PositionJumpGuard()
    assert guard.update((0.0, 0.0), 1_000_000_000, active=True)

    assert guard.update((distance, 0.0), 1_103_000_000, active=True) is accepted
    assert guard.blocked is not accepted


def test_waiting_does_not_make_a_rejected_jump_acceptable():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    assert not guard.update((0.5, 0.0), 1_100_000_000, active=True)
    for stamp in (2_000_000_000, 10_000_000_000, 60_000_000_000):
        assert not guard.update((0.5, 0.0), stamp, active=True, linear_velocity_mps=0.35)
        assert guard.blocked


def test_jump_recovery_requires_three_distinct_samples_near_trusted_anchor():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    guard.update((0.6, 0.0), 1_100_000_000, active=True)
    assert not guard.update((0.01, 0.0), 1_200_000_000, active=True)
    for _ in range(10):
        assert not guard.update((0.01, 0.0), 1_200_000_000, active=True)
    assert not guard.update((0.02, 0.0), 1_300_000_000, active=True)
    assert guard.update((0.01, 0.0), 1_400_000_000, active=True)
    assert not guard.blocked


def test_bad_recovery_sample_restarts_confirmation():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    for position, stamp in [
        ((0.6, 0.0), 1_100_000_000), ((0.01, 0.0), 1_200_000_000),
        ((0.6, 0.0), 1_300_000_000), ((0.01, 0.0), 1_400_000_000),
        ((0.01, 0.0), 1_500_000_000),
    ]:
        assert not guard.update(position, stamp, active=True)
    assert guard.update((0.01, 0.0), 1_600_000_000, active=True)


@pytest.mark.parametrize("position", [None, (math.nan, 0.0), (0.0, math.inf)])
def test_invalid_positions_cannot_become_trusted(position):
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    assert not guard.update(position, 1_100_000_000, active=True)
    assert guard.blocked


def test_manual_mode_can_reanchor_after_operator_checks_position():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    guard.update((1.0, 0.0), 1_100_000_000, active=True)
    assert guard.update((1.0, 0.0), 1_200_000_000, active=False)
    assert not guard.blocked
    assert guard.update((1.01, 0.0), 1_300_000_000, active=True)


def test_long_sample_gap_cannot_allow_arbitrary_relocalization():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    assert not guard.update((3.0, 0.0), 61_000_000_000, active=True)


def test_duplicate_and_out_of_order_samples_do_not_move_anchor():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    assert not guard.update((5.0, 0.0), 1_000_000_000, active=True)
    assert not guard.update((5.0, 0.0), 900_000_000, active=True)
    assert guard.update((0.01, 0.0), 1_100_000_000, active=True)
    assert not guard.blocked


def test_arrival_requires_distinct_samples_and_resets_on_outside_sample():
    confirmation = ArrivalConfirmation()
    assert not confirmation.update("point8", 1, 0.05)
    for _ in range(10):
        assert not confirmation.update("point8", 1, 0.05)
    assert not confirmation.update("point8", 2, 0.04)
    assert not confirmation.update("point8", 3, 0.12)
    assert not confirmation.update("point8", 4, 0.05)
    assert not confirmation.update("point8", 5, 0.05)
    assert confirmation.update("point8", 6, 0.05)


def test_invalid_sample_freezes_fallback_limit_during_recovery():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    assert not guard.update(None, 2_100_000_000, active=True, reference=(0.0, 0.0))
    for stamp in (2_200_000_000, 2_300_000_000, 2_400_000_000):
        assert not guard.update((0.40, 0.0), stamp, active=True)


def test_recovery_accepts_three_samples_inside_frozen_trusted_range():
    guard = PositionJumpGuard()
    guard.update((0.0, 0.0), 1_000_000_000, active=True)
    guard.update((0.60, 0.0), 1_100_000_000, active=True)
    assert not guard.update((0.04, 0.0), 1_200_000_000, active=True)
    assert not guard.update((-0.04, 0.0), 1_300_000_000, active=True)
    assert guard.update((0.04, 0.0), 1_400_000_000, active=True)


def test_missing_source_position_stamp_blocks_motion():
    node, _ = make_position_node()
    node._update_navigation_position(message(0), 10.0, True, True)
    sample = message(100_000_000)
    sample.position_stamp_ns = 0
    node._update_navigation_position(sample, 10.1, True, True)
    assert node._position_guard.blocked
    assert not node._position_sample_trusted
    assert node.stop_count > 0


def test_first_invalid_position_needs_a_consistent_recovery_location():
    guard = PositionJumpGuard()
    assert not guard.update(None, 1_000_000_000, active=True)
    assert not guard.update((0.0, 0.0), 1_100_000_000, active=True)
    assert not guard.update((1.0, 0.0), 1_200_000_000, active=True)
    assert not guard.update((2.0, 0.0), 1_300_000_000, active=True)


def test_arrival_never_inherits_previous_target_or_invalid_sample_count():
    confirmation = ArrivalConfirmation()
    assert not confirmation.update("point8", 1, 0.05)
    assert not confirmation.update("point8", 2, 0.05)
    assert not confirmation.update("point9", 3, 0.05)
    assert not confirmation.update("point9", 4, 0.05, valid=False)
    assert not confirmation.update("point9", 5, 0.05)
    assert not confirmation.update("point9", 6, 0.05)
    assert confirmation.update("point9", 7, 0.05)
    confirmation.reset()
    assert not confirmation.update("point9", 8, 0.05)


def load_position_node():
    source_path = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    methods = {
        "_update_navigation_position", "_confirm_waypoint_arrival",
        "latlon_to_utm", "calc_distance_to_waypoint",
        "rtk_timer_callback", "heading_callback",
        "_pause_imu_odom_fallback", "_process_imu_odom_fallback",
        "_advance_imu_odom_recovery", "_fallback_age",
    }
    node_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "RTKNavControlNode"
    )
    body = [node for node in tree.body if isinstance(node, ast.Assign)]
    body += [
        node for node in tree.body if isinstance(node, ast.ClassDef)
        and node.name in {"NavState", "ControlMode"}
    ]
    body.append(ast.ClassDef(
        name="PositionNode", bases=[], keywords=[], decorator_list=[],
        body=[
            node for node in node_class.body
            if isinstance(node, ast.FunctionDef) and node.name in methods
        ],
    ))
    namespace = {
        "math": math, "Optional": Optional, "Tuple": Tuple,
        "WTRTK": SimpleNamespace,
        "time": SimpleNamespace(monotonic=lambda: 10.0),
        "position_residual_m": position_residual_m,
        "NavSatStatus": SimpleNamespace(SERVICE_GPS=1),
        "NavSatFix": lambda: SimpleNamespace(
            header=None, status=SimpleNamespace(status=0, service=0)
        ),
    }
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])),
                 str(source_path), "exec"), namespace)
    return namespace


def make_position_node():
    namespace = load_position_node()
    node = namespace["PositionNode"]()
    node._position_guard = PositionJumpGuard()
    node._arrival_confirmation = ArrivalConfirmation()
    node._position_sample_trusted = False
    node.current_control_mode = namespace["ControlMode"].AUTO_CLEANING
    node.current_gps = None
    node.current_waypoint_idx = 1
    node.gps_update_sequence = 0
    node.imu_yaw = 0.0
    node.antenna_offset_front = node.antenna_offset_left = 0.0
    node.enable_imu_odom_fallback = False
    node.last_odom_time = None
    node.odom_linear_velocity_mps = math.nan
    node._last_dual_fixed_position_xy = None
    node._last_dual_fixed_position_lonlat = None
    node._apply_fallback_position_blend = lambda lon, lat, now: (lon, lat)
    node.nav_context = {
        "nav_state": namespace["NavState"].WAYPOINT_MOVE,
        "target_waypoint": (120.0, 30.0, 90.0),
    }
    node.published_positions = []
    node.car_center_gps_pub = SimpleNamespace(
        publish=node.published_positions.append
    )
    node.stop_count = 0
    node.brush_active = False
    node.publish_nav_state = lambda *_: None
    node.gate_reasons = []

    def stop():
        node.stop_count += 1

    node.publish_stop_speed = stop
    node._prepare_auto_cleaning_heading_gate = lambda reason, **_: node.gate_reasons.append(reason)
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda *_: None, info=lambda *_: None, debug=lambda *_: None,
    )
    return node, namespace


def message(stamp, lon=120.0, lat=30.0):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=1 + stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000
        )),
        ins_longitude=lon, ins_latitude=lat, ins_altitude=0.0,
        position_status=4, position_stamp_ns=1_000_000_000 + stamp,
    )


@pytest.mark.parametrize("velocity, odom_age, interval_ns, expected_limit", [
    (0.35, 0.10, 103_000_000, 0.33605),
    (-0.20, 0.10, 2_000_000_000, 0.50),
    (0.35, 0.30, 103_000_000, 0.30),
    (0.35, None, 103_000_000, 0.30),
    (math.nan, 0.10, 103_000_000, 0.30),
])
def test_navigation_jump_limit_uses_fresh_odometry(
    velocity, odom_age, interval_ns, expected_limit
):
    node, _ = make_position_node()
    node._update_navigation_position(message(0), 10.0, True, True)
    now = 10.0 + interval_ns / 1e9
    node.last_odom_time = None if odom_age is None else now - odom_age
    node.odom_linear_velocity_mps = velocity
    # 约 0.33 米的位移，可区分 0.30 米基础阈值与有效速度提供的余量。
    node._update_navigation_position(
        message(interval_ns, lon=120.0000034), now, True, True
    )

    assert node._position_guard.allowed_distance == pytest.approx(expected_limit)
    assert node._position_guard.blocked is (expected_limit == 0.30)
    assert (node.stop_count > 0) is node._position_guard.blocked


def test_rejected_jump_never_updates_published_pose_or_fallback_anchor():
    node, _ = make_position_node()
    node._update_navigation_position(message(0), 10.0, True, True)
    trusted_pose = node.current_gps
    trusted_anchor = node._last_dual_fixed_position_xy

    node._update_navigation_position(message(100_000_000, lon=120.000006), 10.1, True, True)

    assert node.current_gps == trusted_pose
    assert node._last_dual_fixed_position_xy == trusted_anchor
    assert len(node.published_positions) == 1
    assert node.gps_update_sequence == 1
    assert node.stop_count > 0
    assert node.gate_reasons
    assert not node._position_sample_trusted


def test_pose_callback_counts_each_fresh_frame_and_catches_between_tick_excursion():
    node, _ = make_position_node()
    for stamp in (0, 100_000_000):
        node._update_navigation_position(
            message(stamp, lat=30.0000007), 10.0 + stamp / 1e9, True, True
        )
    assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])
    node._update_navigation_position(message(200_000_000, lat=30.000001), 10.2, True, True)
    node._update_navigation_position(message(300_000_000, lat=30.0000007), 10.3, True, True)
    assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])
    assert not node._position_guard.blocked
    assert node.gps_update_sequence == 4


def test_repeated_callbacks_cannot_complete_arrival_and_quality_loss_resets_it():
    node, _ = make_position_node()
    node._update_navigation_position(message(0), 10.0, True, True)
    node._update_navigation_position(message(100_000_000), 10.1, True, True)
    for _ in range(5):
        node._update_navigation_position(message(100_000_000), 10.2, True, True)
        assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])
    node._update_navigation_position(message(200_000_000), 10.2, True, False)
    for stamp in (300_000_000, 400_000_000):
        node._update_navigation_position(message(stamp), 10.0 + stamp / 1e9, True, True)
        assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])
    node._update_navigation_position(message(500_000_000), 10.5, True, True)
    assert node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])


def test_initial_waypoint_still_requires_five_fresh_samples():
    node, namespace = make_position_node()
    node.current_waypoint_idx = 0
    node.nav_context["nav_state"] = namespace["NavState"].INITIAL_MOVE
    for stamp in (0, 100_000_000, 200_000_000, 300_000_000):
        node._update_navigation_position(message(stamp), 10.0 + stamp / 1e9, True, True)
        assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])
    node._update_navigation_position(message(400_000_000), 10.4, True, True)
    assert node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])


def test_real_navigation_generator_stops_until_three_new_samples_confirm_arrival():
    from test_segment_direction_recovery import make_node

    node, namespace = make_node()
    node.check_control_mode = lambda: True
    node.get_target_waypoint = lambda index: node.waypoints[index]
    node.calc_distance_to_waypoint = lambda *_: 0.05
    node._reset_boundary_cycle_tracking = lambda: None
    node.calibrate_heading_at_waypoint = lambda *_: iter([(3.0, 3.0)])
    generator = node.multi_waypoint_nav_generator()

    for _ in range(5):
        assert next(generator) == (0.0, 0.0)
        assert node.nav_context["nav_state"] == namespace["NavState"].WAYPOINT_MOVE
    node.gps_update_sequence = 2
    assert next(generator) == (0.0, 0.0)
    assert node.nav_context["nav_state"] == namespace["NavState"].WAYPOINT_MOVE
    node.gps_update_sequence = 3
    assert next(generator) == (0.0, 0.0)
    assert node.nav_context["nav_state"] == namespace["NavState"].WAYPOINT_CALIB


def test_paused_pose_samples_cannot_complete_arrival_after_resume():
    node, namespace = make_position_node()
    for stamp in (0, 100_000_000):
        node._update_navigation_position(message(stamp), 10.0 + stamp / 1e9, True, True)
    node.nav_context["nav_state"] = namespace["NavState"].PAUSE
    node._update_navigation_position(message(200_000_000), 10.2, True, True)
    node.nav_context["nav_state"] = namespace["NavState"].WAYPOINT_MOVE
    node._update_navigation_position(message(300_000_000), 10.3, True, True)
    assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])


@pytest.mark.parametrize("fresh,matching,accepted", [
    (True, True, True), (False, True, False), (True, False, False),
])
def test_fallback_recovery_uses_only_fresh_valid_odometry_reference(fresh, matching, accepted):
    node, _ = make_position_node()
    node._update_navigation_position(message(0), 10.0, True, True)
    anchor = node._last_dual_fixed_position_xy
    node.enable_imu_odom_fallback = True
    node.last_odom_time = 11.9 if fresh else 10.0
    node._fallback_episode = {
        "integrating": True, "estimator_valid": True,
        "anchor_xy": anchor, "east_m": 0.96 if matching else 0.0, "north_m": 0.0,
    }
    node._update_navigation_position(
        message(2_000_000_000, lon=120.00001), 12.0, True, True
    )
    assert (node.gps_update_sequence == 2) is accepted
    assert not node._position_sample_trusted
    assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])


def test_timer_does_not_send_motion_while_position_guard_is_blocked():
    node, _ = make_position_node()
    node._update_navigation_position(message(0), 10.0, True, True)
    node._update_navigation_position(message(100_000_000, lon=120.000006), 10.1, True, True)
    node._fallback_diagnostics_active = lambda: False
    node._last_nav_context_publish = 10.0
    node.update_cleaning_area = lambda: None
    node.ultrasonic_suppression_active = False
    node.geofence_latched = False
    stops_before_tick = node.stop_count

    node.rtk_timer_callback()

    assert node.stop_count == stops_before_tick + 1
    assert node.gps_update_sequence == 1


def test_body_center_compensation_preserves_position_during_in_place_turn():
    node, _ = make_position_node()
    node.antenna_offset_front = 0.263
    node.antenna_offset_left = 0.258
    node._update_navigation_position(
        message(
            0, lon=120.1 + math.degrees(0.258 / (6378137.0 * math.cos(math.radians(30.0)))),
            lat=30.0 + math.degrees(0.263 / 6378137.0),
        ), 10.0, True, True,
    )
    node.imu_yaw = 90.0
    node._update_navigation_position(
        message(
            100_000_000,
            lon=120.1 + math.degrees(0.263 / (6378137.0 * math.cos(math.radians(30.0)))),
            lat=30.0 - math.degrees(0.258 / 6378137.0),
        ), 10.1, True, True,
    )

    assert node.current_gps == pytest.approx((120.1, 30.0), abs=1e-9)
    assert node.gps_update_sequence == 2
    assert not node._position_guard.blocked


def test_first_point_generator_holds_until_five_fresh_samples_then_turns():
    from test_segment_direction_recovery import make_node

    node, namespace = make_node()
    node.current_waypoint_idx = 0
    node.current_gps = node.waypoints[0][:2]
    node.nav_context["nav_state"] = namespace["NavState"].INITIAL_MOVE
    node.imu_initialized = True
    node.calc_distance_to_waypoint = lambda *_: 0.05
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))
    node._calibrate_with_boundary_retreat = lambda *_: iter([(3.0, 3.0)])
    generator = node.move_to_first_waypoint()

    for _ in range(5):
        assert next(generator) == (0.0, 0.0)
        assert node.nav_context["nav_state"] == namespace["NavState"].INITIAL_MOVE
    for sequence in (2, 3, 4):
        node.gps_update_sequence = sequence
        assert next(generator) == (0.0, 0.0)
    node.gps_update_sequence = 5
    assert next(generator) == (3.0, 3.0)
    assert node.nav_context["nav_state"] == namespace["NavState"].WAYPOINT_CALIB


@pytest.mark.parametrize("position_fixed", [False, True])
def test_quality_loss_breaks_position_jump_recovery_sequence(position_fixed):
    node, _ = make_position_node()
    node._update_navigation_position(message(0), 10.0, True, True)
    node._update_navigation_position(message(100_000_000, lon=120.000006), 10.1, True, True)
    node._update_navigation_position(message(200_000_000), 10.2, True, True)
    node._update_navigation_position(message(300_000_000), 10.3, position_fixed, False)
    for stamp in (400_000_000, 500_000_000):
        node._update_navigation_position(message(stamp), 10.0 + stamp / 1e9, True, True)
        assert node._position_guard.blocked
    node._update_navigation_position(message(600_000_000), 10.6, True, True)
    assert not node._position_guard.blocked


@pytest.mark.parametrize("stamp", [100_000_000, 0])
def test_replayed_wtrtk_frames_do_not_refresh_data_timeout_or_heading(stamp):
    node, _ = make_position_node()
    node._last_wtrtk_stamp_ns = 1_100_000_000
    node.last_wtrtk_time = 5.0
    node.imu_yaw = 42.0

    node.heading_callback(message(stamp))

    assert node.last_wtrtk_time == 5.0
    assert node.imu_yaw == 42.0


def make_driver(filename):
    source_path = Path(__file__).parents[1] / "rtk_nav" / filename
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    driver_class = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    body = [ast.ClassDef(
        name="Driver", bases=[], keywords=[], decorator_list=[],
        body=[
            node for node in driver_class.body
            if isinstance(node, ast.FunctionDef)
            and node.name in {"parse_wtrtk", "dms_to_decimal"}
        ],
    )]
    namespace = {
        "WTRTK": lambda: SimpleNamespace(position_stamp_ns=0),
        "Header": lambda: SimpleNamespace(stamp=None, frame_id=""),
        "time": SimpleNamespace(monotonic=lambda: 10.0),
    }
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])),
                 str(source_path), "exec"), namespace)
    driver = namespace["Driver"]()
    driver.latest_fix = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        latitude=30.0, longitude=120.1, status=SimpleNamespace(status=4),
    )
    driver._gga_position_status = 4
    driver._last_gga_time = 10.0
    driver.gga_timeout = 2.0
    driver.get_logger = lambda: SimpleNamespace(warn=lambda *_: None)
    ticks = iter(range(1, 20))
    driver.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=1, nanosec=next(ticks) * 100_000_000)
    ))
    return driver


@pytest.mark.parametrize("filename", ["wtrtk_serial_driver.py", "wtrtk_parse_txt.py"])
def test_real_driver_preserves_gga_identity_across_new_heading_frames(filename):
    driver = make_driver(filename)
    node, _ = make_position_node()
    node.nav_context["target_waypoint"] = (120.1, 30.0, 90.0)
    fields = ["0"] * 25
    fields[7] = "4"
    frame = "$WTRTK," + ",".join(fields) + "*00"
    for _ in range(5):
        parsed = driver.parse_wtrtk(frame)
        assert parsed.position_stamp_ns == 1_000_000_000
        node._update_navigation_position(parsed, 10.0, True, True)
        assert not node._confirm_waypoint_arrival(node.nav_context["target_waypoint"])
    assert node.gps_update_sequence == 1


def test_one_hz_position_with_ten_hz_heading_uses_real_position_interval():
    node, _ = make_position_node()
    node.nav_context["target_waypoint"] = (120.1, 30.0, 90.0)
    for stamp in range(0, 1_000_000_000, 100_000_000):
        sample = message(stamp, lon=120.1)
        sample.position_stamp_ns = 1_000_000_000
        node._update_navigation_position(sample, 10.0 + stamp / 1e9, True, True)
    node._update_navigation_position(
        message(1_000_000_000, lon=120.100002), 11.0, True, True
    )
    assert node.gps_update_sequence == 2
    assert not node._position_guard.blocked


def test_fallback_jump_returns_through_recovery_without_manual_lockout():
    node, namespace = make_position_node()
    node._update_navigation_position(message(0, lon=120.1), 10.0, True, True)
    node.enable_imu_odom_fallback = True
    node._fallback_lockout_reason = None
    node._fallback_position_blend = None
    node._fallback_candidate = None
    node._fallback_episode = {
        "active": True, "integrating": True, "estimator_valid": True,
        "anchor_xy": node._last_dual_fixed_position_xy,
        "east_m": 0.96, "north_m": 0.0,
    }
    node.position_data_valid = node.rtk_solution_ready = True
    node.last_gps_status = node.last_orientation_status = 4
    node.odom_linear_velocity_mps = 0.0

    def fresh_sensors(now):
        node.last_odom_time = node.last_wtrtk_time = node.last_io_time = now

    def heading_gate(*_args, **_kwargs):
        if node.nav_context["nav_state"] != namespace["NavState"].PAUSE:
            node.nav_context["pre_pause_state"] = node.nav_context["nav_state"]
        node.nav_context.update(
            nav_state=namespace["NavState"].PAUSE, pause_reason="auto_heading_gate"
        )

    node._prepare_auto_cleaning_heading_gate = heading_gate
    fresh_sensors(10.1)
    node._update_navigation_position(message(100_000_000, lon=120.10004), 10.1, True, True)
    assert not node._fallback_episode["active"]
    for stamp in (200_000_000, 300_000_000, 400_000_000):
        fresh_sensors(10.0 + stamp / 1e9)
        node._update_navigation_position(
            message(stamp, lon=120.10001), 10.0 + stamp / 1e9, True, True
        )
    assert not node._position_guard.blocked
    for now in (10.5, 11.6, 12.2):
        fresh_sensors(now)
        assert node._process_imu_odom_fallback(now)
        assert node._fallback_lockout_reason is None
    assert not node._fallback_episode["integrating"]
    assert node.nav_context["nav_state"] == namespace["NavState"].WAYPOINT_MOVE
    assert not node._position_sample_trusted
