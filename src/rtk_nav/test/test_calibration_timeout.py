import ast
import math
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Generator, Tuple
from unittest.mock import Mock


SOURCE = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"


def calibration_node():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    source_class = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                        and item.name == "RTKNavControlNode")
    method = next(item for item in source_class.body if isinstance(item, ast.FunctionDef)
                  and item.name == "calibrate_heading_at_waypoint")
    node_class = ast.ClassDef("CalibrationNode", [], [], [method], [])
    constants = [item for item in tree.body if isinstance(item, ast.Assign)]
    scope = {"math": math, "time": time, "Generator": Generator, "Tuple": Tuple,
             "rclpy": SimpleNamespace(ok=lambda: True)}
    exec(compile(ast.fix_missing_locations(ast.Module(constants + [node_class], [])),
                 str(SOURCE), "exec"), scope)
    node = scope["CalibrationNode"]()
    node.clear_rtk_error_bits = Mock()
    node.set_rtk_error_bits = Mock()
    node.get_logger = Mock()
    node.get_heading_error = Mock(return_value=3.0)
    node.get_adaptive_turn_speed = Mock(return_value=5.0)
    node.get_speed_correction = Mock(return_value=0.0)
    node.last_wtrtk_time = time.monotonic()
    node.last_orientation_status = 4
    node.imu_yaw = 0.0
    clock = SimpleNamespace(seconds=0.0)
    class Instant:
        def __sub__(self, other):
            return SimpleNamespace(nanoseconds=int((clock.seconds - other.seconds) * 1e9))
        def __init__(self):
            self.seconds = clock.seconds
    node.get_clock = lambda: SimpleNamespace(now=Instant)
    return node, clock


def test_timeout_with_three_degrees_error_is_failure():
    node, clock = calibration_node()
    generator = node.calibrate_heading_at_waypoint(90.0)
    assert next(generator) != (0.0, 0.0)
    clock.seconds = 41.0
    try:
        next(generator)
    except StopIteration as finished:
        assert finished.value is False
    else:
        assert False, "calibration did not terminate on timeout"


def test_waypoint_timeout_does_not_automatically_retry():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    node_class = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                      and item.name == "RTKNavControlNode")
    method = next(item for item in node_class.body if isinstance(item, ast.FunctionDef)
                  and item.name == "multi_waypoint_nav_generator")
    source = ast.unparse(method)
    assert "if not calib_result:" in source
    assert "第{retry_count}/{CALIB_STUCK_MAX_RETRIES}次自动重试" not in source


def test_stale_heading_never_completes_calibration():
    node, _ = calibration_node()
    node.last_wtrtk_time = time.monotonic() - 5.0
    node.get_heading_error.return_value = 0.0
    generator = node.calibrate_heading_at_waypoint(90.0)
    try:
        next(generator)
    except StopIteration as finished:
        assert finished.value is False
    else:
        assert False, "stale heading was accepted or generated motion"
