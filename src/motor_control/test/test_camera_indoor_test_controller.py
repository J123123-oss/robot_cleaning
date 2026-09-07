"""Unit tests for the indoor camera controller's pure control function."""

import ast
import math
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).parents[1]
    / 'motor_control'
    / 'camera_indoor_test_controller.py'
)


def _compute():
    """Load the pure helper without requiring a ROS installation."""
    tree = ast.parse(SOURCE_PATH.read_text(encoding='utf-8'))
    functions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {'clamp', 'compute_indoor_wheel_speeds'}
    ]
    namespace = {'math': math}
    exec(
        compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE_PATH), 'exec'),
        namespace,
    )
    return namespace['compute_indoor_wheel_speeds']


def _speeds(**overrides):
    """Call the helper with the launch defaults and selected overrides."""
    values = {
        'angle_deg': 0.0,
        'lateral_m': 0.0,
        'detected': True,
        'confidence': 0.9,
        'heading_valid': True,
        'lateral_valid': True,
        'base_speed': 1.0,
        'heading_gain': 0.05,
        'lateral_gain': 5.0,
        'max_correction': 0.8,
        'min_confidence': 0.5,
        'heading_deadband_deg': 0.5,
        'lateral_deadband_m': 0.01,
    }
    values.update(overrides)
    return _compute()(**values)


def test_zero_error_outputs_low_speed_forward_motion():
    """The default indoor behavior is straight forward motion."""
    assert _speeds() == (-1.0, 1.0)


def test_opposite_error_signs_produce_opposite_differential_corrections():
    """Positive and negative visual errors must turn in opposite directions."""
    positive = _speeds(angle_deg=10.0)
    negative = _speeds(angle_deg=-10.0)
    assert positive[0] > negative[0]
    assert positive[1] > negative[1]
    assert positive != negative


def test_lateral_error_is_used_only_when_lateral_geometry_is_valid():
    """A missing lateral measurement must not inject a stale correction."""
    valid = _speeds(lateral_m=0.1)
    invalid = _speeds(lateral_m=0.1, lateral_valid=False)
    assert valid != invalid
    assert invalid == (-1.0, 1.0)


def test_invalid_detection_or_confidence_stops_all_wheel_motion():
    """The controller must fail safe when visual data cannot be trusted."""
    assert _speeds(detected=False) == (0.0, 0.0)
    assert _speeds(heading_valid=False) == (0.0, 0.0)
    assert _speeds(confidence=0.49) == (0.0, 0.0)


def test_correction_does_not_reverse_a_wheel():
    """The configured correction keeps both wheels in the forward direction."""
    left, right = _speeds(angle_deg=90.0, lateral_m=1.0)
    assert left <= 0.0
    assert right >= 0.0
