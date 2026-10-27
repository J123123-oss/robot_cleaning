import ast
from typing import Tuple
from pathlib import Path


SOURCE = Path(__file__).parents[1] / "rtk_nav" / "rtk_nav.py"
LAUNCH = Path(__file__).parents[1] / "launch" / "run.launch.py"


def _function(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    return next(node for node in ast.walk(tree)
                if isinstance(node, ast.FunctionDef) and node.name == name)


def test_rtk_correction_is_limited_to_twenty_percent_of_wheel_speed():
    source = SOURCE.read_text(encoding="utf-8")
    assert "RTK_MAX_CORRECTION_SPEED = MAX_WHEEL_SPEED_RPM * 0.20" in source
    helper = _function("combine_rtk_correction")
    namespace = {"math": __import__("math"), "Tuple": Tuple}
    exec(compile(ast.Module(body=[helper], type_ignores=[]), str(SOURCE), "exec"), namespace)
    combine = namespace["combine_rtk_correction"]
    assert combine(45.0, 13.369015, 1.0) == (13.369015, 0.0, 13.369015)


def test_launch_has_no_visual_correction_or_camera_inputs():
    launch = LAUNCH.read_text(encoding="utf-8")
    for token in ("enable_visual_correction", "visual_heading_gain", "visual_lateral_gain",
                  "camera_serial_port", "openmv_serial_publisher", "line_detector_node"):
        assert token not in launch


def test_stanley_does_not_call_visual_correction():
    steering = ast.unparse(_function("stanley_steering_control"))
    assert "get_visual_steering_correction" not in steering
    assert "combine_rtk_correction" in steering
