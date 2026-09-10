import ast
import math
from pathlib import Path
from typing import Tuple


RTK_SOURCE_PATH = (
    Path(__file__).resolve().parents[1] / "rtk_nav" / "rtk_nav.py"
)


def _function(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    raise AssertionError(f"function {name!r} not found")


def _load_helper():
    tree = ast.parse(RTK_SOURCE_PATH.read_text(encoding="utf-8"))
    helper = _function(tree, "combine_stanley_and_visual_correction")
    namespace = {"math": math, "Tuple": Tuple}
    exec(
        compile(ast.Module(body=[helper], type_ignores=[]), str(RTK_SOURCE_PATH), "exec"),
        namespace,
    )
    return namespace["combine_stanley_and_visual_correction"]


def test_rtk_and_visual_corrections_use_their_own_speed_units_and_limits():
    combine = _load_helper()

    stanley, visual, total = combine(12.0, 1.2, 1.5, 1.5)

    assert math.isclose(stanley, 0.4, abs_tol=1e-9)
    assert math.isclose(visual, 1.2, abs_tol=1e-9)
    assert math.isclose(total, 1.6, abs_tol=1e-9)


def test_corrections_do_not_depend_on_base_speed_scale():
    combine = _load_helper()

    full_speed = combine(45.0, 1.5, 1.5, 1.5)
    reduced_speed = combine(45.0, 1.5, 1.5, 1.5)

    assert full_speed == reduced_speed
    assert full_speed == (1.5, 1.5, 3.0)


def test_independent_ratios_are_applied_before_each_correction_limit():
    combine = _load_helper()

    stanley, visual, total = combine(45.0, 1.5, 1.5, 1.5, 2.0, 0.25)

    assert math.isclose(stanley, 1.5, abs_tol=1e-9)
    assert math.isclose(visual, 0.375, abs_tol=1e-9)
    assert math.isclose(total, 1.875, abs_tol=1e-9)


def test_rtk_and_visual_correction_ratios_scale_independently():
    combine = _load_helper()

    stanley, visual, total = combine(45.0, 1.5, 1.5, 1.5, 0.5, 0.25)

    assert math.isclose(stanley, 0.75, abs_tol=1e-9)
    assert math.isclose(visual, 0.375, abs_tol=1e-9)
    assert math.isclose(total, 1.125, abs_tol=1e-9)


def test_rtk_output_uses_twelve_speed_command_limit():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    timer = ast.unparse(_function(tree, "rtk_timer_callback"))

    assert "RTK_OUTPUT_SPEED_LIMIT = 12.0" in source
    assert "RTK_OUTPUT_SPEED_LIMIT" in timer
    assert "visual_max_steering_deg" not in source


def test_navigation_moves_define_full_speed_scale_outside_low_distance():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    for function_name in ("move_to_first_waypoint", "multi_waypoint_nav_generator"):
        navigation_function = _function(tree, function_name)

        speed_selection = next(
            node
            for node in ast.walk(navigation_function)
            if isinstance(node, ast.If)
            and isinstance(node.test, ast.Compare)
            and isinstance(node.test.left, ast.Name)
            and node.test.left.id == "distance"
            and any(
                isinstance(operator, ast.Lt)
                and isinstance(comparator, ast.Name)
                and comparator.id == "LOW_DISTANCE"
                for operator, comparator in zip(node.test.ops, node.test.comparators)
            )
        )

        assert any(
            isinstance(statement, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "speed_scale"
                for target in statement.targets
            )
            and isinstance(statement.value, ast.Constant)
            and statement.value.value == 1.0
            for statement in speed_selection.orelse
        )


def test_stanley_runtime_log_exposes_correction_components():
    source = RTK_SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    steering = ast.unparse(_function(tree, "stanley_steering_control"))

    assert "self._last_stanley_log_time" in steering
    assert "self.get_logger().info" in steering
    assert "stanley_correction=" in steering
    assert "visual_angle_error_deg=" in steering
    assert "visual_correction_raw=" in steering
    assert "visual_speed_correction=" in steering
    assert "visual_heading_correction=" in steering
    assert "visual_lateral_correction=" in steering
    assert "correction=" in steering
    assert "left_speed=" in steering
    assert "right_speed=" in steering
    assert "base_speed=" not in steering
    assert "scale=" not in steering


def test_stanley_correction_interface_has_no_speed_scale_parameter():
    tree = ast.parse(RTK_SOURCE_PATH.read_text(encoding="utf-8"))
    steering = _function(tree, "stanley_steering_control")
    argument_names = {argument.arg for argument in steering.args.args}
    assert "speed_scale" not in argument_names
