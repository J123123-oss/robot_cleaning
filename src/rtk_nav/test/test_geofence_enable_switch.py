"""Exercise fence switches without requiring ROS2."""

import ast
import os
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).parents[3]


def method(relative_path, name):
    tree = ast.parse((ROOT / relative_path).read_text(encoding="utf-8"))
    function = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    scope = {"os": os, "Int16": object}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(relative_path), "exec"), scope)
    return scope[name]


@pytest.mark.parametrize("enabled", [False, True])
def test_motor_entry_switch_controls_missing_sidecar(enabled):
    reasons = []
    node = SimpleNamespace(
        enable_geofence=enabled, rtk_path_file="missing_route.txt",
        _reject_auto_cleaning_entry=reasons.append,
    )
    check = method("src/motor_control/motor_control/motor_control.py", "_check_auto_cleaning_entry")
    assert check(node) is (not enabled)
    assert bool(reasons) is enabled


@pytest.mark.parametrize("enabled", [False, True])
def test_runtime_switch_controls_missing_sidecar(enabled):
    reasons = []
    node = SimpleNamespace(
        enable_geofence=enabled, geofence_latched=False, geofence_payload=None,
        trip_geofence_stop=reasons.append,
    )
    enforce = method("src/rtk_nav/rtk_nav/rtk_nav.py", "enforce_geofence")
    assert enforce(node) is enabled
    assert reasons == (["sidecar_missing"] if enabled else [])


def test_launch_defaults_off_and_forwards_to_both_nodes():
    source = (ROOT / "src/rtk_nav/launch/run.launch.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    declarations = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument" and node.args
        and isinstance(node.args[0], ast.Constant) and node.args[0].value == "enable_geofence"
    ]
    assert len(declarations) == 1
    assert 'text="false"' in ast.unparse(declarations[0]).replace("'", '"')
    assert source.count("'enable_geofence': ParameterValue(") == 2


def test_disabled_fence_ignores_fault_messages():
    node = SimpleNamespace(enable_geofence=False)
    callback = method(
        "src/motor_control/motor_control/motor_control.py", "geofence_fault_callback"
    )
    callback(node, SimpleNamespace(data=1024))
    assert not hasattr(node, "geofence_fault_latched")


def test_disabled_fence_skips_loading_sidecar():
    node = SimpleNamespace(enable_geofence=False, geofence_latched=True)
    load = method("src/rtk_nav/rtk_nav/rtk_nav.py", "load_geofence_sidecar")
    assert load(node, "missing.fence.json") is True
    assert node.geofence_payload is None
    assert node.geofence_latched is False


def test_resume_only_requires_fence_when_enabled():
    tree = ast.parse((ROOT / "src/rtk_nav/rtk_nav/rtk_nav.py").read_text(encoding="utf-8"))
    callback = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "resume_progress_callback"
    )
    assert "if self.enable_geofence and (not os.path.isfile(fence_path)):" in ast.unparse(callback)
