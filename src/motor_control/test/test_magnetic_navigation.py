import ast
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from motor_control.magnetic_navigation import (  # noqa: E402
    build_read_frame,
    crc16_modbus,
    decode_response,
    calculate_magnetic_error,
)


def test_read_frame_matches_manual():
    assert build_read_frame(0x08) == bytes.fromhex("08 03 00 01 00 01 D5 53")


def test_decode_response_returns_register_and_rejects_bad_crc():
    frame = bytes.fromhex("08 03 02 00 E0 65 CD")
    assert decode_response(frame, 0x08) == 0x00E0
    bad = frame[:-1] + bytes([frame[-1] ^ 0xFF])
    assert decode_response(bad, 0x08) is None


def test_magnetic_error_is_centered_and_signed():
    assert calculate_magnetic_error(0b00011000) == 0.0
    assert calculate_magnetic_error(0b00000100) < 0.0
    assert calculate_magnetic_error(0b01000000) > 0.0
    assert calculate_magnetic_error(0) is None


@pytest.fixture
def unloading_node():
    path = PACKAGE_ROOT / "motor_control" / "motor_control.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node_class = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                      and item.name == "MotorControlNode")
    names = {"handle_unloading_step", "_unloading_magnetic_correction"}
    methods = [item for item in node_class.body if isinstance(item, ast.FunctionDef)
               and item.name in names]
    clock = Mock()
    namespace = dict(time=clock, String=SimpleNamespace, UNLOADING_SETTLE_DURATION=2.0,
                     calculate_magnetic_error=calculate_magnetic_error)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), namespace)
    node = SimpleNamespace(
        is_in_bin_process=True, bin_process_paused=False, current_status="START",
        unloading_phase=None, unloading_forward_duration_sec=14.0,
        motor_ctrl=SimpleNamespace(BASE_SPEED=20.0), set_motors_speed=Mock(),
        enable_magnetic_navigation=True, unloading_use_magnetic_navigation=True,
        magnetic_raw_mask=0xFF, magnetic_mask_update_time=100.0, magnetic_kp=2.0,
        magnetic_max_correction=4.0, get_logger=Mock(), set_brush_speed=Mock(),
        state_pub=Mock(), switch_state=Mock(), _cancel_bin_process_timers=Mock(),
    )
    for name in names:
        setattr(node, name, namespace[name].__get__(node))
    return node, clock.monotonic


@pytest.mark.parametrize("independent", [False, True])
def test_unloading_guides_on_line_then_continues_to_deadline(unloading_node, independent):
    node, monotonic = unloading_node
    node.unloading_command_hold = independent
    if independent:
        node.current_status = "UNLOADING"

    for elapsed, mask, expected in (
        (0.0, 0xFF, (-20.0, 20.0)),
        (1.0, 0x80, (-24.0, 16.0)),
        (2.0, 0x01, (-16.0, 24.0)),
        (8.0, 0x00, (-20.0, 20.0)),
        (13.9, 0x00, (-20.0, 20.0)),
        (14.0, 0x00, (0.0, 0.0)),
    ):
        monotonic.return_value = node.magnetic_mask_update_time = 100.0 + elapsed
        node.magnetic_raw_mask = mask
        node.handle_unloading_step()
        node.set_motors_speed.assert_called_with(*expected)

    assert node.unloading_phase == "UNLOADING_SETTLING"
    monotonic.return_value = 116.0
    node.handle_unloading_step()
    if independent:
        node.switch_state.assert_not_called()
        assert node.current_status == "HOLD"
    else:
        node.switch_state.assert_called_once_with("r")


@pytest.mark.parametrize("override", [
    {"unloading_use_magnetic_navigation": False},
    {"enable_magnetic_navigation": False},
    {"magnetic_mask_update_time": 99.7},
    {"magnetic_mask_update_time": None},
])
def test_unloading_uses_base_speed_when_correction_is_unavailable(unloading_node, override):
    node, monotonic = unloading_node
    monotonic.return_value = 100.0
    node.magnetic_raw_mask = 0x80
    vars(node).update(override)
    node.handle_unloading_step()
    node.set_motors_speed.assert_called_once_with(-20.0, 20.0)
