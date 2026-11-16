"""Tests for the standalone Kinco target-current-limit utility."""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import patch

import pytest


SOURCE = (
    Path(__file__).parents[1]
    / "motor_control"
    / "kinco_current_limit.py"
)


def load_module():
    can = types.ModuleType("can")
    can.Message = lambda **kwargs: types.SimpleNamespace(**kwargs)
    can.Bus = object
    previous = sys.modules.get("can")
    sys.modules["can"] = can
    try:
        spec = importlib.util.spec_from_file_location(
            "kinco_current_limit_under_test", SOURCE
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        if previous is None:
            sys.modules.pop("can", None)
        else:
            sys.modules["can"] = previous


def test_build_sdo_frames_for_read_and_two_byte_write():
    module = load_module()
    assert module.build_sdo_read_frame(3) == bytes.fromhex(
        "40 73 60 00 00 00 00 00"
    )
    assert module.build_sdo_write_frame(3, 603) == bytes.fromhex(
        "2B 73 60 00 5B 02 00 00"
    )
    assert module.build_sdo_save_control_frame(3) == bytes.fromhex(
        "2F F0 2F 01 01 00 00 00"
    )


def test_current_conversion_matches_manual_formula():
    module = load_module()
    assert module.arms_to_dec(10.0, 48.0) == 603
    assert module.dec_to_arms(603, 48.0) == pytest.approx(10.0, abs=0.01)


def test_read_returns_unsigned_16_bit_value_and_rejects_abort():
    module = load_module()
    bus = types.SimpleNamespace(
        send=lambda message: None,
        recv=lambda timeout: types.SimpleNamespace(
            arbitration_id=0x583,
            data=bytes.fromhex("4B 73 60 00 5B 02 00 00"),
        ),
    )
    assert module.read_current_limit(bus, 3, timeout=0.01) == 603

    abort_bus = types.SimpleNamespace(
        send=lambda message: None,
        recv=lambda timeout: types.SimpleNamespace(
            arbitration_id=0x583,
            data=bytes.fromhex("80 73 60 00 30 00 09 06"),
        ),
    )
    with pytest.raises(module.KincoProtocolError):
        module.read_current_limit(abort_bus, 3, timeout=0.01)


def test_set_current_limit_requires_success_response():
    module = load_module()
    sent = []

    class Message:
        def __init__(self, arbitration_id, data, is_extended_id):
            sent.append((arbitration_id, bytes(data), is_extended_id))

    module.can.Message = Message
    responses = iter((
        bytes.fromhex("60 73 60 00 5B 02 00 00"),
        bytes.fromhex("60 F0 2F 01 01 00 00 00"),
    ))
    bus = types.SimpleNamespace(
        send=lambda message: None,
        recv=lambda timeout: types.SimpleNamespace(
            arbitration_id=0x583, data=next(responses)
        ),
    )
    assert module.set_current_limit(bus, 3, 603, timeout=0.01)
    assert sent == [
        (0x603, bytes.fromhex("2B 73 60 00 5B 02 00 00"), False),
        (0x603, bytes.fromhex("2F F0 2F 01 01 00 00 00"), False),
    ]


def test_set_current_limit_saves_control_parameters_after_write():
    module = load_module()
    sent = []

    class Message:
        def __init__(self, arbitration_id, data, is_extended_id):
            sent.append((arbitration_id, bytes(data), is_extended_id))

    module.can.Message = Message
    responses = iter((
        bytes.fromhex("60 73 60 00 5B 02 00 00"),
        bytes.fromhex("60 F0 2F 01 01 00 00 00"),
    ))
    bus = types.SimpleNamespace(
        send=lambda message: None,
        recv=lambda timeout: types.SimpleNamespace(
            arbitration_id=0x583, data=next(responses)
        ),
    )
    assert module.set_current_limit(bus, 3, 603, timeout=0.01)
    assert sent == [
        (0x603, bytes.fromhex("2B 73 60 00 5B 02 00 00"), False),
        (0x603, bytes.fromhex("2F F0 2F 01 01 00 00 00"), False),
    ]


def test_set_current_limit_does_not_save_when_limit_write_fails():
    module = load_module()
    sent = []

    class Message:
        def __init__(self, arbitration_id, data, is_extended_id):
            sent.append((arbitration_id, bytes(data), is_extended_id))

    module.can.Message = Message
    bus = types.SimpleNamespace(
        send=lambda message: None,
        recv=lambda timeout: types.SimpleNamespace(
            arbitration_id=0x583,
            data=bytes.fromhex("80 73 60 00 30 00 09 06"),
        ),
    )
    with pytest.raises(module.KincoProtocolError):
        module.set_current_limit(bus, 3, 603, timeout=0.01)
    assert len(sent) == 1


def test_cli_set_accepts_arms_and_peak_current(monkeypatch, capsys):
    module = load_module()
    calls = []
    monkeypatch.setattr(
        module,
        "open_bus",
        lambda channel, bitrate: types.SimpleNamespace(
            shutdown=lambda: None
        ),
    )
    monkeypatch.setattr(
        module,
        "set_current_limit",
        lambda bus, motor_id, raw_value, timeout: calls.append(
            (motor_id, raw_value)
        ) or True,
    )
    with patch.object(
        sys,
        "argv",
        [
            "kinco_current_limit",
            "set",
            "--motor-id",
            "3",
            "--arms",
            "10",
            "--ipeak",
            "48",
        ],
    ):
        assert module.main() == 0
    assert calls == [(3, 603)]
    assert "603" in capsys.readouterr().out
