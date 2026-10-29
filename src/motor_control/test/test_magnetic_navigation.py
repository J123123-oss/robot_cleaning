import sys
from pathlib import Path

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
