import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / 'motor_control'))
import types

for name in ('rclpy', 'serial', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv', 'custom_msgs', 'custom_msgs.srv'):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['rclpy'].node = types.SimpleNamespace(Node=object)
sys.modules['rclpy.node'] = types.SimpleNamespace(Node=object)
sys.modules['rclpy'].init = lambda *a, **k: None
sys.modules['rclpy'].spin = lambda *a, **k: None
sys.modules['rclpy'].shutdown = lambda *a, **k: None
sys.modules['std_msgs.msg'].Float32MultiArray = object
sys.modules['std_msgs.msg'].Int16 = object
sys.modules['std_srvs.srv'].Trigger = object
sys.modules['custom_msgs.srv'].ChargeControl = object
sys.modules['serial'].SerialException = Exception

from charging import bms_checksum, parse_bms_response, smooth_soc_estimate, voltage_to_percentage


def make_frame():
    payload = bytearray(25)
    payload[0:2] = (0x14, 0xB4)  # 53.00 V
    payload[2:4] = (0xFF, 0x9C)  # -1.00 A
    payload[19] = 78  # RSOC, ignored by the parser
    payload[23:25] = (0x0A, 0xAB)  # 0 C (2731 * 0.1 K)
    frame = bytearray((0xDD, 0x03, 0x00, len(payload))) + payload
    checksum = bms_checksum(frame[2:])
    frame += bytes((checksum >> 8, checksum & 0xFF, 0x77))
    return bytes(frame)


def test_parse_c_protocol_bms_response():
    result = parse_bms_response(make_frame())
    assert result == {
        'capacity_percent': 93.0,
        'total_current': -1.0,
        'total_voltage': 53.0,
        'temperature': 0.0,
    }


def test_parse_c_protocol_rejects_bad_checksum():
    frame = bytearray(make_frame())
    frame[-3] ^= 0x01
    assert parse_bms_response(bytes(frame)) is None


def test_voltage_to_percentage_uses_c_curve_boundaries():
    assert voltage_to_percentage(42.9) == 0
    assert voltage_to_percentage(48.1) == 30
    assert voltage_to_percentage(54.3) == 100


def test_smooth_soc_estimate_limits_runtime_voltage_jumps():
    assert smooth_soc_estimate(None, 80.0, 1.0, 30.0, 0.2) == 80.0
    assert smooth_soc_estimate(50.0, 100.0, 1.0, 30.0, 0.2) == 50.2
    assert smooth_soc_estimate(50.0, 100.0, 30.0, 30.0, 0.2) == 56.0
