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

from charging import bms_checksum, parse_bms_response


def make_frame():
    payload = bytearray(25)
    payload[0:2] = (0x14, 0xB4)  # 53.00 V
    payload[2:4] = (0xFF, 0x9C)  # -1.00 A
    payload[19] = 78  # RSOC
    payload[23:25] = (0x0A, 0xAB)  # 0 C (2731 * 0.1 K)
    frame = bytearray((0xDD, 0x03, 0x00, len(payload))) + payload
    checksum = bms_checksum(frame[2:])
    frame += bytes((checksum >> 8, checksum & 0xFF, 0x77))
    return bytes(frame)


def test_parse_c_protocol_bms_response():
    result = parse_bms_response(make_frame())
    assert result == {
        'capacity_percent': 78.0,
        'total_current': -1.0,
        'total_voltage': 53.0,
        'temperature': 0.0,
    }


def test_parse_c_protocol_rejects_bad_checksum():
    frame = bytearray(make_frame())
    frame[-3] ^= 0x01
    assert parse_bms_response(bytes(frame)) is None
