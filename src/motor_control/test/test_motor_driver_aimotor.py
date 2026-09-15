"""Offline tests for the AIMOTOR CANopen driver."""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import Mock


def _load_module():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy.node = rclpy_node

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32MultiArray = type("Float32MultiArray", (), {})
    std_msgs.msg = std_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = type("Odometry", (), {})
    nav_msgs.msg = nav_msgs_msg

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Quaternion = type("Quaternion", (), {})
    geometry_msgs.msg = geometry_msgs_msg

    can = types.ModuleType("can")
    can.Bus = object
    can.Message = object

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "can": can,
    }
    previous = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    try:
        path = Path(__file__).parents[1] / "motor_control" / "motor_driver(AIMotor).py"
        spec = importlib.util.spec_from_file_location("motor_driver_aimotor", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, previous_module in previous.items():
            if previous_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous_module


def _make_driver(module):
    driver = module.CanMotorDriver.__new__(module.CanMotorDriver)
    driver.velocity_ratio = 10000.0
    driver.encoder_pulses_per_rev = 1000
    driver.motors = [
        {
            "id": motor_id,
            "velocity": 0.0,
            "actual_velocity": 0.0,
            "actual_position": 0.0,
            "actual_torque": 0.0,
            "actual_temperature": 0.0,
            "status_word": 0,
            "mode_display": 0,
            "fault_code": 0,
            "send_errors": 0,
            "online": True,
        }
        for motor_id in (1, 2, 3)
    ]
    driver.motor_fault_publisher = Mock()
    driver.get_logger = Mock(return_value=Mock())
    return driver


def test_standard_can_frame_and_sdo_speed_write():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    capture_frame = lambda can_id, data: frames.append((can_id, data)) or True
    driver.send_can_frame = capture_frame

    assert driver.motor_set_speed(1, 1.25)
    assert frames == [(
        0x601,
        bytes.fromhex("23 ff 60 00 d4 30 00 00"),
    )]

    class Message:
        def __init__(self, arbitration_id, data, is_extended_id):
            self.arbitration_id = arbitration_id
            self.data = data
            self.is_extended_id = is_extended_id

    class Bus:
        def __init__(self):
            self.messages = []

        def send(self, message):
            self.messages.append(message)

    module.can.Message = Message
    driver.bus = Bus()
    driver.send_can_frame = module.CanMotorDriver.send_can_frame.__get__(driver)
    assert driver.send_can_frame(0x601, b"\x01")
    assert driver.bus.messages[0].is_extended_id is False
    assert driver.bus.messages[0].data == b"\x01" + b"\x00" * 7


def test_enable_disable_and_fault_reset_use_cia402_controlword():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_enable(1)
    assert [frame[0] for frame in frames] == [0x601, 0x601, 0x601]
    assert [frame[1] for frame in frames] == [
        bytes.fromhex("2b 40 60 00 06 00 00 00"),
        bytes.fromhex("2b 40 60 00 07 00 00 00"),
        bytes.fromhex("2b 40 60 00 0f 00 00 00"),
    ]

    assert driver.motor_disable(1)
    assert frames[-1] == (0x601, bytes.fromhex("2b 40 60 00 07 00 00 00"))
    assert driver.motor_clear_fault(1)
    assert frames[-1] == (0x601, bytes.fromhex("2b 40 60 00 80 00 00 00"))


def test_mode_and_feedback_queries_use_documented_objects():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_set_mode(1, 2)
    assert frames[-1] == (0x601, bytes.fromhex("2f 60 60 00 03 00 00 00"))

    assert driver.motor_query_feedback(1)
    assert [frame[1] for frame in frames[-4:]] == [
        bytes.fromhex("40 41 60 00 00 00 00 00"),
        bytes.fromhex("40 64 60 00 00 00 00 00"),
        bytes.fromhex("40 6c 60 00 00 00 00 00"),
        bytes.fromhex("40 77 60 00 00 00 00 00"),
    ]


def test_sdo_feedback_and_emcy_are_decoded():
    module = _load_module()
    driver = _make_driver(module)

    driver.parse_motor_feedback(
        0x581, bytes.fromhex("4b 41 60 00 37 02 00 00")
    )
    driver.parse_motor_feedback(
        0x581, bytes.fromhex("43 64 60 00 e8 03 00 00")
    )
    driver.parse_motor_feedback(
        0x581, bytes.fromhex("43 6c 60 00 e0 b1 ff ff")
    )
    driver.parse_motor_feedback(
        0x581, bytes.fromhex("4b 77 60 00 19 00 00 00")
    )

    motor = driver.motors[0]
    assert motor["status_word"] == 0x0237
    assert abs(motor["actual_position"] - 2.0 * 3.141592653589793) < 1e-9
    assert motor["actual_velocity"] == -2.0
    assert motor["actual_torque"] == 2.5

    driver.parse_motor_fault(0x081, bytes.fromhex("34 12 08 00 00 00 00 00"))
    assert motor["fault_code"] == 0x1234
    driver.motor_fault_publisher.publish.assert_called_once()


def test_tpdo1_short_frame_does_not_partially_update_position():
    module = _load_module()
    driver = _make_driver(module)
    driver.motors[0]["actual_position"] = 7.0

    driver.parse_motor_feedback(0x181, bytes.fromhex("37 02 03 e8 03 00"))
    assert driver.motors[0]["actual_position"] == 7.0

    driver.parse_motor_feedback(0x181, bytes.fromhex("37 02 03 e8 03 00 00"))
    assert abs(driver.motors[0]["actual_position"] - 2.0 * 3.141592653589793) < 1e-9


def test_obsolete_vendor_parameter_methods_do_not_send_old_frames():
    module = _load_module()
    driver = _make_driver(module)
    driver.send_can_frame = Mock()

    assert driver.motor_set_current_limit(1, 20.0) is False
    assert driver.motor_set_other_param(1, 15.0) is False
    driver.send_can_frame.assert_not_called()
