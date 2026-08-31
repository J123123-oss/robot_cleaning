"""Unit tests for the CANopen motor protocol adapter."""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import Mock


def _load_motor_driver_module():
    """Load motor_driver.py with ROS message modules replaced by stubs."""
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
        path = Path(__file__).parents[1] / "motor_control" / "motor_driver.py"
        spec = importlib.util.spec_from_file_location("motor_driver_under_test", path)
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
    """Create a driver instance without starting ROS or a receive thread."""
    driver = module.CanMotorDriver.__new__(module.CanMotorDriver)
    driver.velocity_ratio = 10000.0
    driver.SDO_RX_BASE = 0x600
    driver.SDO_TX_BASE = 0x580
    driver.EMCY_BASE = 0x80
    driver.CONTROLWORD_INDEX = 0x6040
    driver.MODES_OF_OPERATION_INDEX = 0x6060
    driver.TARGET_VELOCITY_INDEX = 0x60FF
    driver.ACTUAL_VELOCITY_INDEX = 0x606C
    driver.POLARITY_INDEX = 0x607E
    driver.ACCELERATION_INDEX = 0x6083
    driver.DECELERATION_INDEX = 0x6084
    driver.ERROR_CODE_INDEX = 0x2601
    driver.SDO_READ = 0x40
    driver.SDO_WRITE_1 = 0x2F
    driver.SDO_WRITE_2 = 0x2B
    driver.SDO_WRITE_4 = 0x23
    driver.SDO_WRITE_RESPONSE = 0x60
    driver.SDO_READ_2_RESPONSE = 0x4B
    driver.SDO_READ_4_RESPONSE = 0x43
    driver.SDO_ABORT = 0x80
    driver.CANOPEN_VELOCITY_MODE = 0x03
    driver.motors = [
        {
            "id": motor_id,
            "velocity": 0.0,
            "actual_velocity": 0.0,
            "fault_code": 0,
            "send_errors": 0,
            "online": True,
        }
        for motor_id in (1, 2, 3)
    ]
    driver.get_logger = Mock(return_value=Mock())
    driver.motor_fault_publisher = Mock()
    return driver


def test_sdo_frame_builder_and_speed_commands():
    """Verify command COB-IDs and all speed-related SDO payloads."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)
    frames = []
    assert set(driver.motors[0]) == {
        "id", "velocity", "actual_velocity", "fault_code", "send_errors", "online"
    }

    class Message:
        """Minimal stand-in for python-can Message."""

        def __init__(self, arbitration_id, data, is_extended_id):
            self.arbitration_id = arbitration_id
            self.data = data
            self.is_extended_id = is_extended_id

    class Bus:
        """Minimal stand-in for a python-can bus."""

        def __init__(self):
            self.messages = []

        def send(self, message):
            self.messages.append(message)

    module.can.Message = Message
    driver.bus = Bus()
    assert driver.send_can_frame(0x601, b"\x00")
    assert driver.bus.messages[0].arbitration_id == 0x601
    assert driver.bus.messages[0].data == b"\x00" * 8
    assert driver.bus.messages[0].is_extended_id is False

    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert module.build_sdo_frame(0x2B, 0x6040, 0, b"\x0F\x00") == bytes.fromhex(
        "2b 40 60 00 0f 00 00 00"
    )
    assert driver.motor_enable(1)
    assert frames[-1] == (0x601, bytes.fromhex("2b 40 60 00 0f 00 00 00"))

    assert driver.motor_disable(1)
    assert frames[-1] == (0x601, bytes.fromhex("2b 40 60 00 06 00 00 00"))

    assert driver.motor_set_mode(1, 2)
    assert frames[-1] == (0x601, bytes.fromhex("2f 60 60 00 03 00 00 00"))

    assert driver.motor_set_speed(1, -1.25)
    expected_pulses = (-12500).to_bytes(4, byteorder="little", signed=True)
    assert frames[-1] == (0x601, bytes((0x23, 0xFF, 0x60, 0x00)) + expected_pulses)

    assert driver.motor_query_feedback(1)
    assert frames[-1] == (0x601, bytes.fromhex("40 6c 60 00 00 00 00 00"))


def test_direction_acceleration_deceleration_and_error_query():
    """Verify polarity, ramp, and error object requests."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_set_direction(2)
    assert frames[-1] == (0x602, bytes.fromhex("2f 7e 60 00 00 00 00 00"))
    assert driver.motor_set_direction(2, reverse=True)
    assert frames[-1] == (0x602, bytes.fromhex("2f 7e 60 00 01 00 00 00"))

    assert driver.motor_set_acceleration(2, 0x01020304)
    assert frames[-1] == (0x602, bytes.fromhex("23 83 60 00 04 03 02 01"))
    assert driver.motor_set_deceleration(2, 0x01020304)
    assert frames[-1] == (0x602, bytes.fromhex("23 84 60 00 04 03 02 01"))

    assert driver.motor_query_error_code(2)
    assert frames[-1] == (0x602, bytes.fromhex("40 01 26 00 00 00 00 00"))


def test_sdo_feedback_and_fault_parsing():
    """Decode signed velocity, extended error status, and EMCY frames."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)

    actual_pulses = -25000
    velocity_response = bytes.fromhex("43 6c 60 00") + actual_pulses.to_bytes(
        4, byteorder="little", signed=True
    )
    driver.parse_sdo_response(0x581, velocity_response)
    assert driver.motors[0]["actual_velocity"] == -2.5

    extended_error = bytes.fromhex("43 01 26 00 01 00 34 12")
    driver.parse_sdo_response(0x581, extended_error)
    assert driver.motors[0]["fault_code"] == 0x1234

    driver.parse_sdo_response(0x582, bytes.fromhex("4b 01 26 00 00 00 00 00"))
    assert driver.motors[1]["fault_code"] == 0

    driver.parse_motor_fault(0x083, bytes.fromhex("10 42 23 00 00 00 00 00"))
    assert driver.motors[2]["fault_code"] == 0x4210
