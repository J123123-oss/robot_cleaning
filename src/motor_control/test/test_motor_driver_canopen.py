"""Unit tests for the legacy RS02 motor protocol adapter."""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import Mock


def _load_motor_driver_module():
    """Load motor_driver.py with ROS and python-can modules replaced by stubs."""
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
    """Create a legacy driver instance without starting ROS or a receive thread."""
    driver = module.CanMotorDriver.__new__(module.CanMotorDriver)
    driver.motor_master_id = 0x63
    driver.RUN_MODE_INDEX = 0x7005
    driver.SPEED_REF_INDEX = 0x700A
    driver.LIMIT_CUR_INDEX = 0x7018
    driver.OTHER_PARAM_INDEX = 0x7022
    driver.COMM_WRITE_PARAM = 0x12
    driver.COMM_ENABLE_MOTOR = 0x03
    driver.COMM_DISABLE_MOTOR = 0x04
    driver.MC_CMD_QUERY_MOTOR = 0x03
    driver.motors = [
        {
            "id": motor_id,
            "velocity": 0.0,
            "actual_velocity": 0.0,
            "actual_position": 0.0,
            "actual_torque": 0.0,
            "actual_temperature": 0.0,
            "fault_code": 0,
            "send_errors": 0,
            "online": True,
        }
        for motor_id in (1, 2, 3, 4)
    ]
    driver.get_logger = Mock(return_value=Mock())
    driver.motor_fault_publisher = Mock()
    return driver


def test_legacy_frame_format_and_speed_commands():
    """Verify RS02 extended IDs and parameter-write payloads."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)
    frames = []
    assert set(driver.motors[0]) == {
        "id",
        "velocity",
        "actual_velocity",
        "actual_position",
        "actual_torque",
        "actual_temperature",
        "fault_code",
        "send_errors",
        "online",
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
    assert driver.send_can_frame(0x12006301, b"\x00")
    assert driver.bus.messages[0].arbitration_id == 0x12006301
    assert driver.bus.messages[0].data == b"\x00" * 8
    assert driver.bus.messages[0].is_extended_id is True

    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_enable(1)
    assert frames[-1] == (0x03006301, bytes(8))

    assert driver.motor_disable(1)
    assert frames[-1] == (0x04006301, bytes.fromhex("80 00 00 00 00 00 00 00"))

    assert driver.motor_set_mode(1, 2)
    assert frames[-1] == (0x12006301, bytes.fromhex("05 70 00 00 02 00 00 00"))

    assert driver.motor_set_speed(1, -1.25)
    assert frames[-1] == (0x12006301, bytes.fromhex("0a 70 00 00 00 00 a0 bf"))

    assert driver.motor_set_speed(4, 1.5)
    assert frames[-1] == (0x12006304, bytes.fromhex("0a 70 00 00 00 00 c0 3f"))

    assert driver.motor_query_feedback(1)
    assert frames[-1] == (0x03006301, bytes.fromhex("03 00 00 00 00 00 00 00"))


def test_legacy_parameter_commands_and_fault_reset():
    """Verify current-limit, extra-parameter, and fault-reset commands."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_set_current_limit(2, 20.0)
    assert frames[-1] == (0x12006302, bytes.fromhex("18 70 00 00 00 00 a0 41"))

    assert driver.motor_set_other_param(3, 15.0)
    assert frames[-1] == (0x12006303, bytes.fromhex("22 70 00 00 00 00 70 41"))

    assert driver.motor_clear_fault(4)
    assert frames[-1] == (0x04006304, bytes.fromhex("01 00 00 00 00 00 00 00"))


def test_stop_all_motors_sends_zero_speed_and_disable_frames():
    """Stop directly on CAN so shutdown does not depend on ROS delivery."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)
    driver._stopping = False
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    driver.stop_all_motors()

    assert frames == [
        (0x12006301, bytes.fromhex("0a 70 00 00 00 00 00 00")),
        (0x12006302, bytes.fromhex("0a 70 00 00 00 00 00 00")),
        (0x12006303, bytes.fromhex("0a 70 00 00 00 00 00 00")),
        (0x12006304, bytes.fromhex("0a 70 00 00 00 00 00 00")),
        (0x04006301, bytes.fromhex("80 00 00 00 00 00 00 00")),
        (0x04006302, bytes.fromhex("80 00 00 00 00 00 00 00")),
        (0x04006303, bytes.fromhex("80 00 00 00 00 00 00 00")),
        (0x04006304, bytes.fromhex("80 00 00 00 00 00 00 00")),
    ]


def test_legacy_three_motor_speed_command_clears_second_brush():
    """Keep three-value speed messages compatible with the four-motor driver."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)

    class Message:
        """Minimal speed command message."""

        def __init__(self, data):
            self.data = data

    driver.speed_command_callback(Message([1.0, 2.0, 3.0]))
    assert [motor["velocity"] for motor in driver.motors] == [1.0, 2.0, 3.0, 0.0]

    driver.speed_command_callback(Message([4.0, 5.0, 6.0, 7.0]))
    assert [motor["velocity"] for motor in driver.motors] == [4.0, 5.0, 6.0, 7.0]


def test_legacy_feedback_and_fault_parsing_uses_low_byte_motor_id():
    """Decode type-2 feedback and type-15 fault frames by motor ID."""
    module = _load_motor_driver_module()
    driver = _make_driver(module)
    driver.parse_motor_feedback(
        0x02006302,
        bytes.fromhex("00 00 7f ff ff ff 01 f4"),
    )
    motor = driver.motors[1]
    assert abs(motor["actual_position"] + 12.57) < 1e-6
    assert abs(motor["actual_velocity"] + (44.0 / 65535.0)) < 1e-6
    assert abs(motor["actual_torque"] - 17.0) < 1e-6
    assert motor["actual_temperature"] == 50.0

    driver.parse_motor_fault(
        0x15006303,
        bytes.fromhex("08 00 00 00 00 00 00 00"),
    )
    assert driver.motors[2]["fault_code"] == 0x08
