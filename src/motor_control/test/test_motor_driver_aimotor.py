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
    driver.pulses_per_motor_rev = 1000
    driver.encoder_pulses_per_rev = 1000
    driver.mechanical_reduction_ratios = {
        1: 50.0,
        2: 50.0,
        3: 40.0,
        4: 40.0,
    }
    driver.motor_directions = {1: 1, 2: 1, 3: 1, 4: 1}
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

    assert driver.motor_set_speed(1, 20.0)
    assert frames == [(
        0x601,
        bytes.fromhex("23 ff 60 00 1b 41 00 00"),
    )]

    # 周期写入仍然执行，但相同设定不应重复刷 INFO 日志。
    assert driver.motor_set_speed(1, 20.0)
    motor_one_logs = [
        call.args[0]
        for call in driver.get_logger.return_value.method_calls
        if call.args and "电机1设定" in call.args[0]
    ]
    assert len(motor_one_logs) == 1
    assert len(frames) == 2

    frames.clear()
    assert driver.motor_set_speed(3, 20.0)
    assert frames == [(
        0x603,
        bytes.fromhex("23 ff 60 00 15 34 00 00"),
    )]
    messages = [
        call.args[0]
        for call in driver.get_logger.return_value.method_calls
        if call.args
    ]
    assert any(
        "目标转速=+20.000 r/min（输出轴）" in message
        and "设置脉冲数=16667 Pul/s" in message
        for message in messages
    )

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


def test_profile_acceleration_and_deceleration_use_documented_objects():
    module = _load_module()
    driver = _make_driver(module)
    driver.profile_acceleration = module.CanMotorDriver.DEFAULT_PROFILE_ACCELERATION
    driver.profile_deceleration = module.CanMotorDriver.DEFAULT_PROFILE_DECELERATION
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_set_acceleration(1)
    assert driver.motor_set_deceleration(1)
    assert frames == [
        (0x601, bytes.fromhex("23 83 60 00 35 82 00 00")),
        (0x601, bytes.fromhex("23 84 60 00 35 82 00 00")),
    ]


def test_velocity_direction_uses_607e_bit6():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_set_direction(1, 1)
    assert frames[-1] == (0x601, bytes.fromhex("2f 7e 60 00 00 00 00 00"))

    assert driver.motor_set_direction(1, -1)
    assert frames[-1] == (0x601, bytes.fromhex("2f 7e 60 00 40 00 00 00"))


def test_initialization_sets_mode_and_ramps_before_enable():
    module = _load_module()
    driver = _make_driver(module)
    events = []
    module.time.sleep = lambda _duration: None
    driver.send_nmt_command = lambda command, motor_id: events.append(("nmt", motor_id)) or True
    driver.motor_set_mode = lambda motor_id, mode: events.append(("mode", motor_id)) or True
    driver.motor_set_direction = lambda motor_id: events.append(("direction", motor_id)) or True
    driver.motor_set_acceleration = lambda motor_id: events.append(("accel", motor_id)) or True
    driver.motor_set_deceleration = lambda motor_id: events.append(("decel", motor_id)) or True
    driver.motor_enable = lambda motor_id: events.append(("enable", motor_id)) or True

    driver.initialize_motors()

    assert events[:6] == [
        ("nmt", 1),
        ("mode", 1),
        ("direction", 1),
        ("accel", 1),
        ("decel", 1),
        ("enable", 1),
    ]


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
    assert frames[-1] == (0x601, bytes.fromhex("2b 40 60 00 00 00 00 00"))
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
    driver.motors[0]["velocity"] = 20.0

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
    assert motor["actual_velocity"] == -24.0
    assert motor["actual_torque"] == 2.5

    driver.parse_motor_fault(0x081, bytes.fromhex("12 23 08 00 00 00 00 00"))
    assert motor["fault_code"] == 0x2312
    driver.motor_fault_publisher.publish.assert_called_once()
    messages = [
        call.args[0]
        for call in driver.get_logger.return_value.method_calls
        if call.args
    ]
    assert any("硬件过流" in message for message in messages)
    assert any("H0B-34：0x0201" in message for message in messages)
    assert any(
        "设定转速=+20.000 r/min（输出轴）" in message
        and "实际转速=-24.000 r/min（输出轴）" in message
        and "实际反馈脉冲数=-20000 Pul/s" in message
        for message in messages
    )


def test_manual_fault_table_contains_all_documented_codes():
    module = _load_module()
    expected_codes = {
        (0x0000, 0x0000),
        (0x0101, 0x6320),
        (0x0102, 0x6320),
        (0x0104, 0x6320),
        (0x0105, 0x6320),
        (0x0130, 0x6320),
        (0x0201, 0x2312),
        (0x0208, 0xFF00),
        (0x0207, 0x2311),
        (0x0234, 0xFF00),
        (0x0A33, 0x7306),
        (0x0400, 0x3210),
        (0x0410, 0x3220),
        (0x0620, 0x3230),
        (0x0650, 0x4210),
        (0x0B00, 0x8611),
        (0x0668, 0xFF00),
        (0x0601, 0x8610),
        (0x0900, 0x5442),
        (0x0950, 0x5443),
        (0x0952, 0x5444),
        (0x0731, 0x7306),
        (0x0733, 0x7306),
        (0x0735, 0x7306),
        (0x0730, 0x7307),
        (0x0D03, 0x8130),
        (0x0941, 0xFF00),
        (0x0942, 0x7600),
    }

    table = module.CanMotorDriver.AIMOTOR_FAULT_TABLE
    assert len(table) == 28
    assert {(fault[0], fault[1]) for fault in table} == expected_codes


def test_emcy_parser_logs_all_candidates_for_non_unique_standard_code():
    module = _load_module()
    driver = _make_driver(module)

    driver.parse_motor_fault(
        0x081, bytes.fromhex("20 63 00 01 02 03 04 05")
    )

    assert driver.motors[0]["fault_code"] == 0x6320
    messages = [
        call.args[0]
        for call in driver.get_logger.return_value.method_calls
        if call.args
    ]
    assert any("无法仅凭 EMCY 帧唯一确定" in message for message in messages)
    for manufacturer_code in (0x0101, 0x0102, 0x0104, 0x0105, 0x0130):
        assert any(
            f"H0B-34=0x{manufacturer_code:04X}" in message
            for message in messages
        )
    assert any("厂家特定数据：01 02 03 04 05" in message for message in messages)


def test_emcy_parser_ignores_invalid_frames_without_publishing():
    module = _load_module()
    driver = _make_driver(module)
    driver.motors[0]["fault_code"] = 0x2312

    driver.parse_motor_fault(0x081, b"")
    driver.parse_motor_fault(0x081, bytes.fromhex("12 23"))
    driver.parse_motor_fault(0x123, bytes.fromhex("12 23 00"))

    assert driver.motors[0]["fault_code"] == 0x2312
    driver.motor_fault_publisher.publish.assert_not_called()


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
