"""Offline tests for the ROS1 AIMOTOR CANopen driver."""

import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import Mock


def _load_module():
    """Load the driver with ROS1 and python-can dependencies replaced."""
    rospy = types.ModuleType("rospy")
    rospy.Time = Mock()
    rospy.Duration = Mock()
    rospy.Publisher = Mock()
    rospy.Subscriber = Mock()
    rospy.Timer = Mock()
    rospy.get_param = Mock(side_effect=lambda _name, default=None: default)
    rospy.is_shutdown = Mock(return_value=False)
    rospy.ROSInterruptException = RuntimeError
    rospy.loginfo = Mock()
    rospy.logwarn = Mock()
    rospy.logerr = Mock()
    rospy.on_shutdown = Mock()
    rospy.spin = Mock()

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Quaternion = type("Quaternion", (), {})
    geometry_msgs.msg = geometry_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = type("Odometry", (), {})
    nav_msgs.msg = nav_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32MultiArray = type("Float32MultiArray", (), {})
    std_msgs.msg = std_msgs_msg

    can = types.ModuleType("can")
    can.Bus = object
    can.Message = object
    can.interface = types.SimpleNamespace(Bus=object)

    modules = {
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "can": can,
    }
    previous = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    try:
        path = (
            Path(__file__).parents[1]
            / "motor_control"
            / "motor_driver(AIMotor_ros1).py"
        )
        spec = importlib.util.spec_from_file_location(
            "motor_driver_aimotor_ros1", path
        )
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
    """Create only the protocol state needed by unit-level tests."""
    driver = module.CanMotorDriver.__new__(module.CanMotorDriver)
    driver.pulses_per_motor_rev = 1000
    driver.encoder_pulses_per_rev = 1000
    driver.mechanical_reduction_ratios = {
        1: 50.0,
        2: 50.0,
        3: 40.0,
        4: 40.0,
    }
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
    return driver


def test_ros1_driver_imports_without_rclpy_and_writes_standard_sdo_speed():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = (
        lambda can_id, data: frames.append((can_id, data)) or True
    )

    assert driver.motor_set_speed(1, 20.0)
    assert frames == [
        (0x601, bytes.fromhex("23 ff 60 00 1b 41 00 00")),
    ]

    frames.clear()
    assert driver.motor_set_speed(3, 20.0)
    assert frames == [
        (0x603, bytes.fromhex("23 ff 60 00 15 34 00 00")),
    ]


def test_ros1_driver_writes_profile_acceleration_and_deceleration():
    module = _load_module()
    driver = _make_driver(module)
    driver.profile_acceleration = module.CanMotorDriver.DEFAULT_PROFILE_ACCELERATION
    driver.profile_deceleration = module.CanMotorDriver.DEFAULT_PROFILE_DECELERATION
    frames = []
    driver.send_can_frame = (
        lambda can_id, data: frames.append((can_id, data)) or True
    )

    assert driver.motor_set_acceleration(1)
    assert driver.motor_set_deceleration(1)
    assert frames == [
        (0x601, bytes.fromhex("23 83 60 00 1a 41 00 00")),
        (0x601, bytes.fromhex("23 84 60 00 67 2b 00 00")),
    ]


def test_ros1_driver_initialization_sets_ramps_before_enable():
    module = _load_module()
    driver = _make_driver(module)
    events = []
    module.time.sleep = lambda _duration: None
    driver.send_nmt_command = lambda command, motor_id: events.append(("nmt", motor_id)) or True
    driver.motor_set_mode = lambda motor_id, mode: events.append(("mode", motor_id)) or True
    driver.motor_set_acceleration = lambda motor_id: events.append(("accel", motor_id)) or True
    driver.motor_set_deceleration = lambda motor_id: events.append(("decel", motor_id)) or True
    driver.motor_enable = lambda motor_id: events.append(("enable", motor_id)) or True

    driver.initialize_motors()

    assert events[:5] == [
        ("nmt", 1),
        ("mode", 1),
        ("accel", 1),
        ("decel", 1),
        ("enable", 1),
    ]


def test_ros1_driver_enable_mode_and_feedback_queries_keep_public_api():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = (
        lambda can_id, data: frames.append((can_id, data)) or True
    )

    assert driver.motor_set_mode(1, 2)
    assert frames[-1] == (0x601, bytes.fromhex("2f 60 60 00 03 00 00 00"))

    assert driver.motor_enable(1)
    assert [frame[1] for frame in frames[-3:]] == [
        bytes.fromhex("2b 40 60 00 06 00 00 00"),
        bytes.fromhex("2b 40 60 00 07 00 00 00"),
        bytes.fromhex("2b 40 60 00 0f 00 00 00"),
    ]

    assert driver.motor_query_feedback(1)
    assert [frame[1] for frame in frames[-4:]] == [
        bytes.fromhex("40 41 60 00 00 00 00 00"),
        bytes.fromhex("40 64 60 00 00 00 00 00"),
        bytes.fromhex("40 6c 60 00 00 00 00 00"),
        bytes.fromhex("40 77 60 00 00 00 00 00"),
    ]


def test_ros1_driver_decodes_sdo_tpdo_and_emcy_feedback():
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
    assert motor["actual_velocity"] == -24.0
    assert motor["actual_torque"] == 2.5

    driver.parse_motor_feedback(
        0x281, bytes.fromhex("e0 b1 ff ff 19 00")
    )
    assert driver.motors[0]["actual_velocity"] == -24.0
    assert driver.motors[0]["actual_torque"] == 2.5

    driver.parse_motor_fault(0x081, bytes.fromhex("34 12 08 00 00 00 00 00"))
    assert motor["fault_code"] == 0x1234
    driver.motor_fault_publisher.publish.assert_called_once()


def test_ros1_driver_does_not_send_obsolete_vendor_parameter_frames():
    module = _load_module()
    driver = _make_driver(module)
    driver.send_can_frame = Mock()

    assert driver.motor_set_current_limit(1, 20.0) is False
    assert driver.motor_set_other_param(1, 15.0) is False
    driver.send_can_frame.assert_not_called()
