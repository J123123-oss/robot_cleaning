"""Offline tests for the AIMOTOR CANopen driver."""

import ast
import importlib.util
import sys
import threading
import types
from pathlib import Path
from unittest.mock import Mock, patch

import pytest


@pytest.mark.parametrize(
    "configured,overrides,expected",
    [
        (
            {1: -1, 2: -1, 3: -1, 4: 1},
            {1: 1, 2: 1, 3: 1, 4: -1},
            {1: -1, 2: -1, 3: -1, 4: 1},
        ),
        (None, {1: -1, 2: 1, 3: -1, 4: 1}, {1: -1, 2: 1, 3: -1, 4: 1}),
        ({3: -1}, {1: -1, 2: -1, 3: 1, 4: -1},
         {1: -1, 2: -1, 3: -1, 4: -1}),
        ({}, {}, {1: 1, 2: 1, 3: 1, 4: 1}),
    ],
)
def test_constructor_direction_precedence_preserves_explicit_configuration(
    configured, overrides, expected
):
    module = _load_module()
    path = Path(__file__).parents[1] / "motor_control" / "motor_driver(AIMotor).py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node = next(item for item in tree.body
                if isinstance(item, ast.ClassDef) and item.name == "CanMotorDriver")
    initializer = next(item for item in node.body
                       if isinstance(item, ast.FunctionDef) and item.name == "__init__")
    start = next(
        index for index, item in enumerate(initializer.body)
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Name)
                and target.id == "configured_motor_directions"
                for target in item.targets)
    )
    end = next(
        index for index, item in enumerate(initializer.body)
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Attribute) and target.attr == "motors"
                for target in item.targets)
    )
    parameters = {}

    def declare_parameter(name, default, *, ignore_override=False):
        motor_id = int(name.rsplit("_", 1)[1])
        parameters[name] = (
            default if ignore_override else overrides.get(motor_id, default)
        )

    driver = types.SimpleNamespace(
        motor_ids=(1, 2, 3, 4), DEFAULT_MOTOR_DIRECTION=1,
        declare_parameter=declare_parameter,
        get_parameter=lambda name: types.SimpleNamespace(value=parameters[name]),
        _validate_motor_direction=module.CanMotorDriver._validate_motor_direction,
    )
    exec(
        compile(ast.Module(body=initializer.body[start:end], type_ignores=[]),
                str(path), "exec"),
        {"self": driver, "motor_directions": configured},
    )

    assert driver.motor_directions == expected
    assert parameters == {
        f"motor_direction_{motor_id}": direction
        for motor_id, direction in expected.items()
    }


def _load_module():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    rclpy.node = rclpy_node
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    rclpy_impl = types.ModuleType("rclpy.impl")
    rclpy_singleton = types.ModuleType("rclpy.impl.implementation_singleton")
    rclpy_singleton.rclpy_implementation = types.SimpleNamespace(
        RCLError=type("RCLError", (Exception,), {}),
        InvalidHandle=type("InvalidHandle", (Exception,), {}),
    )
    rclpy._test_implementation = rclpy_singleton.rclpy_implementation

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32MultiArray = type("Float32MultiArray", (), {})
    std_msgs_msg.Float64MultiArray = type("Float64MultiArray", (), {})
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
        "rclpy.executors": rclpy_executors,
        "rclpy.impl": rclpy_impl,
        "rclpy.impl.implementation_singleton": rclpy_singleton,
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
    driver.motor_ids = (1, 2, 3)
    driver.profile_acceleration = 60000
    driver.mechanical_reduction_ratios = {
        1: 50.0,
        2: 50.0,
        3: 40.0,
        4: 40.0,
    }
    driver.motor_directions = {1: 1, 2: 1, 3: 1, 4: 1}
    driver._feedback_lock = threading.RLock()
    driver.motors = [
        {
            "id": motor_id,
            "velocity": 0.0,
            "actual_velocity": 0.0,
            "actual_position": 0.0,
            "actual_torque": 0.0,
            "actual_temperature": 0.0,
            "actual_current": 0.0,
            "actual_current_updated_monotonic": 0.0,
            "status_word": 0,
            "mode_display": 0,
            "fault_code": 0,
            "manufacturer_fault_code": 0,
            "fault_history_query_code": None,
            "fault_history_query_monotonic": 0.0,
            "fault_history_resolved_code": None,
            "fault_history_pending_code": None,
            "send_errors": 0,
            "online": True,
        }
        for motor_id in (1, 2, 3)
    ]
    driver.motor_fault_publisher = Mock()
    driver.motor_feedback_publisher = Mock()
    driver._feedback_poll_requested = threading.Event()
    driver._feedback_log_last_time = {}
    driver._feedback_log_last_signature = {}
    driver.get_logger = Mock(return_value=Mock())
    return driver


def test_feedback_snapshot_is_detached_from_driver_cache():
    module = _load_module()
    driver = _make_driver(module)
    driver.motors[0]["actual_velocity"] = 12.5

    snapshot = driver.get_feedback_snapshot()
    snapshot[0]["actual_velocity"] = 0.0

    assert driver.motors[0]["actual_velocity"] == 12.5
    assert snapshot[0]["id"] == 1


def test_poll_feedback_only_publishes_snapshot_without_querying():
    module = _load_module()
    driver = _make_driver(module)
    driver.query_motor_feedback = Mock()
    driver.motors[0]["actual_velocity"] = 8.0

    snapshot = driver.poll_feedback()

    driver.query_motor_feedback.assert_not_called()
    assert snapshot[0]["actual_velocity"] == 8.0
    driver.motor_feedback_publisher.publish.assert_called_once()


def test_repeated_feedback_log_is_rate_limited():
    module = _load_module()
    driver = _make_driver(module)
    driver.parse_motor_feedback(
        0x581, bytes.fromhex("43 6c 60 00 e0 b1 ff ff")
    )
    driver.parse_motor_feedback(
        0x581, bytes.fromhex("43 6c 60 00 e0 b1 ff ff")
    )

    messages = [
        call.args[0]
        for call in driver.get_logger.return_value.method_calls
        if call.args and "电机1速度" in call.args[0]
    ]
    assert len(messages) == 1


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


def test_can_sender_queue_serializes_and_prioritizes_control_frames():
    module = _load_module()
    driver = _make_driver(module)
    driver.can_tx_interval_sec = 0.0
    driver._can_tx_condition = threading.Condition()
    driver._can_tx_heap = []
    driver._can_tx_sequence = module.itertools.count()
    driver._can_tx_stop = threading.Event()
    driver._can_tx_thread = None
    driver._can_tx_last_send_time = 0.0
    sent = []

    def capture_frame(can_id, data):
        sent.append((can_id, data))
        return True

    driver._send_can_frame_direct = capture_frame
    try:
        # Queue both before waking the worker so priority, not submission
        # timing, determines which class of frame is sent first.
        requests = []
        for can_id, data in (
            (0x601, bytes.fromhex("40 6c 60 00 00 00 00 00")),
            (0x601, bytes.fromhex("23 ff 60 00 01 00 00 00")),
        ):
            request = {
                "can_id": can_id,
                "data": data,
                "done": threading.Event(),
                "result": False,
            }
            requests.append(request)
            with driver._can_tx_condition:
                module.heapq.heappush(
                    driver._can_tx_heap,
                    (
                        driver._can_tx_priority(data),
                        next(driver._can_tx_sequence),
                        request,
                    ),
                )
        driver.start_can_sender()
        with driver._can_tx_condition:
            driver._can_tx_condition.notify_all()
        assert all(request["done"].wait(1.0) for request in requests)
    finally:
        driver.stop_can_sender()

    assert sent[0][1][0] == 0x23
    assert sent[1][1][0] == module.CanMotorDriver.SDO_READ


def test_remote_request_keeps_rtr_and_zero_dlc_through_sender_queue():
    module = _load_module()
    driver = _make_driver(module)
    driver.can_tx_interval_sec = 0.0
    driver._can_tx_condition = threading.Condition()
    driver._can_tx_heap = []
    driver._can_tx_sequence = module.itertools.count()
    driver._can_tx_stop = threading.Event()
    driver._can_tx_thread = None
    driver._can_tx_last_send_time = 0.0
    module.can.Message = lambda **options: types.SimpleNamespace(**options)
    driver.bus = Mock()
    try:
        driver.start_can_sender()
        assert driver.send_can_frame(0x703, b"", is_remote_frame=True)
    finally:
        driver.stop_can_sender()
    message = driver.bus.send.call_args.args[0]
    assert message.arbitration_id == 0x703
    assert message.is_remote_frame is True
    assert message.is_extended_id is False
    assert message.data == b""


def test_initialization_failure_disables_all_selected_nodes():
    module = _load_module()
    driver = _make_driver(module)
    driver.send_nmt_command = Mock(return_value=True)
    driver.motor_set_mode = Mock(side_effect=[True, False])
    driver.motor_set_direction = Mock(return_value=True)
    driver.motor_set_acceleration = Mock(return_value=True)
    driver.motor_set_deceleration = Mock(return_value=True)
    driver.motor_set_speed = Mock(return_value=True)
    driver.motor_enable = Mock(return_value=True)
    driver.motor_disable = Mock(return_value=True)
    driver.prepare_motor_initialization = Mock(return_value=True)
    with patch("time.sleep"):
        assert not driver.initialize_motors(motor_ids=(1, 2))
    driver.motor_enable.assert_called_once_with(1)
    assert {call.args[0] for call in driver.motor_disable.call_args_list} == {1, 2}


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
        (0x601, bytes.fromhex("23 84 60 00 48 ca 06 00")),
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
    driver.prepare_motor_initialization = Mock(return_value=True)

    driver.motor_set_speed = lambda motor_id, speed: True
    driver.initialize_motors()

    assert events[:6] == [
        ("nmt", 1),
        ("mode", 1),
        ("direction", 1),
        ("accel", 1),
        ("decel", 1),
        ("enable", 1),
    ]


def test_selected_startup_nodes_are_zeroed_before_enable():
    module = _load_module()
    driver = _make_driver(module)
    driver.motor_ids = (1, 2, 3)
    driver.motors[0]["velocity"] = 20.0
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True
    driver.motor_set_deceleration = Mock(return_value=True)
    driver._read_sdo_response = Mock(
        side_effect=[
            bytes.fromhex(f"4B 41 60 00 {status} 00 00 00")
            for status in ("40", "40", "27", "40", "40", "27")
        ]
    )
    with patch("time.sleep"):
        assert driver.initialize_motors(motor_ids=(1, 2))
    assert all(can_id != 0x603 for can_id, _ in frames)
    for motor_id in (1, 2):
        requests = [data for can_id, data in frames if can_id == 0x600 + motor_id]
        zero = bytes.fromhex("23 FF 60 00 00 00 00 00")
        enable = bytes.fromhex("2B 40 60 00 0F 00 00 00")
        assert requests.index(zero) < requests.index(enable)


def test_enable_disable_and_fault_reset_use_cia402_controlword():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True
    driver._read_sdo_response = Mock(
        side_effect=[
            bytes.fromhex("4B 41 60 00 40 00 00 00"),
            bytes.fromhex("4B 41 60 00 27 00 00 00"),
        ]
    )

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


def test_enable_keeps_live_operation_enabled_state_without_shutdown_sequence():
    module = _load_module()
    driver = _make_driver(module)
    driver.motors[0]["status_word"] = 0
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("4b 41 60 00 37 06 00 00")
    )
    driver.send_can_frame = Mock(return_value=True)

    assert driver.motor_enable(1)
    driver._read_sdo_response.assert_called_once_with(
        1, driver.STATUSWORD_INDEX, 0, driver.SDO_READ_2_RESPONSE,
        timeout_sec=0.5,
    )
    driver.send_can_frame.assert_not_called()


def test_enable_rejects_missing_live_status_even_if_cached_status_is_enabled():
    module = _load_module()
    driver = _make_driver(module)
    driver.motors[0]["status_word"] = 0x0237
    driver._read_sdo_response = Mock(return_value=None)
    driver.send_can_frame = Mock(return_value=True)

    assert not driver.motor_enable(1)
    driver.send_can_frame.assert_not_called()


def test_live_disabled_status_overrides_enabled_cache_and_runs_enable_sequence():
    module = _load_module()
    driver = _make_driver(module)
    driver.motors[0]["status_word"] = 0x0237
    driver._read_sdo_response = Mock(
        side_effect=[
            bytes.fromhex("4B 41 60 00 50 02 00 00"),
            bytes.fromhex("4B 41 60 00 37 02 00 00"),
        ]
    )
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append(data) or True

    assert driver.motor_enable(1)
    assert [int.from_bytes(data[4:6], "little") for data in frames] == [6, 7, 15]


def test_enable_fails_when_operation_enabled_status_is_not_confirmed():
    module = _load_module()
    driver = _make_driver(module)
    driver.send_can_frame = Mock(return_value=True)
    driver._read_sdo_response = Mock(side_effect=[
        bytes.fromhex("4B 41 60 00 40 00 00 00"), None,
    ])

    assert not driver.motor_enable(1)
    assert driver._read_sdo_response.call_count == 2


def test_status_confirmation_times_out_when_drive_stays_ready_instead_of_enabled():
    module = _load_module()
    driver = _make_driver(module)
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("4B 41 60 00 21 00 00 00")
    )
    with patch("time.monotonic", side_effect=[0.0, 0.1, 0.2, 0.6]), patch("time.sleep"):
        assert not driver.wait_motor_status(1, 0x006F, 0x0027)
    assert driver._read_sdo_response.call_count == 1


def test_initialization_resets_latched_fault_before_configuring_and_confirms_enable():
    module = _load_module()
    driver = _make_driver(module)
    driver.profile_deceleration = 445000
    frames = []
    status = 0x0008

    def send(can_id, data):
        nonlocal status
        frames.append((can_id, bytes(data)))
        if data[:4] == bytes.fromhex("2B 40 60 00"):
            controlword = int.from_bytes(data[4:6], "little")
            status = {0: 0x40, 0x80: 0x40, 6: 0x21, 7: 0x23, 15: 0x27}[controlword]
        return True

    def readback(motor_id, index, subindex, expected_command, **kwargs):
        assert index == driver.STATUSWORD_INDEX
        return bytes.fromhex("4B 41 60 00") + status.to_bytes(2, "little") + bytes(2)

    driver.send_can_frame = send
    driver._read_sdo_response = Mock(side_effect=readback)
    with patch("time.sleep"):
        assert driver.initialize_motors(motor_ids=(1,))
    reset = (0x601, bytes.fromhex("2B 40 60 00 80 00 00 00"))
    mode = (0x601, bytes.fromhex("2F 60 60 00 03 00 00 00"))
    assert reset in frames
    assert frames.index(reset) < frames.index(mode)
    assert frames[0] == (0x601, bytes.fromhex("23 FF 60 00 00 00 00 00"))
    assert status == 0x27
    assert frames.count(reset) == 1


def test_unconfirmed_fault_reset_prevents_configuration_and_enable():
    module = _load_module()
    driver = _make_driver(module)
    driver.profile_deceleration = 445000
    driver.send_can_frame = Mock(return_value=True)
    driver._read_sdo_response = Mock(side_effect=[
        bytes.fromhex("4B 41 60 00 08 00 00 00"), None,
    ])
    driver.motor_set_mode = Mock(return_value=True)
    driver.motor_enable = Mock(return_value=True)

    with patch("time.sleep"):
        assert not driver.initialize_motors(motor_ids=(1,))
    driver.motor_set_mode.assert_not_called()
    driver.motor_enable.assert_not_called()


def test_receive_timeout_does_not_send_disable_or_reset_frames():
    module = _load_module()
    driver = _make_driver(module)
    driver.running = True
    driver.can_initialized = True
    driver.bus = Mock()
    driver.send_can_frame = Mock()
    polls = []

    def receive_timeout(timeout):
        polls.append(timeout)
        if len(polls) == 10:
            driver.running = False
        return None

    driver.bus.recv.side_effect = receive_timeout
    driver.receive_can_frames()

    assert len(polls) == 10
    driver.send_can_frame.assert_not_called()
    driver.bus.shutdown.assert_not_called()


@pytest.mark.parametrize("exception_name", ["RCLError", "InvalidHandle"])
@pytest.mark.parametrize("context_active", [False, True])
def test_receive_distinguishes_shutdown_from_live_publish_errors(
    exception_name, context_active
):
    module = _load_module()
    driver = _make_driver(module)
    driver.running = True
    driver.can_initialized = True
    driver.bus = Mock()
    module.rclpy.ok = Mock(return_value=context_active)
    message = types.SimpleNamespace(arbitration_id=0x581, data=bytes(8))

    def receive(timeout):
        driver.running = False
        return message

    driver.bus.recv.side_effect = receive
    driver.parse_motor_feedback = Mock(side_effect=getattr(
        module.rclpy._test_implementation, exception_name
    )("publisher's context is invalid"))
    logger = driver.get_logger()
    with patch("time.sleep") as sleep:
        driver.receive_can_frames()

    assert logger.error.call_count == int(context_active)
    driver.bus.shutdown.assert_not_called()
    if context_active:
        sleep.assert_called_once_with(1.0)
    else:
        sleep.assert_not_called()
    assert driver.can_initialized


def test_restart_stops_at_zero_and_reuses_aimotor_enable_without_controlword_drop():
    module = _load_module()
    driver = _make_driver(module)
    driver.running = True
    driver._stopping = False
    driver.stop_feedback_thread = Mock()
    driver.profile_deceleration = module.CanMotorDriver.DEFAULT_PROFILE_DECELERATION
    driver.motors[0]["velocity"] = 20.0
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("4b 41 60 00 37 02 00 00")
    )
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    driver.stop_all_motors(preserve_aimotor_enable=True)
    assert all(motor["velocity"] == 0.0 for motor in driver.motors)
    assert len(frames) == len(driver.motors)
    assert all(data == bytes.fromhex("23 ff 60 00 00 00 00 00")
               for _, data in frames)
    driver._stopping = False
    driver.running = True
    with patch("time.sleep"):
        assert driver.initialize_motors()
    assert not any(data[1:3] == bytes.fromhex("40 60") for _, data in frames)


def test_explicit_stop_still_disables_aimotor_and_failed_zero_cannot_keep_enable():
    module = _load_module()
    for preserve_enabled, speed_result in ((False, True), (True, False)):
        driver = _make_driver(module)
        driver._stopping = False
        driver.stop_feedback_thread = Mock()
        driver.motor_set_speed = Mock(return_value=speed_result)
        driver.motor_disable = Mock(return_value=True)

        driver.stop_all_motors(preserve_aimotor_enable=preserve_enabled)

        assert driver.motor_disable.call_count == len(driver.motors)


def test_driver_main_distinguishes_shutdown_signals_from_fatal_errors():
    module = _load_module()
    implementation = module.rclpy._test_implementation
    cases = (
        (KeyboardInterrupt(), False, False),
        (module.ExternalShutdownException(), False, False),
        (RuntimeError("driver failed"), False, True),
        (implementation.RCLError("publisher's context is invalid"), False, False),
        (implementation.InvalidHandle("destruction was requested"), False, False),
        (implementation.RCLError("publish failed"), True, True),
        (implementation.InvalidHandle("invalid handle"), True, True),
    )
    for exit_reason, context_active, should_disable in cases:
        driver = Mock()
        module.CanMotorDriver = Mock(return_value=driver)
        module.rclpy.init = Mock()
        module.rclpy.spin = Mock(side_effect=exit_reason)
        module.rclpy.ok = Mock(return_value=context_active)
        module.rclpy.shutdown = Mock()

        module.main()

        driver.destroy_node.assert_called_once()
        if should_disable:
            driver.stop_all_motors.assert_called_once_with()
        else:
            driver.stop_all_motors.assert_not_called()


def test_mode_and_feedback_queries_use_documented_objects():
    module = _load_module()
    driver = _make_driver(module)
    frames = []
    driver.send_can_frame = lambda can_id, data: frames.append((can_id, data)) or True

    assert driver.motor_set_mode(1, 2)
    assert frames[-1] == (0x601, bytes.fromhex("2f 60 60 00 03 00 00 00"))

    assert driver.motor_query_feedback(1)
    assert [frame[1] for frame in frames[-6:]] == [
        bytes.fromhex("40 41 60 00 00 00 00 00"),
        bytes.fromhex("40 64 60 00 00 00 00 00"),
        bytes.fromhex("40 6c 60 00 00 00 00 00"),
        bytes.fromhex("40 77 60 00 00 00 00 00"),
        bytes.fromhex("40 78 60 00 00 00 00 00"),
        bytes.fromhex("40 3f 60 00 00 00 00 00"),
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


def test_sdo_current_and_fault_objects_are_decoded_from_manual():
    module = _load_module()
    driver = _make_driver(module)
    driver.can_initialized = True
    driver.bus = object()
    driver._sdo_read = Mock(return_value=True)

    driver.parse_motor_feedback(
        0x581, bytes.fromhex("4b 78 60 00 06 ff 00 00")
    )
    assert driver.motors[0]["actual_current"] == -2.5
    assert driver.motors[0]["actual_current_updated_monotonic"] > 0.0

    driver.parse_motor_feedback(
        0x581, bytes.fromhex("4b 3f 60 00 12 23 00 00")
    )
    assert driver.motors[0]["fault_code"] == 0x2312
    assert driver.motors[0]["fault_history_pending_code"] == 0x2312
    assert driver._feedback_poll_requested.is_set()
    driver._sdo_read.assert_not_called()

    driver._process_pending_fault_history_queries()
    driver._sdo_read.assert_called_once_with(1, 0x1003, 1)

    driver.parse_motor_feedback(
        0x581, bytes.fromhex("43 03 10 01 12 23 01 02")
    )
    assert driver.motors[0]["manufacturer_fault_code"] == 0x0201
    assert driver.motor_fault_publisher.publish.call_count == 2


def test_feedback_topic_appends_current_age_and_vendor_fault_code():
    module = _load_module()
    driver = _make_driver(module)
    driver.motors[0]["actual_current"] = 1.25
    driver.motors[0]["actual_current_updated_monotonic"] = module.time.monotonic()
    driver.motors[0]["manufacturer_fault_code"] = 0x0201

    driver.poll_feedback()

    published = driver.motor_feedback_publisher.publish.call_args.args[0].data
    assert len(published) == len(driver.motors) * 8
    assert published[:5] == [1.0, 0.0, 0.0, 0.0, 0.0]
    assert published[5] == 1.25
    assert published[6] >= 0.0
    assert published[7] == 0x0201


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
