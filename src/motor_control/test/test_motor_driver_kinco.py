"""Offline checks against the Kinco iSMK manual's CANopen examples."""

import ast
import importlib.util
import json
import math
import runpy
import threading
import types
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from test_motor_driver_aimotor import _load_module, _make_driver


def load_kinco():
    aimotor = _load_module()
    path = Path(__file__).parents[1] / "motor_control" / "motor_driver_kinco.py"
    spec = importlib.util.spec_from_file_location("kinco_under_test", path)
    module = importlib.util.module_from_spec(spec)
    messages = types.ModuleType("std_msgs.msg")
    messages.Float32MultiArray = aimotor.Float32MultiArray
    messages.String = type("String", (), {})
    with patch.dict("sys.modules", {"std_msgs.msg": messages}), patch(
        "importlib.import_module", return_value=aimotor
    ):
        spec.loader.exec_module(module)
    return module, aimotor


def make_driver():
    module, aimotor = load_kinco()
    driver = module.CanMotorDriver.__new__(module.CanMotorDriver)
    driver.__dict__.update(_make_driver(aimotor).__dict__)
    driver.motors.append(dict(driver.motors[2], id=4))
    driver.motor_ids = (1, 2, 3, 4)
    driver.kinco_encoder_resolution = 65536
    driver.kinco_reduction_ratio = 1.0
    driver.kinco_acceleration_rps2 = 100.0
    driver.kinco_deceleration_rps2 = 100.0
    driver.kinco_peak_current_a = 47.9
    driver.motor_velocity_feedback_publisher = Mock()
    driver.profile_acceleration = 60000
    driver.profile_deceleration = 445000
    driver.drive_motor_ids = (1, 2)
    driver._feedback_thread = None
    driver._sdo_transaction_lock = threading.Lock()
    driver._kinco_guard_nodes = {}
    driver.configure_kinco_guard = Mock(return_value=True)
    frames = []
    driver.send_can_frame = (
        lambda can_id, data: frames.append((can_id, bytes(data))) or True
    )
    return driver, frames


def test_speed_matches_manual_negative_100_rpm_example():
    driver, frames = make_driver()
    assert driver.motor_set_speed(3, -100)
    assert frames == [(0x603, bytes.fromhex("23 FF 60 00 7E B1 E4 FF"))]
    assert driver.motor_set_speed(4, 100)
    assert frames[-1] == (0x604, bytes.fromhex("23 FF 60 00 82 4E 1B 00"))
    assert driver.motor_set_speed(1, 60)
    assert frames[-1] == (0x601, bytes.fromhex("23 FF 60 00 50 C3 00 00"))


def test_shutdown_preserves_only_aimotor_enable_and_disables_kinco():
    driver, frames = make_driver()
    driver._stopping = False
    driver.running = True
    driver.stop_feedback_thread = Mock()

    driver.stop_all_motors(preserve_aimotor_enable=True)

    controlwords = [(can_id, data) for can_id, data in frames
                    if data[1:3] == bytes.fromhex("40 60")]
    assert controlwords == [
        (0x603, bytes.fromhex("2B 40 60 00 06 00 00 00")),
        (0x604, bytes.fromhex("2B 40 60 00 06 00 00 00")),
    ]
    assert all(motor["velocity"] == 0.0 for motor in driver.motors)


def test_guard_configuration_requires_readback_and_initial_remote_request():
    driver, frames = make_driver()
    del driver.configure_kinco_guard
    driver.kinco_guard_time_ms = 100
    driver.kinco_guard_life_factor = 5
    driver._kinco_guard_nodes = {}
    driver._read_sdo_response = Mock(side_effect=[
        bytes.fromhex("4B 17 10 00 00 00 00 00"),
        bytes.fromhex("4B 0C 10 00 64 00 00 00"),
        bytes.fromhex("4F 0D 10 00 05 00 00 00"),
        bytes.fromhex("4B 07 60 00 01 00 00 00"),
    ])
    driver.send_can_frame = Mock(return_value=True)
    assert driver.configure_kinco_guard(3)
    assert driver.send_can_frame.call_args.kwargs == {"is_remote_frame": True}
    assert driver.send_can_frame.call_args.args == (0x703, b"")
    assert 3 in driver._kinco_guard_nodes
    writes = [call.args for call in driver.send_can_frame.call_args_list[:-1]]
    assert writes == [
        (0x603, bytes.fromhex("2B 17 10 00 00 00 00 00")),
        (0x603, bytes.fromhex("2B 0C 10 00 64 00 00 00")),
        (0x603, bytes.fromhex("2F 0D 10 00 05 00 00 00")),
        (0x603, bytes.fromhex("2B 07 60 00 01 00 00 00")),
    ]


@pytest.mark.parametrize("response", [None, bytes.fromhex("4B 0C 10 00 00 00 00 00")])
def test_guard_readback_failure_blocks_enable(response):
    driver, frames = make_driver()
    del driver.configure_kinco_guard
    driver.kinco_guard_time_ms = 100
    driver.kinco_guard_life_factor = 5
    driver._read_sdo_response = Mock(return_value=response)
    assert not driver.motor_enable(3)
    assert not any(data == bytes.fromhex("2B 40 60 00 0F 00 00 00")
                   for _, data in frames)


def test_guard_service_uses_remote_frames_only_for_armed_nodes_and_stops_on_exit():
    driver, _ = make_driver()
    driver.kinco_guard_time_ms = 100
    driver._kinco_guard_nodes = {3: 1.0}
    driver.running = True
    driver._stopping = False
    driver._send_can_frame_direct = Mock(return_value=True)
    with patch("time.monotonic", return_value=1.05):
        driver.service_periodic_can_frames()
    driver._send_can_frame_direct.assert_not_called()
    with patch("time.monotonic", return_value=1.11):
        driver.service_periodic_can_frames()
    driver._send_can_frame_direct.assert_called_once_with(
        0x703, b"", is_remote_frame=True
    )
    driver._stopping = True
    with patch("time.monotonic", return_value=2.0):
        driver.service_periodic_can_frames()
    assert driver._send_can_frame_direct.call_count == 1


def test_guard_runs_in_idle_sender_without_creating_an_extra_thread():
    driver, _ = make_driver()
    _, aimotor = load_kinco()
    driver.running = True
    driver._stopping = False
    driver.kinco_guard_time_ms = 100
    driver._kinco_guard_nodes = {4: 0.0}
    driver.can_tx_interval_sec = 0.0
    driver._can_tx_condition = threading.Condition()
    driver._can_tx_heap = []
    driver._can_tx_sequence = aimotor.itertools.count()
    driver._can_tx_stop = threading.Event()
    driver._can_tx_thread = None
    driver._can_tx_last_send_time = 0.0
    sent = threading.Event()
    threads = []

    def send_remote(can_id, data, is_remote_frame=False):
        assert (can_id, data, is_remote_frame) == (0x704, b"", True)
        threads.append(threading.current_thread())
        sent.set()
        return True

    driver._send_can_frame_direct = send_remote
    try:
        driver.start_can_sender()
        assert sent.wait(1.0)
        assert threads[0] is driver._can_tx_thread
    finally:
        driver.stop_can_sender()


def test_gearbox_and_encoder_are_independent_of_aimotor():
    driver, frames = make_driver()
    driver.kinco_encoder_resolution = 10000
    driver.kinco_reduction_ratio = 5.0
    assert driver.motor_set_speed(4, 60)
    assert frames[-1] == (0x604, bytes.fromhex("23 FF 60 00 00 80 0C 00"))
    driver.parse_motor_feedback(0x584, bytes.fromhex("43 6C 60 00 00 80 0C 00"))
    assert driver.motors[3]["actual_velocity"] == 60.0


def test_direction_disable_reset_and_ramps_match_kinco():
    driver, frames = make_driver()
    assert driver.motor_set_direction(4, -1)
    assert frames[-1] == (0x604, bytes.fromhex("2F 7E 60 00 01 00 00 00"))
    assert driver.motor_set_direction(3, 1)
    assert frames[-1] == (0x603, bytes.fromhex("2F 7E 60 00 00 00 00 00"))
    assert driver.motor_disable(3)
    assert frames[-1] == (0x603, bytes.fromhex("2B 40 60 00 06 00 00 00"))
    assert driver.motor_clear_fault(4)
    assert frames[-1] == (0x604, bytes.fromhex("2B 40 60 00 86 00 00 00"))
    assert driver.motor_set_acceleration(3)
    assert frames[-1] == (0x603, bytes.fromhex("23 83 60 00 6E A3 01 00"))
    assert driver.motor_set_deceleration(4)
    assert frames[-1] == (0x604, bytes.fromhex("23 84 60 00 6E A3 01 00"))
    assert driver.motor_disable(1)
    assert frames[-1] == (0x601, bytes.fromhex("2B 40 60 00 00 00 00 00"))


def test_kinco_velocity_mode_writes_three():
    driver, frames = make_driver()

    assert driver.motor_set_mode(3, 3)

    assert frames[-1] == (
        0x603,
        bytes.fromhex("2F 60 60 00 03 00 00 00"),
    )


def test_kinco_enable_follows_manual_sequence_and_leaves_operation_enabled():
    driver, frames = make_driver()

    assert driver.motor_enable(3)

    assert frames == [
        (0x603, bytes.fromhex("23 FF 60 00 00 00 00 00")),
        (0x603, bytes.fromhex("2B 40 60 00 06 00 00 00")),
        (0x603, bytes.fromhex("2B 40 60 00 07 00 00 00")),
        (0x603, bytes.fromhex("2B 40 60 00 0F 00 00 00")),
    ]


@pytest.mark.parametrize("speed", [float("nan"), float("inf"), 1e30, "bad"])
def test_invalid_speed_never_reaches_can(speed):
    driver, frames = make_driver()
    assert driver.motor_set_speed(3, speed) is False
    assert not frames


def test_feedback_only_accepts_uploads_and_uses_kinco_position_object():
    driver, _ = make_driver()
    driver.parse_motor_feedback(0x583, bytes.fromhex("43 6C 60 00 7E B1 E4 FF"))
    assert driver.motors[2]["actual_velocity"] == pytest.approx(-100, abs=0.001)
    driver.parse_motor_feedback(0x583, bytes.fromhex("60 6C 60 00 00 00 00 00"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("80 6C 60 00 00 00 00 00"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("43 6C 60 01 00 00 00 00"))
    driver.parse_motor_feedback(0x583, b"\x43")
    assert driver.motors[2]["actual_velocity"] == pytest.approx(-100, abs=0.001)
    driver.parse_motor_feedback(0x583, bytes.fromhex("43 63 60 00 00 00 01 00"))
    assert driver.motors[2]["actual_position"] == pytest.approx(2 * math.pi)
    driver.parse_motor_feedback(0x183, bytes.fromhex("37 02 03 E8 03 00 00"))
    assert driver.motors[2]["actual_position"] == pytest.approx(2 * math.pi)
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 78 60 00 2C 01 00 00"))
    assert driver.motors[2]["actual_current"] == pytest.approx(
        300.0 * 47.9 / (1.414 * 2048)
    )
    assert driver.motors[2]["actual_current_updated_monotonic"] > 0.0


def test_feedback_queries_kinco_objects_without_aimotor_history_or_torque():
    driver, frames = make_driver()
    assert driver.motor_query_feedback(3)
    assert frames == [
        (0x603, bytes.fromhex(value))
        for value in (
            "40 41 60 00 00 00 00 00",
            "40 61 60 00 00 00 00 00",
            "40 63 60 00 00 00 00 00",
            "40 6C 60 00 00 00 00 00",
            "40 3F 60 00 00 00 00 00",
            "40 01 26 00 00 00 00 00",
            "40 02 26 00 00 00 00 00",
            "40 10 64 19 00 00 00 00",
            "40 78 60 00 00 00 00 00",
        )
    ]


def test_kinco_debug_snapshot_publishes_units_raw_values_and_unread_fields():
    driver, frames = make_driver()
    driver.kinco_feedback_publisher = Mock()
    driver.kinco_reduction_ratio = 10.0
    driver.parse_motor_feedback(0x583, bytes.fromhex("43 6C 60 00 00 00 00 01"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 78 60 00 00 FC 00 00"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("4F 61 60 00 03 00 00 00"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 41 60 00 37 46 00 00"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 10 64 19 18 00 00 00"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("43 63 60 00 00 00 01 00"))
    driver.parse_motor_fault(0x084, bytes.fromhex("10 42 08 00 10 00 20 00"))
    driver.poll_feedback()
    payload = json.loads(driver.kinco_feedback_publisher.publish.call_args.args[0].data)
    assert payload["timestamp_unix_sec"] > 0
    assert [motor["id"] for motor in payload["motors"]] == [3, 4]
    motor = payload["motors"][0]
    assert motor["velocity_raw"] == 0x01000000
    assert motor["output_speed_rpm"] == pytest.approx(93.75)
    assert motor["motor_speed_rpm"] == pytest.approx(937.5)
    assert motor["current_raw"] == -1024
    assert motor["output_current_ap"] == pytest.approx(-23.95)
    assert motor["output_current_arms"] == pytest.approx(-23.95 / 1.414)
    assert motor["mode_display"] == 3
    assert motor["status_word"] == 0x4637
    assert motor["temperature_c"] == 24
    assert motor["motor_position_rad"] == pytest.approx(2 * math.pi)
    assert motor["sample_age_sec"]["output_current_arms"] >= 0
    assert motor["fault_code"] is None
    assert motor["sample_age_sec"]["fault_code"] is None
    assert payload["motors"][1]["output_speed_rpm"] is None
    assert payload["motors"][1]["error_status"] == 0x10
    assert payload["motors"][1]["error_status_2"] == 0x20
    assert frames == []
    driver.motor_feedback_publisher.publish.assert_called_once()


def test_kinco_debug_snapshot_ignores_invalid_responses_and_reports_aging():
    driver, _ = make_driver()
    with patch("time.monotonic", return_value=100.0):
        driver.parse_motor_feedback(0x583, bytes.fromhex("4F 61 60 00 FD 00 00 00"))
    for invalid in (
        "60 61 60 00 03 00 00 00",
        "80 61 60 00 03 00 00 00",
        "4F 61 60 01 03 00 00 00",
        "4B 61 60 00 03 00 00 00",
    ):
        driver.parse_motor_feedback(0x583, bytes.fromhex(invalid))
    with patch("time.monotonic", return_value=110.0):
        snapshot = driver.get_kinco_feedback_snapshot()
    assert snapshot[0]["mode_display"] == -3
    assert snapshot[0]["sample_age_sec"]["mode_display"] == 10.0
    driver.motors = driver.motors[:2]
    assert driver.get_kinco_feedback_snapshot() == []


def test_faults_keep_emergency_code_and_both_kinco_error_words():
    driver, _ = make_driver()
    driver.parse_motor_fault(0x083, bytes.fromhex("10 42 08 00 10 00 00 00"))
    assert driver.motors[2]["fault_code"] == 0x4210
    assert driver.motors[2]["kinco_error_status"] == 0x0010
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 01 26 00 01 00 00 00"))
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 02 26 00 00 10 00 00"))
    assert driver.motors[2]["kinco_error_status"] == 1
    assert driver.motors[2]["kinco_error_status_2"] == 0x1000
    assert driver.motors[2]["fault_code"] == 0x4210
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 3F 60 00 00 00 00 00"))
    assert driver.motors[2]["fault_code"] == 0


def test_initialization_disables_brushes_before_polarity_and_zeroes_before_enable():
    driver, frames = make_driver()
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("4B 41 60 00 40 00 00 00")
    )
    with patch("time.sleep"):
        driver.initialize_motors()
    for motor_id in (3, 4):
        requests = [data for can_id, data in frames if can_id == 0x600 + motor_id]
        disable = bytes.fromhex("2B 40 60 00 06 00 00 00")
        zero = bytes.fromhex("23 FF 60 00 00 00 00 00")
        switch_on = bytes.fromhex("2B 40 60 00 07 00 00 00")
        enable = bytes.fromhex("2B 40 60 00 0F 00 00 00")
        polarity = next(data for data in requests if data[1:3] == b"\x7e\x60")
        assert requests.index(disable) < requests.index(polarity)
        assert requests.index(zero) < requests.index(enable)
        assert requests.index(switch_on) < requests.index(enable)
        assert requests[-1] == enable


def test_deceleration_readback_checks_kinco_raw_units():
    driver, _ = make_driver()
    driver.receive_thread = Mock()
    driver.receive_thread.is_alive.return_value = True
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("43 84 60 00 6E A3 01 00")
    )
    assert driver.motor_set_deceleration(3)
    driver._read_sdo_response.return_value = bytes.fromhex(
        "43 84 60 00 00 00 00 00"
    )
    assert driver.motor_set_deceleration(3) is False


@pytest.mark.parametrize(
    "parameter,value",
    [
        ("kinco_encoder_resolution", 0),
        ("kinco_reduction_ratio", -1),
        ("kinco_acceleration_rps2", float("nan")),
        ("kinco_acceleration_rps2", 0.0001),
        ("kinco_acceleration_rps2", 1e30),
        ("kinco_deceleration_rps2", 0),
        ("kinco_deceleration_rps2", 0.0001),
        ("kinco_guard_time_ms", 49),
        ("kinco_guard_time_ms", 65536),
        ("kinco_guard_life_factor", 1),
        ("kinco_guard_life_factor", 256),
    ],
)
def test_invalid_conversion_configuration_is_rejected_before_startup(parameter, value):
    module, _ = load_kinco()
    with pytest.raises(ValueError):
        module.CanMotorDriver(**{parameter: value})


def test_launch_exposes_default_off_switch_and_kinco_units():
    class Action:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class Description:
        def __init__(self):
            self.actions = []

        def add_action(self, action):
            self.actions.append(action)

    modules = {}
    for name, attributes in {
        "launch": {"LaunchDescription": Description},
        "launch_ros.actions": {"Node": Action},
        "launch.substitutions": {
            "LaunchConfiguration": Action, "TextSubstitution": Action,
        },
        "launch.actions": {"DeclareLaunchArgument": Action},
        "launch.conditions": {"IfCondition": Action},
        "launch_ros.parameter_descriptions": {"ParameterValue": Action},
    }.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    path = Path(__file__).parents[2] / "rtk_nav" / "launch" / "run.launch.py"
    with patch.dict("sys.modules", modules):
        launch = runpy.run_path(str(path))["generate_launch_description"]()
    arguments = {
        action.args[0]: action.kwargs for action in launch.actions if action.args
    }
    assert arguments["brush_use_kinco"]["default_value"].kwargs["text"] == "true"
    assert arguments["motor_auto_enable"]["default_value"].kwargs["text"] == "true"
    assert arguments["kinco_encoder_resolution"]["default_value"].kwargs["text"] == "65536"
    assert arguments["kinco_reduction_ratio"]["default_value"].kwargs["text"] == "10.0"
    assert arguments["kinco_peak_current_a"]["default_value"].kwargs["text"] == "47.9"
    assert arguments["kinco_brush_speed_rpm"]["default_value"].kwargs["text"] == "150.0"
    controller = next(
        action for action in launch.actions
        if action.kwargs.get("executable") == "motor_control"
    )
    parameters = {
        name: value
        for group in controller.kwargs["parameters"]
        for name, value in group.items()
    }
    assert parameters["brush_use_kinco"].kwargs["value_type"] is bool
    assert parameters["motor_auto_enable"].kwargs["value_type"] is bool
    for name in (
        "motor_auto_enable", "brush_use_kinco", "kinco_encoder_resolution",
        "kinco_reduction_ratio", "kinco_acceleration_rps2",
        "kinco_deceleration_rps2", "kinco_peak_current_a",
        "kinco_brush_speed_rpm",
    ):
        assert parameters[name].args[0].args == (name,)


@pytest.mark.parametrize("abort", [False, True])
def test_real_sdo_transaction_receives_kinco_readback_or_abort(abort):
    driver, _ = make_driver()
    driver._pending_sdo_lock = threading.Lock()
    driver._pending_sdo_responses = {}
    driver._sdo_node_locks = {3: threading.RLock()}

    def receive_response(can_id, payload):
        if payload[:4] == bytes.fromhex("40 84 60 00"):
            driver.parse_motor_feedback(
                0x583, bytes.fromhex(
                    "80 84 60 00 00 00 02 06" if abort
                    else "43 84 60 00 6E A3 01 00"
                ),
            )
        return True

    driver.send_can_frame = receive_response
    response = driver._read_sdo_response(
        3, 0x6084, 0, driver.SDO_READ_4_RESPONSE, timeout_sec=0.01
    )
    assert response == (
        None if abort else bytes.fromhex("43 84 60 00 6E A3 01 00")
    )
    assert not driver._pending_sdo_responses


def test_shutdown_sends_zero_and_protocol_specific_disable_to_all_nodes():
    driver, frames = make_driver()
    driver._stopping = False
    with patch("time.sleep"):
        driver.stop_all_motors()
    for motor_id in (1, 2, 3, 4):
        assert (
            0x600 + motor_id, bytes.fromhex("23 FF 60 00 00 00 00 00")
        ) in frames
        controlword = b"\x06" if motor_id in (3, 4) else b"\x00"
        assert (
            0x600 + motor_id,
            bytes.fromhex("2B 40 60 00") + controlword + bytes(3),
        ) in frames


def test_failed_kinco_acceleration_write_keeps_brush_disabled():
    driver, frames = make_driver()
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("4B 41 60 00 40 00 00 00")
    )
    send = driver.send_can_frame

    def fail_acceleration(can_id, data):
        send(can_id, data)
        return not (
            can_id == 0x603 and data[:4] == bytes.fromhex("23 83 60 00")
        )

    driver.send_can_frame = fail_acceleration
    with patch("time.sleep"):
        driver.initialize_motors()
    assert (0x603, bytes.fromhex("2B 40 60 00 0F 00 00 00")) not in frames
    assert (0x604, bytes.fromhex("2B 40 60 00 0F 00 00 00")) in frames
    assert (0x601, bytes.fromhex("2B 40 60 00 0F 00 00 00")) in frames
    driver.send_can_frame = send
    with patch("time.sleep"):
        driver.initialize_motors()
    assert (0x603, bytes.fromhex("2B 40 60 00 0F 00 00 00")) in frames


@pytest.mark.parametrize("use_kinco", [False, True])
@pytest.mark.parametrize("brush_ids", [[], [3], [3, 4]])
def test_controller_selects_driver_and_preserves_topology(use_kinco, brush_ids):
    class Aimotor:
        def __init__(self, **options):
            self.options = options
            self.motors = []

    class Kinco(Aimotor):
        pass

    defaults = {
        "kinco_encoder_resolution": 10000,
        "kinco_reduction_ratio": 5.0,
        "kinco_acceleration_rps2": 20.0,
        "kinco_deceleration_rps2": 40.0,
    }
    parameters = dict(defaults)
    parameters.update({f"motor_direction_{motor_id}": 1 for motor_id in range(1, 5)})
    controller = types.SimpleNamespace(
        brush_motor_ids=brush_ids, brush_use_kinco=use_kinco,
        get_parameter=lambda name: types.SimpleNamespace(value=parameters[name]),
    )
    path = Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "MotorControlNode"
    )
    initializer = next(
        item for item in node.body
        if isinstance(item, ast.FunctionDef) and item.name == "__init__"
    )
    start = next(
        index for index, item in enumerate(initializer.body)
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "motor_ids"
                for target in item.targets)
    )
    end = next(
        index for index, item in enumerate(initializer.body)
        if isinstance(item, ast.Assign)
        and any(isinstance(target, ast.Attribute) and target.attr == "motor_ctrl"
                for target in item.targets)
    )
    namespace = {
        "self": controller, "CanMotorDriver": Aimotor,
        "kinco_defaults": defaults, "profile_acceleration": 60000,
        "profile_deceleration": 445000,
        "importlib": types.SimpleNamespace(
            import_module=lambda name: types.SimpleNamespace(CanMotorDriver=Kinco)
        ),
    }
    exec(compile(ast.Module(
        body=initializer.body[start:end + 1], type_ignores=[]
    ), str(path), "exec"), namespace)
    assert type(controller.motor_ctrl) is (Kinco if use_kinco else Aimotor)
    assert controller.motor_ctrl.options["motor_ids"] == (1, 2, *brush_ids)
    for name, value in defaults.items():
        if use_kinco:
            assert controller.motor_ctrl.options[name] == value
        else:
            assert name not in controller.motor_ctrl.options
