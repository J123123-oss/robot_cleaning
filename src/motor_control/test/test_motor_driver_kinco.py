"""Offline checks against the Kinco iSMK manual's CANopen examples."""

import ast
import importlib.util
import inspect
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
    driver._read_sdo_response = Mock(side_effect=[
        bytes.fromhex("4B 41 60 00 21 00 00 00"),
        bytes.fromhex("4F 7E 60 00 01 00 00 00"),
        bytes.fromhex("4B 41 60 00 21 00 00 00"),
        bytes.fromhex("4F 7E 60 00 00 00 00 00"),
    ])
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


@pytest.mark.parametrize("direction,polarity", [(1, 0), (-1, 1)])
def test_direction_waits_for_shutdown_and_confirms_polarity(direction, polarity):
    driver, frames = make_driver()
    driver._pending_sdo_lock = threading.Lock()
    driver._pending_sdo_responses = {}
    driver._sdo_node_locks = {3: threading.RLock()}
    status_reads = 0

    def receive_response(can_id, payload):
        nonlocal status_reads
        frames.append((can_id, bytes(payload)))
        if payload[:4] == bytes.fromhex("40 41 60 00"):
            status_reads += 1
            status = 0x4637 if status_reads == 1 else 0x4231
            driver.parse_motor_feedback(
                0x583, bytes.fromhex("4B 41 60 00")
                + status.to_bytes(2, "little") + bytes(2),
            )
        elif payload[:4] == bytes.fromhex("2F 7E 60 00"):
            assert status_reads == 2
        elif payload[:4] == bytes.fromhex("40 7E 60 00"):
            driver.parse_motor_feedback(
                0x583, bytes.fromhex("4F 7E 60 00") + bytes((polarity, 0, 0, 0))
            )
        return True

    driver.send_can_frame = receive_response
    with patch("time.sleep"):
        assert driver.motor_set_direction(3, direction)
    assert frames[:2] == [
        (0x603, bytes.fromhex("23 FF 60 00 00 00 00 00")),
        (0x603, bytes.fromhex("2B 40 60 00 06 00 00 00")),
    ]
    assert frames[-1] == (0x603, bytes.fromhex("40 7E 60 00 00 00 00 00"))
    assert driver.motor_directions[3] == direction
    frames.clear()
    assert driver.motor_set_speed(3, 100)
    assert frames == [(0x603, bytes.fromhex("23 FF 60 00 82 4E 1B 00"))]


@pytest.mark.parametrize("polarity_response", [
    None, bytes.fromhex("4F 7E 60 00 00 00 00 00"),
])
def test_unconfirmed_direction_fails_initialization_without_enabling(polarity_response):
    driver, frames = make_driver()
    driver.motor_directions[3] = -1

    def readback(motor_id, index, subindex, expected_command, **kwargs):
        if index == driver.STATUSWORD_INDEX:
            return bytes.fromhex("4B 41 60 00 21 00 00 00")
        assert index == driver.COMMAND_POLARITY_INDEX
        return polarity_response

    driver._read_sdo_response = Mock(side_effect=readback)
    with patch("time.sleep"):
        assert not driver.initialize_motors(motor_ids=(3,))
    assert (0x603, bytes.fromhex("2B 40 60 00 0F 00 00 00")) not in frames


def test_direction_is_not_written_when_shutdown_is_unconfirmed():
    driver, frames = make_driver()
    driver._read_sdo_response = Mock(return_value=None)
    assert not driver.motor_set_direction(3, -1)
    assert not any(data[1:3] == bytes.fromhex("7E 60") for _, data in frames)
    assert driver.motor_directions[3] == 1


@pytest.mark.parametrize("status", [0x4637, 0x0008])
def test_direction_is_not_written_when_motor_remains_enabled_or_faulted(status):
    driver, frames = make_driver()
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("4B 41 60 00")
        + status.to_bytes(2, "little") + bytes(2)
    )
    with patch("time.monotonic", side_effect=[0.0, 0.1, 0.2, 0.6]), patch("time.sleep"):
        assert not driver.motor_set_direction(3, -1)
    assert not any(data[1:3] == bytes.fromhex("7E 60") for _, data in frames)
    assert driver.motor_directions[3] == 1


@pytest.mark.parametrize("abort", [False, True])
def test_rejected_polarity_write_does_not_report_direction_success(abort):
    driver, _ = make_driver()
    driver._pending_sdo_lock = threading.Lock()
    driver._pending_sdo_responses = {}
    driver._sdo_node_locks = {3: threading.RLock()}

    def receive_response(can_id, payload):
        if payload[:4] == bytes.fromhex("40 41 60 00"):
            driver.parse_motor_feedback(
                0x583, bytes.fromhex("4B 41 60 00 31 42 00 00")
            )
        elif payload[:4] == bytes.fromhex("2F 7E 60 00"):
            driver.parse_motor_feedback(
                0x583, bytes.fromhex("80 7E 60 00 22 00 00 08")
            )
        elif payload[:4] == bytes.fromhex("40 7E 60 00"):
            driver.parse_motor_feedback(
                0x583, bytes.fromhex(
                    "80 7E 60 00 00 00 02 06" if abort
                    else "4F 7E 60 00 00 00 00 00"
                ),
            )
        return True

    driver.send_can_frame = receive_response
    assert not driver.motor_set_direction(3, -1)
    assert driver.motor_directions[3] == 1
    assert not driver._pending_sdo_responses


def mock_configuration_readback(driver, initial_statuses=None):
    statuses = {motor_id: 0x40 for motor_id in driver.motor_ids}
    statuses.update(initial_statuses or {})
    send = driver.send_can_frame

    def send_with_status(can_id, data):
        if data[:4] == bytes.fromhex("2B 40 60 00"):
            motor_id = can_id - 0x600
            controlword = int.from_bytes(data[4:6], "little")
            if controlword in (0x80, 0x86):
                statuses[motor_id] = 0x40
            elif not statuses[motor_id] & 0x0008:
                statuses[motor_id] = {0: 0x40, 6: 0x21, 7: 0x23, 15: 0x27}[controlword]
        return send(can_id, data)

    def readback(motor_id, index, subindex, expected_command, **kwargs):
        if index == driver.COMMAND_POLARITY_INDEX:
            polarity = 1 if driver.motor_directions[motor_id] == -1 else 0
            return bytes.fromhex("4F 7E 60 00") + bytes((polarity, 0, 0, 0))
        assert index == driver.STATUSWORD_INDEX
        status = statuses[motor_id]
        return bytes.fromhex("4B 41 60 00") + status.to_bytes(2, "little") + bytes(2)

    driver.send_can_frame = send_with_status
    driver._read_sdo_response = Mock(side_effect=readback)


def test_kinco_velocity_mode_writes_three():
    driver, frames = make_driver()

    assert driver.motor_set_mode(3, 3)

    assert frames[-1] == (
        0x603,
        bytes.fromhex("2F 60 60 00 03 00 00 00"),
    )


def test_kinco_enable_follows_manual_sequence_and_leaves_operation_enabled():
    driver, frames = make_driver()
    driver._read_sdo_response = Mock(
        return_value=bytes.fromhex("4B 41 60 00 37 46 00 00")
    )

    assert driver.motor_enable(3)

    assert frames == [
        (0x603, bytes.fromhex("23 FF 60 00 00 00 00 00")),
        (0x603, bytes.fromhex("2B 40 60 00 06 00 00 00")),
        (0x603, bytes.fromhex("2B 40 60 00 07 00 00 00")),
        (0x603, bytes.fromhex("2B 40 60 00 0F 00 00 00")),
    ]


def test_kinco_enable_fails_without_live_operation_enabled_confirmation():
    driver, _ = make_driver()
    driver._read_sdo_response = Mock(return_value=None)

    assert not driver.motor_enable(3)


def test_kinco_startup_restores_guard_before_resetting_latched_communication_fault():
    driver, frames = make_driver()
    mock_configuration_readback(driver, {3: 0x0008})
    events = []
    driver.configure_kinco_guard.side_effect = (
        lambda motor_id: events.append(("guard", motor_id)) or True
    )
    send = driver.send_can_frame

    def record_reset(can_id, data):
        if data[:4] == bytes.fromhex("2B 40 60 00") and data[4] == 0x86:
            events.append(("reset", can_id - 0x600))
        return send(can_id, data)

    driver.send_can_frame = record_reset
    with patch("time.sleep"):
        assert driver.initialize_motors(motor_ids=(3,))
    assert events[:2] == [("guard", 3), ("reset", 3)]
    reset = (0x603, bytes.fromhex("2B 40 60 00 86 00 00 00"))
    polarity = (0x603, bytes.fromhex("2F 7E 60 00 00 00 00 00"))
    assert frames.index(reset) < frames.index(polarity)
    assert frames[-1] == (0x603, bytes.fromhex("2B 40 60 00 0F 00 00 00"))
    assert driver.motors[2]["velocity"] == 0.0


def test_kinco_startup_does_not_reset_or_configure_when_guard_setup_fails():
    driver, frames = make_driver()
    mock_configuration_readback(driver, {3: 0x0008})
    driver.configure_kinco_guard.return_value = False
    with patch("time.sleep"):
        assert not driver.initialize_motors(motor_ids=(3,))
    assert not any(data[:4] == bytes.fromhex("2F 7E 60 00") for _, data in frames)
    assert not any(data == bytes.fromhex("2B 40 60 00 86 00 00 00")
                   or data == bytes.fromhex("2B 40 60 00 0F 00 00 00")
                   for _, data in frames)


def test_mixed_startup_configures_brush_without_enabling_it():
    driver, frames = make_driver()
    defaults = inspect.signature(driver.__init__).parameters
    driver.kinco_acceleration_rps2 = defaults["kinco_acceleration_rps2"].default
    driver.kinco_deceleration_rps2 = defaults["kinco_deceleration_rps2"].default
    mock_configuration_readback(driver)
    with patch("time.sleep"):
        assert driver.initialize_motors(motor_ids=(1, 2, 3), enable_kinco=False)
    enable = bytes.fromhex("2B 40 60 00 0F 00 00 00")
    assert (0x601, enable) in frames
    assert (0x602, enable) in frames
    assert (0x603, enable) not in frames
    assert any(can_id == 0x603 and data[1:3] == b"\x7e\x60"
               for can_id, data in frames)
    assert driver.kinco_acceleration_rps2 == driver.kinco_deceleration_rps2 == 10.0
    ramp = bytes.fromhex("F1 29 00 00")
    assert (0x603, bytes.fromhex("23 83 60 00") + ramp) in frames
    assert (0x603, bytes.fromhex("23 84 60 00") + ramp) in frames


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


@pytest.mark.parametrize("code,description", [
    (0x8100, "总线通讯错误"),
    (0x81FF, "总线通讯超时"),
    (0x5442, "正限位报错"),
    (0x5443, "预使能报警"),
    (0x7331, "通讯式编码器没有连接或没有回应"),
    (0x0000, "无故障"),
])
def test_kinco_fault_table_matches_manual_emergency_codes(code, description):
    driver, _ = make_driver()
    assert driver.KINCO_FAULT_TABLE[code] == description


def test_kinco_emergency_decodes_error_register_and_publishes_named_fault():
    driver, _ = make_driver()
    with patch("time.monotonic", return_value=100.0):
        driver.parse_motor_fault(0x083, bytes.fromhex("00 81 15 AA 34 12 00 10"))
    with patch("time.monotonic", return_value=100.25):
        motor = driver.get_kinco_feedback_snapshot()[0]
    assert motor["fault_code"] == 0x8100
    assert motor["fault_description"] == "总线通讯错误"
    assert motor["error_register"] == 0x15
    assert motor["error_register_descriptions"] == ["一般错误", "电压", "通讯错误"]
    assert motor["error_status"] == 0x1234
    assert motor["error_status_2"] == 0x1000
    assert motor["sample_age_sec"]["error_register"] == pytest.approx(0.25)
    log = driver.get_logger.return_value.error.call_args.args[0]
    assert "0x8100" in log and "总线通讯错误" in log
    assert "1001=0x15" in log and "电压" in log and "通讯错误" in log
    assert "2601=0x1234" in log and "2602=0x1000" in log
    driver.motor_fault_publisher.publish.assert_called_once()


def test_kinco_fault_reset_clears_description_and_error_register():
    driver, _ = make_driver()
    driver.parse_motor_fault(0x084, bytes.fromhex("FF 81 10 00 00 00 00 10"))
    driver.parse_motor_fault(0x084, bytes(8))
    motor = driver.get_kinco_feedback_snapshot()[1]
    assert motor["fault_description"] == "无故障"
    assert motor["error_register"] == 0
    assert motor["error_register_descriptions"] == []
    assert motor["error_status"] == 0
    assert motor["error_status_2"] == 0
    assert "无故障" in driver.get_logger.return_value.info.call_args.args[0]


def test_kinco_unknown_fault_is_explicit_and_keeps_raw_code():
    driver, _ = make_driver()
    driver.parse_motor_fault(0x083, bytes.fromhex("EF BE 80 00 00 00 00 00"))
    motor = driver.get_kinco_feedback_snapshot()[0]
    assert motor["fault_code"] == 0xBEEF
    assert motor["fault_description"] == "手册未定义故障码 0xBEEF"
    assert motor["error_register_descriptions"] == ["保留"]
    assert "0xBEEF" in driver.get_logger.return_value.error.call_args.args[0]


def test_kinco_sdo_fault_uses_same_table_and_keeps_unread_register_null():
    driver, _ = make_driver()
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 3F 60 00 FF 81 00 00"))
    motor = driver.get_kinco_feedback_snapshot()[0]
    assert motor["fault_description"] == "总线通讯超时"
    assert motor["error_register"] is None
    assert motor["error_register_descriptions"] is None
    assert motor["sample_age_sec"]["error_register"] is None
    assert "总线通讯超时" in driver.get_logger.return_value.error.call_args.args[0]
    driver.parse_motor_feedback(0x583, bytes.fromhex("4B 3F 60 00 00 00 00 00"))
    assert driver.get_kinco_feedback_snapshot()[0]["fault_description"] == "无故障"


def test_kinco_fault_descriptions_are_in_published_json():
    driver, _ = make_driver()
    driver.kinco_feedback_publisher = Mock()
    driver.parse_motor_fault(0x083, bytes.fromhex("00 81 10 00 00 00 00 10"))
    driver.poll_feedback()
    message = driver.kinco_feedback_publisher.publish.call_args.args[0].data
    motor = json.loads(message)["motors"][0]
    assert motor["fault_description"] == "总线通讯错误"
    assert motor["error_register_descriptions"] == ["通讯错误"]
    assert "总线通讯错误" in message


def test_mixed_driver_retains_aimotor_fault_meaning_for_wheels():
    driver, _ = make_driver()
    driver.parse_motor_fault(0x081, bytes.fromhex("42 54 01 00 00 00 00 00"))
    errors = [call.args[0] for call in driver.get_logger.return_value.error.call_args_list]
    assert any("紧急停机" in message for message in errors)
    assert not any("正限位报错" in message for message in errors)


def test_initialization_disables_brushes_before_polarity_and_zeroes_before_enable():
    driver, frames = make_driver()
    mock_configuration_readback(driver)
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


@pytest.mark.parametrize("setting, index", [("acceleration", 0x83), ("deceleration", 0x84)])
def test_ramp_readback_checks_kinco_raw_units(setting, index):
    driver, _ = make_driver()
    setattr(driver, f"kinco_{setting}_rps2", 10.0)
    write_ramp = getattr(driver, f"motor_set_{setting}")
    driver.receive_thread = Mock()
    driver.receive_thread.is_alive.return_value = True
    response_header = bytes((0x43, index, 0x60, 0x00))
    driver._read_sdo_response = Mock(
        return_value=response_header + bytes.fromhex("F1 29 00 00")
    )
    assert write_ramp(3)
    driver._read_sdo_response.return_value = response_header + bytes(4)
    assert write_ramp(3) is False


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
    assert arguments["enable_brush_test"]["default_value"].kwargs["text"] == "false"
    assert arguments["kinco_encoder_resolution"]["default_value"].kwargs["text"] == "65536"
    assert arguments["kinco_reduction_ratio"]["default_value"].kwargs["text"] == "10.0"
    assert arguments["kinco_acceleration_rps2"]["default_value"].kwargs["text"] == "10.0"
    assert arguments["kinco_deceleration_rps2"]["default_value"].kwargs["text"] == "10.0"
    assert arguments["kinco_peak_current_a"]["default_value"].kwargs["text"] == "47.9"
    assert arguments["kinco_brush_speed_rpm"]["default_value"].kwargs["text"] == "110.0"
    for motor_id in (3, 4):
        name = f"kinco_motor_direction_{motor_id}"
        assert arguments[name]["default_value"].kwargs["text"] == "-1"
        assert arguments[name]["choices"] == ["1", "-1"]
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
    assert parameters["enable_brush_test"].kwargs["value_type"] is bool
    for motor_id in (3, 4):
        parameter = parameters[f"kinco_motor_direction_{motor_id}"]
        assert parameter.kwargs["value_type"] is int
        assert parameter.args[0].args == (f"kinco_motor_direction_{motor_id}",)
    for name in (
        "motor_auto_enable", "enable_brush_test", "brush_use_kinco", "kinco_encoder_resolution",
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
    mock_configuration_readback(driver)
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
    parameters.update({
        "kinco_motor_direction_3": -1, "kinco_motor_direction_4": 1,
    })
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
    assert controller.motor_ctrl.options["motor_directions"] == {
        motor_id: (-1 if use_kinco and motor_id == 3 else 1)
        for motor_id in (1, 2, *brush_ids)
    }
    for name, value in defaults.items():
        if use_kinco:
            assert controller.motor_ctrl.options[name] == value
        else:
            assert name not in controller.motor_ctrl.options
