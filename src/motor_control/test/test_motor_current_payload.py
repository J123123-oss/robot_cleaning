"""Contract tests for publishing compact per-motor current data."""

import ast
import json
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


SOURCE = Path(__file__).parents[1] / "motor_control" / "motor_control.py"


def test_robot_state_payload_contains_compact_motor_current():
    source = SOURCE.read_text(encoding="utf-8")
    assert '"motor_current": build_motor_current_payload(' in source


def test_state_payload_rounds_nested_floats_without_changing_coordinates_or_types():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    function = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                    and item.name == "round_state_payload")
    namespace = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 str(SOURCE), "exec"), namespace)
    payload = {
        "current_lon": 120.123456789, "current_lat": 30.987654321,
        "battery_current": -1.23456,
        "battery_temperatures": [23.45678, 24.12345],
        "angular_velocity": {"x": 1.23456, "y": -2.34567, "z": 0.0},
        "motor_current": [0.12345, None, -0.98765],
        "details": [{"sample_age_sec": 0.123456}],
        "motor_fault": [0, 123], "complete_state": False,
        "status": "ENABLE", "areas": "12.34567", "missing": None,
    }

    rounded = namespace["round_state_payload"](payload)

    assert rounded["current_lon"] == payload["current_lon"]
    assert rounded["current_lat"] == payload["current_lat"]
    assert rounded["battery_current"] == -1.23
    assert rounded["battery_temperatures"] == [23.46, 24.12]
    assert rounded["angular_velocity"] == {"x": 1.23, "y": -2.35, "z": 0.0}
    assert rounded["motor_current"] == [0.12, None, -0.99]
    assert rounded["details"] == [{"sample_age_sec": 0.12}]
    assert rounded["motor_fault"] == [0, 123]
    assert type(rounded["motor_fault"][0]) is int
    assert rounded["complete_state"] is False
    assert rounded["missing"] is None
    assert rounded["status"] == "ENABLE"
    assert rounded["areas"] == "12.34567"
    assert payload["battery_temperatures"] == [23.45678, 24.12345]
    assert payload["angular_velocity"]["x"] == 1.23456


def test_publish_state_rounds_numeric_fields_and_preserves_coordinate_precision():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                and item.name == "MotorControlNode")
    method = next(item for item in node.body if isinstance(item, ast.FunctionDef)
                  and item.name == "publish_state")
    functions = [item for item in tree.body if isinstance(item, ast.FunctionDef)
                 and item.name == "round_state_payload"]
    namespace = {
        "json": json, "time": time, "String": SimpleNamespace,
        "WHEEL_RPM_TO_MPS": 0.01,
        "build_motor_fault_payload": lambda *args: {"motor_fault": [0, 123, 0]},
        "build_motor_current_payload": lambda *args: [0.123456, -2.34567, None],
    }
    exec(compile(ast.Module(body=functions + [method], type_ignores=[]),
                 str(SOURCE), "exec"), namespace)
    feedback = [
        {"actual_velocity": 12.3456}, {"actual_velocity": -9.87654},
        {"actual_velocity": 100.123456},
    ]
    controller = SimpleNamespace(
        get_motor_feedback_snapshot=Mock(return_value=feedback),
        current_status="ENABLE", nav_status="IDLE",
        battery_remaining=98.7654, battery_total_voltage=48.123456,
        battery_current=0.123456, battery_temperatures=[23.45678, 24.12345],
        imu_yaw_deg=123.456789, rtk_position_status=4, rtk_orientation_status=3,
        current_lon=120.123456789, current_lat=30.987654321,
        rtk_point_seq=7, route_id=None, cleaning_area="12.34567",
        build_error_code=Mock(return_value=1024),
        angle_x=1.23456, angle_y=-2.34567, sensors_status=3,
        magnetic_error_data=Mock(return_value="0.123456"),
        laser_distance=[0.123456, 0.987654], complete_state=False,
        charging_v=48.765432, charging_i=0.987654, charging_fault=0,
        motor_fault_codes=[0, 123, 0], brush_current_sample_max_age_sec=2.0,
        charge_resume_count=1, robot_state_pub=Mock(), dock_state_pub=Mock(),
        get_logger=Mock(return_value=Mock()),
    )

    namespace["publish_state"](controller)

    payload = json.loads(controller.robot_state_pub.publish.call_args.args[0].data)
    assert payload["current_lon"] == controller.current_lon
    assert payload["current_lat"] == controller.current_lat
    assert payload["battery"] == 98.77
    assert payload["battery_total_voltage"] == 48.12
    assert payload["battery_current"] == 0.12
    assert payload["battery_temperatures"] == [23.46, 24.12]
    assert payload["imu_yaw"] == 123.46
    assert payload["angular_velocity"] == {"x": 1.23, "y": -2.35, "z": 123.46}
    assert payload["acceleration"] == {"x": 12.35, "y": -9.88, "z": 100.12}
    assert payload["velocity"] == 0.11
    assert payload["laser_left"] == 0.12
    assert payload["laser_right"] == 0.99
    assert payload["charging_v"] == 48.77
    assert payload["charging_i"] == 0.99
    assert payload["motor_current"] == [0.12, -2.35, None]
    assert payload["error"] == 1024
    assert payload["sensors_status"] == 3
    assert payload["complete_state"] is False
    assert payload["areas"] == "12.34567"
    assert payload["dock_sensors"] == "0.123456"
    assert controller.battery_total_voltage == 48.123456
    assert feedback[0]["actual_velocity"] == 12.3456
    controller.dock_state_pub.publish.assert_called_once()

