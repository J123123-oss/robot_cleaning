"""离线验证进出仓IO保护及定时器重复写入时的最终轮速拦截。"""

import ast
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def make_node(mode="NORMAL", in_bin=True):
    """加载实际控制方法，使用替身电机记录下发结果，无需ROS或CAN。"""
    path = Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                      and n.name == "MotorControlNode")
    names = {"io_data_callback", "_update_blocked_directions",
             "_wheel_command_blocked_by_sensors", "set_motors_speed"}
    methods = [n for n in node_class.body
               if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = {
        "time": time, "UInt8": object, "Vector3": SimpleNamespace,
        "MIN_SPEED": -100.0, "MAX_SPEED": 100.0,
        "BOUNDARY_TRIGGER_CONFIRM_FRAMES": 1,
        "BOUNDARY_CLEAR_CONFIRM_FRAMES": 2,
    }
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"),
         namespace)
    controller = type("Controller", (), {name: namespace[name] for name in names})
    node = controller()
    node.current_control_mode = mode
    node.is_in_bin_process = in_bin
    node.current_status = "LOADING"
    node.confirmed_sensors = set()
    node.blocked_directions = set()
    node.boundary_active_count = 0
    node.boundary_clear_count = 0
    node.boundary_test_control_active = False
    node.current_left_speed = 0.0
    node.current_right_speed = 0.0
    node.brush_speed = 0.0
    node.motor_ctrl = SimpleNamespace(
        motors=[{"id": 1}, {"id": 2}], motor_set_speed=Mock())
    node.speed_pub = Mock()
    node.get_logger = lambda: Mock()
    node.set_brush_speed = Mock()
    return node


@pytest.mark.parametrize("status,left,right,io", [
    ("LOADING", 10.0, -10.0, 0x0B),  # 后左触发，禁止后退
    ("UNLOADING", -10.0, 10.0, 0x0E),  # 前左触发，禁止前进
    ("START", -10.0, 10.0, 0x0D),  # 前右触发，禁止前进
])
@pytest.mark.parametrize("mode", ["NORMAL", "AUTO_CLEANING"])
def test_docking_blocks_repeated_commands_and_allows_escape(status, left, right, io, mode):
    node = make_node(mode=mode)
    node.current_status = status
    node.set_motors_speed(left, right)
    node.io_data_callback(SimpleNamespace(data=io))
    assert (node.current_left_speed, node.current_right_speed) == (0.0, 0.0)
    assert node.sensors_status != 0
    for _ in range(3):
        node.set_motors_speed(left, right)
        assert (node.current_left_speed, node.current_right_speed) == (0.0, 0.0)
        assert node.motor_ctrl.motor_set_speed.call_args_list[-2].args[1] == 0.0
        assert node.motor_ctrl.motor_set_speed.call_args_list[-1].args[1] == 0.0
    node.set_motors_speed(-left, -right)
    assert (node.current_left_speed, node.current_right_speed) == (-left, -right)
    node.io_data_callback(SimpleNamespace(data=0x0F))
    node.io_data_callback(SimpleNamespace(data=0x0F))
    node.set_motors_speed(left, right)
    assert (node.current_left_speed, node.current_right_speed) == (left, right)


def test_cleaning_keeps_rtk_boundary_ownership():
    node = make_node(mode="AUTO_CLEANING", in_bin=False)
    node.current_status = "AUTO_CLEANING"
    node.io_data_callback(SimpleNamespace(data=0x0E))
    assert node.sensors_status == 1
    assert not node.blocked_directions
    node.set_motors_speed(-10.0, 10.0)
    assert (node.current_left_speed, node.current_right_speed) == (-10.0, 10.0)
