"""离线验证所有充电启动入口都要求最新磁导航末端信号。"""

import ast
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def controller():
    """执行实际控制方法，仅替换ROS服务、日志和时钟。"""
    path = Path(__file__).parents[1] / "motor_control" / "motor_control.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                      and n.name == "MotorControlNode")
    names = {
        "_request_charge_if_docked", "start_charge_async", "_start_charge_callback",
        "charge_resume_callback", "magnetic_mask_callback",
        "_disable_if_docked_while_enabled", "charging_fault_code_cb",
    }
    methods = [n for n in node_class.body
               if isinstance(n, ast.FunctionDef) and n.name in names]
    clock = SimpleNamespace(now=1000.0)
    namespace = {
        "time": SimpleNamespace(monotonic=lambda: clock.now, time=lambda: clock.now),
        "UInt16": object, "Int16": object,
        "ChargeControl": SimpleNamespace(Request=SimpleNamespace),
    }
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"),
         namespace)
    node = type("Controller", (), {name: namespace[name] for name in names})()
    node.clock = clock
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock.now * 1e9)))
    node.get_logger = Mock(return_value=Mock())
    node.cli_start_charge = Mock()
    node.cli_start_charge.service_is_ready.return_value = True
    node.charge_futures = []

    def send_charge_request(req):
        future = Future()
        node.charge_futures.append(future)
        return future

    node.cli_start_charge.call_async.side_effect = send_charge_request
    node.current_status = "DISABLE"
    node.enable_magnetic_navigation = True
    node.is_in_bin_process = False
    node.magnetic_loading_end_active_bits = 6
    node.magnetic_loading_end_count_required = 3
    node.magnetic_raw_mask = 0
    node.magnetic_mask_update_time = None
    node.magnetic_mask = 0
    node.magnetic_last_nonzero_mask = 0
    node.loading_countdown_started = False
    node.is_charging = False
    node.charging_i = 0.04
    node.charging_v = 51.62
    node.charging_fault = 5
    node.last_charging_fault = 5
    node.battery_remaining = 84.0
    node.is_charge_paused = True
    node.charge_resume_threshold = 90
    node.last_charge_stop_time = 0.0
    node.last_charge_resume_time = 0.0
    node.charge_resume_count = 0
    node._charge_start_pending = False
    node._charge_start_last_attempt = 0.0
    node._charge_start_retry_interval = 30.0
    return node


@pytest.mark.parametrize("mask", [0x00, 0x01, 0x18, 0x3E])
@pytest.mark.parametrize("source", ["电量恢复", "LOADING完成", "本地磁导航末端电平"])
def test_start_rejects_fewer_than_six_raw_bits(controller, mask, source):
    controller.magnetic_raw_mask = controller.magnetic_mask = mask
    controller.magnetic_mask_update_time = controller.clock.now
    assert not controller._request_charge_if_docked(source)
    controller.cli_start_charge.call_async.assert_not_called()
    assert not controller._charge_start_pending
    assert controller._charge_start_last_attempt == 0.0


def test_low_battery_resume_with_zero_mask_does_not_charge(controller):
    """复现84%电量、DISABLE、mask=0时错误恢复充电的现场记录。"""
    controller.magnetic_mask_callback(SimpleNamespace(data=0))
    controller.charge_resume_callback()
    controller.cli_start_charge.call_async.assert_not_called()
    assert controller.last_charge_resume_time == 0.0
    assert not any("车体到位" in str(call)
                   for call in controller.get_logger().info.call_args_list)


@pytest.mark.parametrize("age", [None, 0.51, 30.0])
def test_old_or_missing_mask_cannot_authorize_charge(controller, age):
    controller.magnetic_raw_mask = controller.magnetic_mask = 0xFC
    controller.magnetic_mask_update_time = (
        None if age is None else controller.clock.now - age)
    assert not controller._request_charge_if_docked("电量恢复")
    controller.cli_start_charge.call_async.assert_not_called()


@pytest.mark.parametrize("mask", [0x3F, 0xFC, 0xFF])
def test_fresh_six_or_more_bits_start_once(controller, mask):
    controller.magnetic_mask_callback(SimpleNamespace(data=mask))
    controller.cli_start_charge.call_async.assert_called_once()
    assert controller._charge_start_pending
    controller.charge_resume_callback()
    controller.cli_start_charge.call_async.assert_called_once()


def test_reused_loading_mask_is_not_current_dock_evidence(controller):
    controller.current_status = "LOADING"
    controller.loading_countdown_started = True
    controller.magnetic_last_nonzero_mask = 0xFC
    controller.magnetic_mask_callback(SimpleNamespace(data=0))
    assert controller.magnetic_mask == 0xFC  # 保留后退纠偏的既有复用行为。
    controller.current_status = "DISABLE"
    assert not controller._request_charge_if_docked("LOADING完成")
    controller.cli_start_charge.call_async.assert_not_called()


@pytest.mark.parametrize("status", ["ENABLE", "HOLD", "LOADING", "UNLOADING"])
def test_other_states_cannot_start_charging(controller, status):
    controller.current_status = status
    controller.magnetic_raw_mask = controller.magnetic_mask = 0xFC
    controller.magnetic_mask_update_time = controller.clock.now
    assert not controller._request_charge_if_docked("电量恢复")
    controller.cli_start_charge.call_async.assert_not_called()


def test_failed_charge_retries_after_thirty_seconds_only_if_still_docked(controller):
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    controller.charge_futures[0].set_result(
        SimpleNamespace(success=False, message="failed"))
    controller.clock.now += 29.0
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    controller.cli_start_charge.call_async.assert_called_once()
    controller.clock.now += 1.0
    controller.magnetic_mask_callback(SimpleNamespace(data=0))
    controller.charge_resume_callback()
    controller.cli_start_charge.call_async.assert_called_once()
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    assert controller.cli_start_charge.call_async.call_count == 2


def test_charger_fault_recovers_with_current_dock_signal_without_new_edge(controller):
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    controller.charge_futures[0].set_result(
        SimpleNamespace(success=True, message="started"))
    assert controller.is_charging
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    controller.cli_start_charge.call_async.assert_called_once()
    controller.charging_fault_code_cb(SimpleNamespace(data=5))
    assert not controller.is_charging
    controller.clock.now += 30.0
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    assert controller.cli_start_charge.call_async.call_count == 2


def test_service_unavailable_does_not_block_later_valid_retry(controller):
    controller.cli_start_charge.service_is_ready.return_value = False
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    controller.cli_start_charge.call_async.assert_not_called()
    assert not controller._charge_start_pending
    controller.cli_start_charge.service_is_ready.return_value = True
    controller.clock.now += 30.0
    controller.magnetic_mask_callback(SimpleNamespace(data=0xFC))
    controller.cli_start_charge.call_async.assert_called_once()
