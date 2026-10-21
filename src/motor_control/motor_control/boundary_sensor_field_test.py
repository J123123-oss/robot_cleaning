#!/usr/bin/env python3
"""Low-speed forward/reverse field test for the four boundary sensors."""

import argparse
import csv
import os
import sys
import time
from datetime import datetime

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String, UInt8


PARAMETER_NAME = "manual_drive_speed_scale"
MAX_LINEAR_SPEED_MPS = 0.35
SENSOR_NAMES = ("mid_left", "mid_right", "back_left", "back_right")


class BoundaryTestNode(Node):
    def __init__(self):
        super().__init__("boundary_sensor_field_test")
        self.io_raw = None
        self.io_time = None
        self.io_wall_time = None
        self.motor_state = None
        self.control_mode = None
        self.feedback = {}
        self.feedback_time = None
        self.writer = None
        self.writer_file = None
        self.event_context = {}
        self.leg_active = False
        self.expected_mask = 0
        self.sensor_trigger_mono = None
        self.sensor_trigger_wall = None
        self.gpio_trigger_seen = False
        self.first_zero_command_mono = None
        self.first_stop_feedback_mono = None
        self.stop_rpm = 1.0
        self.previous_feedback_pair = None
        self.feedback_duplicate_count = 0
        self.keyboard_pub = self.create_publisher(String, "/keyboard/control", 10)
        self.create_subscription(UInt8, "/io_data", self._io_callback, 10)
        self.create_subscription(
            Float64MultiArray, "/io_data_diagnostics", self._diagnostic_callback, 10
        )
        self.create_subscription(Vector3, "/motor/current_speed", self._speed_callback, 10)
        self.create_subscription(String, "/motor/state", self._state_callback, 10)
        self.create_subscription(String, "/control/mode", self._mode_callback, 10)
        self.create_subscription(
            Float32MultiArray, "/motor_feedback", self._feedback_callback, 10
        )
        self.parameter_client = AsyncParameterClient(self, "/motor_control_node")

    def _io_callback(self, msg):
        self.io_raw = int(msg.data) & 0x0F
        self.io_time = time.monotonic()
        self.io_wall_time = time.time()
        if self.leg_active:
            mask = self.triggered_mask or 0
            self._log_event(
                "IO_TOPIC_SAMPLE", self.io_time, self.io_wall_time,
                io_bitmap=f"0x{self.io_raw:02x}",
                trigger_sensor_bits=f"0x{mask:02x}",
            )
            if self.sensor_trigger_mono is None and mask & self.expected_mask:
                self.sensor_trigger_mono = self.io_time
                self.sensor_trigger_wall = self.io_wall_time
                self._log_event(
                    "IO_TOPIC_TRIGGER", self.io_time, self.io_wall_time,
                    io_bitmap=f"0x{self.io_raw:02x}",
                    trigger_sensor_bits=f"0x{mask & self.expected_mask:02x}",
                )

    def _state_callback(self, msg):
        self.motor_state = msg.data.strip().upper()

    def _mode_callback(self, msg):
        self.control_mode = msg.data.strip().upper()

    def _feedback_callback(self, msg):
        values = list(msg.data)
        if len(values) % 5:
            return
        feedback = {}
        for offset in range(0, len(values), 5):
            motor_id = int(values[offset])
            feedback[motor_id] = float(values[offset + 2])
        self.feedback = feedback
        self.feedback_time = time.monotonic()
        pair = (feedback.get(1), feedback.get(2))
        changed = pair != self.previous_feedback_pair
        if self.leg_active and self.sensor_trigger_mono is not None:
            self._log_event(
                "FEEDBACK_SAMPLE", self.feedback_time, time.time(),
                left_rpm=pair[0], right_rpm=pair[1],
                feedback_changed=int(changed),
            )
            if not changed:
                self.feedback_duplicate_count += 1
            if (self.first_stop_feedback_mono is None and
                    pair[0] is not None and pair[1] is not None and
                    abs(pair[0]) <= self.stop_rpm and abs(pair[1]) <= self.stop_rpm):
                self.first_stop_feedback_mono = self.feedback_time
                self._log_event(
                    "FIRST_STOP_FEEDBACK", self.feedback_time, time.time(),
                    left_rpm=pair[0], right_rpm=pair[1],
                    feedback_changed=int(changed),
                )
        self.previous_feedback_pair = pair

    def _log_event(self, event, monotonic_time, wall_time, **values):
        if self.writer is None:
            return
        row = {
            **self.event_context,
            "event": event,
            "event_time": datetime.fromtimestamp(wall_time).astimezone().isoformat(
                timespec="milliseconds"
            ),
            "event_monotonic": f"{monotonic_time:.6f}",
            "elapsed_from_sensor_ms": (
                f"{(monotonic_time - self.sensor_trigger_mono) * 1000:.1f}"
                if self.sensor_trigger_mono is not None else ""
            ),
            **values,
        }
        self.writer.writerow(row)
        self.writer_file.flush()

    def _diagnostic_callback(self, msg):
        if len(msg.data) < 4 or not self.leg_active:
            return
        wall, monotonic, raw_bitmap, io_bitmap = msg.data[:4]
        raw_bitmap, io_bitmap = int(raw_bitmap), int(io_bitmap)
        active_mask = (~io_bitmap) & 0x0F
        self._log_event(
            "GPIO_SAMPLE", monotonic, wall,
            gpio_raw_bitmap=f"0x{raw_bitmap:02x}",
            io_bitmap=f"0x{io_bitmap:02x}",
            trigger_sensor_bits=f"0x{active_mask:02x}",
        )
        if active_mask & self.expected_mask and not self.gpio_trigger_seen:
            self.gpio_trigger_seen = True
            if self.first_zero_command_mono is None:
                self.sensor_trigger_mono = monotonic
                self.sensor_trigger_wall = wall
            self._log_event(
                "GPIO_TRIGGER_SAMPLE", monotonic, wall,
                gpio_raw_bitmap=f"0x{raw_bitmap:02x}",
                io_bitmap=f"0x{io_bitmap:02x}",
                trigger_sensor_bits=f"0x{active_mask & self.expected_mask:02x}",
            )

    def _speed_callback(self, msg):
        if not self.leg_active:
            return
        now = time.monotonic()
        zero = max(abs(msg.x), abs(msg.y), abs(msg.z)) < 1e-6
        if zero and self.sensor_trigger_mono is not None and self.first_zero_command_mono is None:
            self.first_zero_command_mono = now
            self._log_event(
                "FIRST_ZERO_SPEED_COMMAND", now, time.time(),
                motor_x=msg.x, motor_y=msg.y, motor_z=msg.z,
            )
        self._log_event(
            "MOTOR_SPEED_COMMAND", now, time.time(),
            motor_x=msg.x, motor_y=msg.y, motor_z=msg.z,
        )

    @property
    def triggered_mask(self):
        if self.io_raw is None:
            return None
        # Match motor_control.io_data_callback: a low topic bit means active.
        return (~self.io_raw) & 0x0F

    def publish_key(self, key):
        msg = String()
        msg.data = key
        self.keyboard_pub.publish(msg)


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def await_future(node, future, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError("ROS parameter service timed out")
    return future.result()


def set_speed_scale(node, value):
    if not node.parameter_client.wait_for_services(timeout_sec=3.0):
        raise RuntimeError(
            "找不到 /motor_control_node 参数服务；请确认运行的是已更新并重新构建的 motor_control"
        )
    response = await_future(
        node,
        node.parameter_client.set_parameters(
            [Parameter(name=PARAMETER_NAME, value=float(value))]
        ),
    )
    result = response.results[0]
    if not result.successful:
        raise RuntimeError(f"设置速度参数失败：{result.reason}")


def get_speed_scale(node):
    response = await_future(
        node,
        node.parameter_client.get_parameters([PARAMETER_NAME]),
    )
    if not response.values or response.values[0].type != ParameterType.PARAMETER_DOUBLE:
        raise RuntimeError(
            f"参数 {PARAMETER_NAME} 不存在；请先构建并运行更新后的 motor_control"
        )
    return float(response.values[0].double_value)


def parse_args():
    parser = argparse.ArgumentParser(
        description="低速前后往返，记录边界传感器触发到驱动轮停稳的时间。"
    )
    parser.add_argument("--round-trips", type=int, default=3, help="往返次数（默认3）")
    parser.add_argument(
        "--speed-scale", type=float, default=0.10,
        help="手动前后速度比例，范围0.01到0.4；默认0.10，约0.035m/s",
    )
    parser.add_argument("--pause-sec", type=float, default=2.0, help="每次停稳后的暂停秒数")
    parser.add_argument("--leg-timeout-sec", type=float, default=30.0, help="每段等待传感器触发的超时")
    parser.add_argument("--stop-timeout-sec", type=float, default=5.0, help="触发后等待轮子停稳的超时")
    parser.add_argument("--stop-rpm", type=float, default=1.0, help="轮子停稳阈值（绝对转速RPM）")
    parser.add_argument("--log-dir", default="~/robot_cleaning/test_logs", help="CSV日志目录")
    return parser.parse_args()


def require_runtime_ready(
    node, require_clear_sensors=True, allowed_states=("ENABLE", "HOLD")
):
    spin_for(node, 1.0)
    now = time.monotonic()
    if node.io_raw is None or node.io_time is None or now - node.io_time > 0.5:
        raise RuntimeError("/io_data 没有新数据；确认 sensors_485_node 正在运行")
    if node.motor_state not in allowed_states:
        expected = "/".join(allowed_states)
        raise RuntimeError(
            f"电机状态必须是 {expected}，当前为 {node.motor_state!r}；"
            "测试不会自动使能或切换模式"
        )
    if node.control_mode != "NORMAL":
        raise RuntimeError(f"控制模式必须是 NORMAL，当前为 {node.control_mode!r}")
    if not {1, 2}.issubset(node.feedback):
        raise RuntimeError("/motor_feedback 未包含左右驱动轮 ID 1、2")
    if node.feedback_time is None or now - node.feedback_time > 0.5:
        raise RuntimeError("/motor_feedback 数据超时")
    if require_clear_sensors and node.triggered_mask != 0:
        raise RuntimeError(
            f"传感器基线不是全释放：io_data=0x{node.io_raw:02x}, "
            f"triggered=0x{node.triggered_mask:01x}；请先排除触发/接线问题"
        )
    if any(abs(node.feedback[motor_id]) > 1.0 for motor_id in (1, 2)):
        raise RuntimeError("启动前驱动轮还在转；请先让车停稳")


def wait_for_stop(node, args, direction_mask, expected_state, leg_timeout):
    leg_start = time.monotonic()
    trigger_time = None
    trigger_wall_time = None
    raw_at_trigger = None
    stop_wall_time = None
    trigger_mask = 0
    stop_count = 0
    state_check_at = 0.0
    command_seen = False

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        mask = node.triggered_mask
        if node.io_time is None or now - node.io_time > 0.5:
            return "IO_TIMEOUT", trigger_time, trigger_wall_time, None, stop_wall_time, trigger_mask, raw_at_trigger
        if now >= state_check_at:
            if node.control_mode != "NORMAL":
                return "CONTROL_CHANGED", trigger_time, trigger_wall_time, None, stop_wall_time, trigger_mask, raw_at_trigger
            if node.motor_state == expected_state:
                command_seen = True
            elif command_seen:
                return "CONTROL_CHANGED", trigger_time, trigger_wall_time, None, stop_wall_time, trigger_mask, raw_at_trigger
            elif now - leg_start > 1.0:
                return "COMMAND_NOT_ACCEPTED", trigger_time, trigger_wall_time, None, stop_wall_time, trigger_mask, raw_at_trigger
            elif node.motor_state not in ("ENABLE", "HOLD"):
                return "CONTROL_CHANGED", trigger_time, trigger_wall_time, None, stop_wall_time, trigger_mask, raw_at_trigger
            state_check_at = now + 0.2

        if trigger_time is None:
            if mask & direction_mask:
                trigger_time = node.io_time
                trigger_wall_time = node.io_wall_time
                raw_at_trigger = node.io_raw
                trigger_mask = mask & direction_mask
            elif now - leg_start >= leg_timeout:
                return "NO_TRIGGER_TIMEOUT", None, None, None, None, 0, None
        else:
            if mask:
                trigger_mask |= mask
            if node.feedback_time is not None and now - node.feedback_time <= 0.5:
                if all(abs(node.feedback.get(motor_id, float("inf"))) <= args.stop_rpm for motor_id in (1, 2)):
                    stop_count += 1
                else:
                    stop_count = 0
            else:
                stop_count = 0
            if stop_count >= 3:
                stop_wall_time = time.time()
                return "OK", trigger_time, trigger_wall_time, now, stop_wall_time, trigger_mask, raw_at_trigger
            if now - trigger_time >= args.stop_timeout_sec:
                return "STOP_TIMEOUT", trigger_time, trigger_wall_time, None, None, trigger_mask, raw_at_trigger
    return "INTERRUPTED", trigger_time, trigger_wall_time, None, None, trigger_mask, raw_at_trigger


def main():
    args = parse_args()
    if args.round_trips < 1:
        raise SystemExit("--round-trips 必须至少为1")
    if not 0.01 <= args.speed_scale <= 0.4:
        raise SystemExit("--speed-scale 范围是0.01到0.4")
    if min(args.pause_sec, args.leg_timeout_sec, args.stop_timeout_sec) <= 0:
        raise SystemExit("暂停和超时时间必须大于0")
    if args.stop_rpm <= 0:
        raise SystemExit("--stop-rpm 必须大于0")

    rclpy.init()
    node = BoundaryTestNode()
    log_file = None
    original_speed_scale = None
    started = False
    exit_code = 0
    run_timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
    target_speed_mps = MAX_LINEAR_SPEED_MPS * args.speed_scale
    log_dir = os.path.expanduser(args.log_dir)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    speed_tag = f"{args.speed_scale:.2f}".replace(".", "p")
    log_path = os.path.join(
        log_dir,
        f"boundary_test_{stamp}_speed{speed_tag}_rounds{args.round_trips}.csv",
    )
    fieldnames = [
        "run_timestamp", "speed_scale", "target_speed_mps", "round_trip_count",
        "event", "leg_index", "round_trip", "direction", "departure_time",
        "expected_sensor_bits",
        "trigger_sensor_names", "trigger_time", "stop_time", "trigger_to_stop_ms",
        "result", "io_raw_at_trigger", "event_time", "event_monotonic",
        "elapsed_from_sensor_ms", "gpio_raw_bitmap", "io_bitmap",
        "trigger_sensor_bits", "motor_x", "motor_y", "motor_z",
        "left_rpm", "right_rpm", "feedback_changed",
        "sensor_to_zero_command_ms", "sensor_to_first_stop_feedback_ms",
        "zero_command_to_first_stop_feedback_ms", "feedback_duplicate_count",
    ]
    writer = None

    try:
        require_runtime_ready(node)
        original_speed_scale = get_speed_scale(node)
        set_speed_scale(node, args.speed_scale)
        spin_for(node, 0.3)
        require_runtime_ready(node)

        print(f"速度比例: {args.speed_scale:.3f}，理论线速度约 {target_speed_mps:.3f} m/s")
        print(f"往返次数: {args.round_trips}，每段停顿: {args.pause_sec:.1f} 秒")
        print(f"日志文件: {log_path}")
        print("确认机器人在平整、安全、可控的测试场地，周围无人和障碍物。")
        if input("输入 GO 开始运动，其他输入取消：").strip() != "GO":
            print("已取消；机器人未收到运动指令。")
            return 0

        os.makedirs(log_dir, exist_ok=True)
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(log_file, fieldnames=fieldnames)
        node.writer = writer
        node.writer_file = log_file
        writer.writeheader()
        log_file.flush()

        started = True
        leg_index = 0
        for round_trip in range(1, args.round_trips + 1):
            for direction, key, mask in (
                ("FORWARD", "w", 0b0011),
                ("BACKWARD", "s", 0b1100),
            ):
                require_runtime_ready(
                    node,
                    require_clear_sensors=False,
                    allowed_states=("ENABLE", "HOLD"),
                )
                leg_index += 1
                expected_names = [
                    SENSOR_NAMES[bit] for bit in range(4) if mask & (1 << bit)
                ]
                if node.triggered_mask & mask:
                    raise RuntimeError(
                        f"本段目标传感器仍处于触发状态：{expected_names}；拒绝继续运动"
                    )
                print(f"第 {leg_index}/{args.round_trips * 2} 段：{direction}，等待 {expected_names} 触发")
                departure_wall_time = time.time()
                node.event_context = {
                    "run_timestamp": run_timestamp,
                    "speed_scale": f"{args.speed_scale:.4f}",
                    "target_speed_mps": f"{target_speed_mps:.4f}",
                    "round_trip_count": args.round_trips,
                    "leg_index": leg_index,
                    "round_trip": round_trip,
                    "direction": direction,
                    "departure_time": datetime.fromtimestamp(departure_wall_time)
                        .astimezone().isoformat(timespec="milliseconds"),
                    "expected_sensor_bits": f"0x{mask:02x}",
                }
                node.expected_mask = mask
                node.sensor_trigger_mono = None
                node.sensor_trigger_wall = None
                node.gpio_trigger_seen = False
                node.first_zero_command_mono = None
                node.first_stop_feedback_mono = None
                node.feedback_duplicate_count = 0
                node.stop_rpm = args.stop_rpm
                node.leg_active = True
                common_row = {
                    "run_timestamp": run_timestamp,
                    "speed_scale": f"{args.speed_scale:.4f}",
                    "target_speed_mps": f"{target_speed_mps:.4f}",
                    "round_trip_count": args.round_trips,
                    "leg_index": leg_index,
                    "round_trip": round_trip,
                    "direction": direction,
                    "departure_time": datetime.fromtimestamp(departure_wall_time).astimezone().isoformat(timespec="milliseconds"),
                    "expected_sensor_bits": f"0x{mask:02x}",
                }
                writer.writerow({
                    **common_row,
                    "event": "DEPARTURE",
                    "trigger_sensor_names": "",
                    "trigger_time": "",
                    "stop_time": "",
                    "trigger_to_stop_ms": "",
                    "result": "RUNNING",
                    "io_raw_at_trigger": "",
                })
                log_file.flush()
                node.publish_key(key)
                result, trigger_time, trigger_wall_time, stop_time, stop_wall_time, seen_mask, raw_at_trigger = wait_for_stop(
                    node,
                    args,
                    mask,
                    "FORWARD" if direction == "FORWARD" else "BACKWARD",
                    args.leg_timeout_sec,
                )
                node.leg_active = False
                if result == "OK":
                    node.publish_key("h")
                    spin_for(node, args.pause_sec)
                else:
                    node.publish_key("h")

                trigger_names = [
                    SENSOR_NAMES[bit] for bit in range(4) if seen_mask & (1 << bit)
                ]
                writer.writerow({
                    **common_row,
                    "event": "RESULT",
                    "trigger_sensor_names": ";".join(trigger_names),
                    "trigger_time": datetime.fromtimestamp(trigger_wall_time).astimezone().isoformat(timespec="milliseconds") if trigger_wall_time else "",
                    "stop_time": datetime.fromtimestamp(stop_wall_time).astimezone().isoformat(timespec="milliseconds") if stop_wall_time else "",
                    "trigger_to_stop_ms": f"{(stop_time - trigger_time) * 1000:.1f}" if trigger_time and stop_time else "",
                    "result": result,
                    "io_raw_at_trigger": f"0x{raw_at_trigger:02x}" if trigger_time and raw_at_trigger is not None else "",
                    "sensor_to_zero_command_ms": (
                        f"{(node.first_zero_command_mono - node.sensor_trigger_mono) * 1000:.1f}"
                        if node.sensor_trigger_mono is not None and node.first_zero_command_mono is not None else ""
                    ),
                    "sensor_to_first_stop_feedback_ms": (
                        f"{(node.first_stop_feedback_mono - node.sensor_trigger_mono) * 1000:.1f}"
                        if node.sensor_trigger_mono is not None and node.first_stop_feedback_mono is not None else ""
                    ),
                    "zero_command_to_first_stop_feedback_ms": (
                        f"{(node.first_stop_feedback_mono - node.first_zero_command_mono) * 1000:.1f}"
                        if node.first_zero_command_mono is not None and node.first_stop_feedback_mono is not None else ""
                    ),
                    "feedback_duplicate_count": node.feedback_duplicate_count,
                })
                log_file.flush()
                print(f"结果: {result}" + (
                    f"，触发到停稳 {(stop_time - trigger_time) * 1000:.1f} ms"
                    if trigger_time and stop_time else ""
                ))
                if result != "OK":
                    print("测试中止；机器人已收到 HOLD。请检查传感器方向、话题和反馈。")
                    exit_code = 1
                    break
            if exit_code:
                break
        print(f"CSV 已写入: {log_path}")
    except KeyboardInterrupt:
        print("收到 Ctrl+C，正在发送 HOLD。")
        exit_code = 130
    except Exception as exc:
        print(f"测试未能继续: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if started:
            node.publish_key("h")
            spin_for(node, 0.2)
        if original_speed_scale is not None:
            try:
                set_speed_scale(node, original_speed_scale)
            except Exception as exc:
                print(f"恢复原速度参数失败（当前保持测试速度参数）: {exc}", file=sys.stderr)
                exit_code = 1
        if log_file is not None:
            log_file.close()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
