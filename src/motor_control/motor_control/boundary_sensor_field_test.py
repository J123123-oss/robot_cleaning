#!/usr/bin/env python3
"""IMU heading-hold forward/reverse field test for the boundary sensors."""

import argparse
import csv
import math
import os
import signal
import sys
import time
from collections import deque
from datetime import datetime

import rclpy
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64MultiArray, String, UInt8


MAX_LINEAR_SPEED_MPS = 0.35
WHEEL_RADIUS_M = 0.05
MAX_WHEEL_SPEED_RPM = (
    MAX_LINEAR_SPEED_MPS * 60.0 / (2.0 * math.pi * WHEEL_RADIUS_M)
)
DEFAULT_HEADING_DEADZONE_DEG = 0.3
IMU_TIMEOUT_SEC = 0.35
CONTROL_PERIOD_SEC = 0.05
SENSOR_NAMES = ("mid_left", "mid_right", "back_left", "back_right")
EMERGENCY_STOP_TIMEOUT_SEC = 3.0


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
        self.feedback_wall_time = None
        self.feedback_seq = 0
        self.velocity_feedback = {}
        self.velocity_feedback_time = {}
        self.velocity_feedback_wall_time = {}
        self.velocity_feedback_seq = {}
        self.velocity_feedback_event_seq = 0
        self.velocity_feedback_events = deque(maxlen=512)
        self.speed_command = None
        self.speed_command_time = None
        self.speed_command_wall_time = None
        self.speed_command_seq = 0
        self.can_speed_send_events = deque(maxlen=256)
        self.imu_heading_deg = None
        self.imu_heading_time = None
        self.imu_heading_wall_time = None
        self.imu_heading_seq = 0
        self.boundary_test_status = None
        self.rtk_parameter_client = AsyncParameterClient(self, "/rtk_nav")
        self.keyboard_pub = self.create_publisher(String, "/keyboard/control", 10)
        self.test_speed_pub = self.create_publisher(
            Vector3, "/boundary_test/motor_speed", 10
        )
        self.test_control_pub = self.create_publisher(
            Bool, "/boundary_test/control_active", 10
        )
        self.create_subscription(UInt8, "/io_data", self._io_callback, 10)
        self.create_subscription(String, "/motor/state", self._state_callback, 10)
        self.create_subscription(String, "/control/mode", self._mode_callback, 10)
        self.create_subscription(Float32, "/imu_heading", self._imu_heading_callback, 10)
        self.create_subscription(
            String, "/boundary_test/status", self._boundary_test_status_callback, 10
        )
        self.create_subscription(
            Vector3, "/motor/current_speed", self._speed_command_callback, 10
        )
        self.create_subscription(
            Float64MultiArray,
            "/motor/can_speed_send_trace",
            self._can_speed_send_trace_callback,
            20,
        )
        self.create_subscription(
            Float32MultiArray, "/motor_feedback", self._feedback_callback, 10
        )
        self.create_subscription(
            Float32MultiArray,
            "/motor_velocity_feedback",
            self._velocity_feedback_callback,
            10,
        )

    def _imu_heading_callback(self, msg):
        heading = float(msg.data)
        if not math.isfinite(heading):
            return
        self.imu_heading_deg = (heading + 180.0) % 360.0 - 180.0
        self.imu_heading_time = time.monotonic()
        self.imu_heading_wall_time = time.time()
        self.imu_heading_seq += 1

    def _boundary_test_status_callback(self, msg):
        self.boundary_test_status = msg.data.strip().upper()

    def _io_callback(self, msg):
        self.io_raw = int(msg.data) & 0x0F
        self.io_time = time.monotonic()
        self.io_wall_time = time.time()

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
        self.feedback_wall_time = time.time()
        self.feedback_seq += 1

    def _velocity_feedback_callback(self, msg):
        values = list(msg.data)
        if len(values) != 2:
            return
        motor_id = int(values[0])
        if motor_id not in (1, 2):
            return
        actual_rpm = float(values[1])
        sample_time = time.monotonic()
        sample_wall_time = time.time()
        self.velocity_feedback[motor_id] = actual_rpm
        self.velocity_feedback_time[motor_id] = sample_time
        self.velocity_feedback_wall_time[motor_id] = sample_wall_time
        self.velocity_feedback_seq[motor_id] = (
            self.velocity_feedback_seq.get(motor_id, 0) + 1
        )
        self.velocity_feedback_event_seq += 1
        self.velocity_feedback_events.append({
            "seq": self.velocity_feedback_event_seq,
            "motor_id": motor_id,
            "actual_rpm": actual_rpm,
            "monotonic": sample_time,
            "wall_time": sample_wall_time,
        })

    def _speed_command_callback(self, msg):
        self.speed_command = (float(msg.x), float(msg.y))
        self.speed_command_time = time.monotonic()
        self.speed_command_wall_time = time.time()
        self.speed_command_seq += 1

    def _can_speed_send_trace_callback(self, msg):
        if len(msg.data) != 4:
            return
        motor_id = int(msg.data[0])
        target_pulses = int(msg.data[1])
        if motor_id not in (1, 2) or target_pulses != 0:
            return
        self.can_speed_send_events.append({
            "motor_id": motor_id,
            "target_pulses": target_pulses,
            "send_monotonic": float(msg.data[2]),
            "send_wall_time": float(msg.data[3]),
        })

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

    def publish_test_control(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.test_control_pub.publish(msg)

    def publish_test_speed(self, left_speed, right_speed, direction_code):
        msg = Vector3()
        msg.x = float(left_speed)
        msg.y = float(right_speed)
        msg.z = float(direction_code)
        self.test_speed_pub.publish(msg)


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def stop_and_wait_after_interrupt(node, stop_rpm, started):
    """Repeatedly request HOLD/zero speed and verify fresh wheel feedback is stopped."""
    stop_started = time.monotonic()
    deadline = stop_started + EMERGENCY_STOP_TIMEOUT_SEC
    last_pair_seq = {
        motor_id: node.velocity_feedback_seq.get(motor_id, 0)
        for motor_id in (1, 2)
    }
    stable_pair_count = 0
    next_stop_publish = 0.0

    if started:
        node.publish_key("h")

    print("正在重复发送零速和解除测试控制，并等待左右轮反馈确认停稳……")
    while rclpy.ok() and time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_stop_publish:
            node.publish_test_speed(0.0, 0.0, 0)
            node.publish_test_control(False)
            next_stop_publish = now + 0.1

        rclpy.spin_once(node, timeout_sec=0.02)
        current_pair_seq = {
            motor_id: node.velocity_feedback_seq.get(motor_id, 0)
            for motor_id in (1, 2)
        }
        if not all(
            current_pair_seq[motor_id] > last_pair_seq[motor_id]
            for motor_id in (1, 2)
        ):
            continue

        last_pair_seq = current_pair_seq
        sample_times = [
            node.velocity_feedback_time.get(motor_id, 0.0) for motor_id in (1, 2)
        ]
        sample_now = time.monotonic()
        fresh_and_aligned = (
            min(sample_times) >= stop_started
            and max(sample_times) - min(sample_times) <= 0.15
            and all(sample_now - sample_time <= 0.2 for sample_time in sample_times)
        )
        stopped = all(
            abs(node.velocity_feedback.get(motor_id, float("inf"))) <= stop_rpm
            for motor_id in (1, 2)
        )
        if fresh_and_aligned and stopped:
            stable_pair_count += 1
            if stable_pair_count >= 3:
                elapsed_ms = (sample_now - stop_started) * 1000.0
                print(
                    "停车已确认：左右轮连续3组新反馈均低于 "
                    f"{stop_rpm:.2f} RPM，用时 {elapsed_ms:.1f} ms。"
                )
                return True
        else:
            stable_pair_count = 0

    left_rpm = node.velocity_feedback.get(1)
    right_rpm = node.velocity_feedback.get(2)
    left_age = (
        (time.monotonic() - node.velocity_feedback_time[1]) * 1000.0
        if 1 in node.velocity_feedback_time else None
    )
    right_age = (
        (time.monotonic() - node.velocity_feedback_time[2]) * 1000.0
        if 2 in node.velocity_feedback_time else None
    )
    print(
        "停止请求已发送，但未能在 "
        f"{EMERGENCY_STOP_TIMEOUT_SEC:.1f} 秒内确认停稳；"
        f"当前轮速 L/R={left_rpm!s}/{right_rpm!s} RPM，"
        f"反馈年龄 L/R={left_age!s}/{right_age!s} ms，"
        f"控制状态={node.boundary_test_status or '未知'}。"
    )
    return False


def await_future(node, future, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError("读取 /rtk_nav 参数超时")
    return future.result()


def get_rtk_heading_parameters(node):
    if not node.rtk_parameter_client.wait_for_services(timeout_sec=3.0):
        raise RuntimeError("找不到 /rtk_nav 参数服务；请确认 RTK 导航节点正在运行")
    response = await_future(
        node,
        node.rtk_parameter_client.get_parameters(
            ["rtk_max_correction", "rtk_correction_ratio"]
        ),
    )
    if len(response.values) != 2 or any(
        value.type != ParameterType.PARAMETER_DOUBLE for value in response.values
    ):
        raise RuntimeError("/rtk_nav 的 rtk_max_correction/rtk_correction_ratio 参数类型无效")
    max_correction = float(response.values[0].double_value)
    correction_ratio = float(response.values[1].double_value)
    if (
        not math.isfinite(max_correction)
        or not math.isfinite(correction_ratio)
        or max_correction < 0.0
        or correction_ratio < 0.0
    ):
        raise RuntimeError("/rtk_nav 航向纠偏参数无效")
    return max_correction, correction_ratio


def update_zero_can_send_metrics(node, metrics, zero_command_time):
    """Associate the zero-speed topic event with the preceding drive-wheel CAN sends."""
    lower_bound = zero_command_time - 0.25
    latest_by_motor = {}
    for event in node.can_speed_send_events:
        send_time = event["send_monotonic"]
        if not lower_bound <= send_time <= zero_command_time + 0.005:
            continue
        motor_id = event["motor_id"]
        if (
            motor_id not in latest_by_motor
            or send_time > latest_by_motor[motor_id]["send_monotonic"]
        ):
            latest_by_motor[motor_id] = event

    if not {1, 2}.issubset(latest_by_motor):
        return

    left_event = latest_by_motor[1]
    right_event = latest_by_motor[2]
    metrics["left_zero_can_send_time"] = left_event["send_monotonic"]
    metrics["left_zero_can_send_wall_time"] = left_event["send_wall_time"]
    metrics["right_zero_can_send_time"] = right_event["send_monotonic"]
    metrics["right_zero_can_send_wall_time"] = right_event["send_wall_time"]
    later_event = max(
        (left_event, right_event), key=lambda item: item["send_monotonic"]
    )
    metrics["both_zero_can_send_time"] = later_event["send_monotonic"]
    metrics["both_zero_can_send_wall_time"] = later_event["send_wall_time"]


def normalize_heading_error(target_deg, current_deg):
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def capture_target_heading(node, capture_sec=1.5):
    """Capture a stable circular mean while the robot is stationary."""
    deadline = time.monotonic() + capture_sec
    samples = []
    last_seq = node.imu_heading_seq
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.imu_heading_seq != last_seq:
            last_seq = node.imu_heading_seq
            samples.append(node.imu_heading_deg)

    if len(samples) < 8 or node.imu_heading_time is None:
        raise RuntimeError("没有足够的新鲜 /imu_heading 样本；检查 IMU 话题是否在发布")
    if time.monotonic() - node.imu_heading_time > IMU_TIMEOUT_SEC:
        raise RuntimeError("采集目标航向时 /imu_heading 已超时")

    radians = [math.radians(value) for value in samples]
    mean_sin = sum(math.sin(value) for value in radians)
    mean_cos = sum(math.cos(value) for value in radians)
    if math.hypot(mean_sin, mean_cos) < 1e-6:
        raise RuntimeError("目标航向样本分散，无法确定稳定方向")
    target_deg = math.degrees(math.atan2(mean_sin, mean_cos))
    deviations = [
        abs(normalize_heading_error(target_deg, value)) for value in samples
    ]
    if max(deviations) > 3.0:
        raise RuntimeError(
            f"采集期间航向变化过大（最大偏差 {max(deviations):.1f}°）；"
            "请摆正并扶稳机器人后重新运行"
        )
    return (target_deg + 180.0) % 360.0 - 180.0, len(samples), max(deviations)


def await_test_control_active(node, timeout_sec=3.0):
    node.boundary_test_status = None
    node.publish_test_control(True)
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.boundary_test_status == "ACTIVE":
            return
        if node.boundary_test_status and node.boundary_test_status != "STOPPED":
            raise RuntimeError(
                "底盘未接受测试控制："
                f"{node.boundary_test_status}；请检查模式、使能状态和 motor_control 更新版本"
            )
    raise RuntimeError(
        "没有收到 /boundary_test/status=ACTIVE；请构建并重启更新后的 motor_control"
    )


def hold_test_control(node, seconds):
    deadline = time.monotonic() + seconds
    next_publish = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        if node.boundary_test_status != "ACTIVE":
            raise RuntimeError(
                f"底盘测试控制已退出：{node.boundary_test_status or '无状态'}"
            )
        now = time.monotonic()
        if now >= next_publish:
            node.publish_test_speed(0.0, 0.0, 0)
            next_publish = now + 0.1
        rclpy.spin_once(node, timeout_sec=0.02)


def calculate_heading_command(args, direction, target_heading, current_heading):
    heading_error = normalize_heading_error(target_heading, current_heading)
    base_rpm = MAX_WHEEL_SPEED_RPM * args.speed_scale
    correction_limit = min(
        args.heading_max_correction_rpm,
        base_rpm * 0.5,
    )
    if abs(heading_error) <= args.heading_deadzone_deg:
        correction = 0.0
    else:
        stanley_heading_angle = max(-45.0, min(45.0, -heading_error))
        correction = stanley_heading_angle * args.heading_kp
        correction = max(
            -correction_limit,
            min(correction_limit, correction),
        )

    if direction == "FORWARD":
        left_speed = -base_rpm + correction
        right_speed = base_rpm + correction
        direction_code = 1
    else:
        left_speed = base_rpm + correction
        right_speed = -base_rpm + correction
        direction_code = -1

    peak = max(abs(left_speed), abs(right_speed))
    if peak > MAX_WHEEL_SPEED_RPM:
        scale = MAX_WHEEL_SPEED_RPM / peak
        left_speed *= scale
        right_speed *= scale
    return heading_error, correction, left_speed, right_speed, direction_code


def parse_args():
    parser = argparse.ArgumentParser(
        description="IMU保持航向前后往返，记录边界传感器触发到驱动轮停稳的时间。"
    )
    parser.add_argument("--round-trips", type=int, default=3, help="往返次数（默认3）")
    parser.add_argument(
        "--speed-scale", type=float, default=0.10,
        help="手动前后速度比例，范围0.01到1.0；默认0.10，约0.035m/s",
    )
    parser.add_argument(
        "--heading-kp", type=float, default=None,
        help="航向比例增益（r/min/度），默认从运行中的 /rtk_nav 读取",
    )
    parser.add_argument(
        "--heading-max-correction-rpm", type=float, default=None,
        help="最大左右轮共同速度修正量（r/min），默认从运行中的 /rtk_nav 读取",
    )
    parser.add_argument(
        "--heading-deadzone-deg", type=float, default=DEFAULT_HEADING_DEADZONE_DEG,
        help="航向误差死区（度）",
    )
    parser.add_argument(
        "--max-heading-error-deg", type=float, default=30.0,
        help="运行中允许的最大航向偏差，超限停车（默认30度）",
    )
    parser.add_argument("--pause-sec", type=float, default=2.0, help="每次停稳后的暂停秒数")
    parser.add_argument("--leg-timeout-sec", type=float, default=30.0, help="每段等待传感器触发的超时")
    parser.add_argument("--stop-timeout-sec", type=float, default=5.0, help="触发后等待轮子停稳的超时")
    parser.add_argument("--stop-rpm", type=float, default=1.0, help="轮子停稳阈值（绝对转速RPM）")
    parser.add_argument("--log-dir", default="~/robot_cleaning/test_logs", help="CSV日志目录")
    return parser.parse_args()


def require_runtime_ready(
    node, require_clear_sensors=True, allowed_states=("ENABLE", "HOLD"),
    keep_test_control=False,
):
    if keep_test_control:
        hold_test_control(node, 1.0)
    else:
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
    if not {1, 2}.issubset(node.velocity_feedback):
        raise RuntimeError(
            "/motor_velocity_feedback 没有左右轮新鲜采样；"
            "请构建并启动更新后的 motor_control"
        )
    if any(
        now - node.velocity_feedback_time.get(motor_id, 0.0) > 0.5
        for motor_id in (1, 2)
    ):
        raise RuntimeError("/motor_velocity_feedback 数据超时")
    if require_clear_sensors and node.triggered_mask != 0:
        raise RuntimeError(
            f"传感器基线不是全释放：io_data=0x{node.io_raw:02x}, "
            f"triggered=0x{node.triggered_mask:01x}；请先排除触发/接线问题"
        )
    if any(
        abs(node.velocity_feedback[motor_id]) > 1.0 for motor_id in (1, 2)
    ):
        raise RuntimeError("启动前驱动轮还在转；请先让车停稳")


def wait_for_stop(
    node, args, direction, direction_mask, leg_timeout, target_heading,
    writer, common_row, log_file,
):
    leg_start = time.monotonic()
    trigger_time = None
    trigger_wall_time = None
    raw_at_trigger = None
    trigger_mask = 0
    next_command_at = 0.0
    last_terminal_log = 0.0
    last_csv_flush = time.monotonic()
    stop_count = 0
    last_velocity_seq = {
        motor_id: node.velocity_feedback_seq.get(motor_id, 0)
        for motor_id in (1, 2)
    }
    velocity_pair_sample_times = []
    metrics = {
        "zero_command_time": None,
        "zero_command_wall_time": None,
        "left_zero_can_send_time": None,
        "left_zero_can_send_wall_time": None,
        "right_zero_can_send_time": None,
        "right_zero_can_send_wall_time": None,
        "both_zero_can_send_time": None,
        "both_zero_can_send_wall_time": None,
        "first_stop_feedback_time": None,
        "first_stop_feedback_wall_time": None,
        "stable_stop_feedback_time": None,
        "stable_stop_feedback_wall_time": None,
        "velocity_pair_sample_count": 0,
        "velocity_pair_interval_avg_ms": None,
        "velocity_pair_interval_max_ms": None,
        "velocity_sample_events": [],
        "last_velocity_event_seq": 0,
        "last_heading_error_deg": None,
        "last_heading_correction_rpm": None,
        "last_command_left_rpm": None,
        "last_command_right_rpm": None,
    }

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.monotonic()
        if node.boundary_test_status != "ACTIVE":
            node.publish_test_speed(0.0, 0.0, 0)
            return "TEST_CONTROL_LOST", trigger_time, trigger_wall_time, None, None, trigger_mask, raw_at_trigger, metrics

        mask = node.triggered_mask
        if node.io_time is None or now - node.io_time > 0.5:
            node.publish_test_speed(0.0, 0.0, 0)
            return "IO_TIMEOUT", trigger_time, trigger_wall_time, None, None, trigger_mask, raw_at_trigger, metrics

        if trigger_time is None:
            if mask & direction_mask:
                trigger_time = node.io_time
                trigger_wall_time = node.io_wall_time
                raw_at_trigger = node.io_raw
                trigger_mask = mask & direction_mask
                last_velocity_seq = {
                    motor_id: node.velocity_feedback_seq.get(motor_id, 0)
                    for motor_id in (1, 2)
                }
                node.publish_test_speed(0.0, 0.0, 0)
                next_command_at = now + CONTROL_PERIOD_SEC
            elif now - leg_start >= leg_timeout:
                node.publish_test_speed(0.0, 0.0, 0)
                return "NO_TRIGGER_TIMEOUT", None, None, None, None, 0, None, metrics
        else:
            if mask:
                trigger_mask |= mask

        if now >= next_command_at:
            imu_fresh = (
                node.imu_heading_deg is not None
                and node.imu_heading_time is not None
                and now - node.imu_heading_time <= IMU_TIMEOUT_SEC
            )
            if trigger_time is None:
                if not imu_fresh:
                    node.publish_test_speed(0.0, 0.0, 0)
                    return "IMU_TIMEOUT", None, None, None, None, 0, None, metrics
                heading_error, correction, left_speed, right_speed, direction_code = (
                    calculate_heading_command(
                        args, direction, target_heading, node.imu_heading_deg
                    )
                )
                if abs(heading_error) > args.max_heading_error_deg:
                    node.publish_test_speed(0.0, 0.0, 0)
                    return "HEADING_ERROR_LIMIT", None, None, None, None, 0, None, metrics
                node.publish_test_speed(left_speed, right_speed, direction_code)
            else:
                heading_error = (
                    normalize_heading_error(target_heading, node.imu_heading_deg)
                    if node.imu_heading_deg is not None else None
                )
                correction = 0.0
                left_speed = 0.0
                right_speed = 0.0
                node.publish_test_speed(0.0, 0.0, 0)

            metrics["last_heading_error_deg"] = heading_error
            metrics["last_heading_correction_rpm"] = correction
            metrics["last_command_left_rpm"] = left_speed
            metrics["last_command_right_rpm"] = right_speed
            heading_age_ms = (
                (now - node.imu_heading_time) * 1000.0
                if node.imu_heading_time is not None else None
            )
            if writer is not None:
                writer.writerow({
                    **common_row,
                    "event": "IMU_SAMPLE",
                    "sample_time": datetime.fromtimestamp(
                        node.imu_heading_wall_time
                        if node.imu_heading_wall_time is not None else time.time()
                    ).astimezone().isoformat(timespec="milliseconds"),
                    "target_heading_deg": f"{target_heading:.3f}",
                    "imu_heading_deg": f"{node.imu_heading_deg:.3f}"
                    if node.imu_heading_deg is not None else "",
                    "heading_error_deg": f"{heading_error:.3f}"
                    if heading_error is not None else "",
                    "imu_age_ms": f"{heading_age_ms:.1f}"
                    if heading_age_ms is not None else "",
                    "heading_correction_rpm": f"{correction:.3f}",
                    "command_left_rpm": f"{left_speed:.3f}",
                    "command_right_rpm": f"{right_speed:.3f}",
                    "result": "TRACE",
                })
                if now - last_csv_flush >= 1.0:
                    log_file.flush()
                    last_csv_flush = now

            if now - last_terminal_log >= 0.5:
                phase = "STOPPING" if trigger_time is not None else "DRIVE"
                yaw_text = (
                    f"{node.imu_heading_deg:.2f}°"
                    if node.imu_heading_deg is not None else "--"
                )
                error_text = f"{heading_error:+.2f}°" if heading_error is not None else "--"
                age_text = f"{heading_age_ms:.0f}ms" if heading_age_ms is not None else "--"
                print(
                    f"  [IMU/{phase}] target={target_heading:.2f}° current={yaw_text} "
                    f"error={error_text} age={age_text} "
                    f"corr={correction:+.2f}rpm L/R={left_speed:+.2f}/{right_speed:+.2f}rpm"
                )
                last_terminal_log = now
            next_command_at = now + CONTROL_PERIOD_SEC

        if trigger_time is None:
            continue
        else:

            if (
                metrics["zero_command_time"] is None
                and node.speed_command_time is not None
                and node.speed_command_time >= trigger_time
                and node.speed_command is not None
                and all(abs(speed) <= 0.01 for speed in node.speed_command)
            ):
                metrics["zero_command_time"] = node.speed_command_time
                metrics["zero_command_wall_time"] = node.speed_command_wall_time
                last_velocity_seq = {
                    motor_id: node.velocity_feedback_seq.get(motor_id, 0)
                    for motor_id in (1, 2)
                }

            if metrics["zero_command_time"] is not None:
                update_zero_can_send_metrics(
                    node, metrics, metrics["zero_command_time"]
                )
                if metrics["both_zero_can_send_time"] is not None:
                    for event in node.velocity_feedback_events:
                        if event["seq"] <= metrics["last_velocity_event_seq"]:
                            continue
                        metrics["last_velocity_event_seq"] = event["seq"]
                        motor_id = event["motor_id"]
                        zero_send_time = metrics[
                            "left_zero_can_send_time"
                            if motor_id == 1
                            else "right_zero_can_send_time"
                        ]
                        if (
                            zero_send_time is not None
                            and event["monotonic"] >= zero_send_time
                        ):
                            metrics["velocity_sample_events"].append(event)

            current_velocity_seq = {
                motor_id: node.velocity_feedback_seq.get(motor_id, 0)
                for motor_id in (1, 2)
            }
            if (
                metrics["zero_command_time"] is not None
                and all(
                    current_velocity_seq[motor_id] > last_velocity_seq[motor_id]
                    for motor_id in (1, 2)
                )
            ):
                last_velocity_seq = current_velocity_seq
                sample_times = [
                    node.velocity_feedback_time[motor_id]
                    for motor_id in (1, 2)
                ]
                sample_wall_times = [
                    node.velocity_feedback_wall_time[motor_id] for motor_id in (1, 2)
                ]
                sample_time = max(sample_times)
                sample_wall_time = max(sample_wall_times)
                measurement_start = metrics["zero_command_time"]
                samples_are_aligned = max(sample_times) - min(sample_times) <= 0.1
                samples_are_fresh = all(now - value <= 0.15 for value in sample_times)

                if (
                    samples_are_aligned
                    and samples_are_fresh
                    and all(value >= measurement_start for value in sample_times)
                ):
                    velocity_pair_sample_times.append(sample_time)
                    metrics["velocity_pair_sample_count"] += 1
                    if len(velocity_pair_sample_times) > 1:
                        intervals_ms = [
                            (later - earlier) * 1000.0
                            for earlier, later in zip(
                                velocity_pair_sample_times,
                                velocity_pair_sample_times[1:],
                            )
                        ]
                        metrics["velocity_pair_interval_avg_ms"] = (
                            sum(intervals_ms) / len(intervals_ms)
                        )
                        metrics["velocity_pair_interval_max_ms"] = max(intervals_ms)

                    stopped = all(
                        abs(node.velocity_feedback.get(motor_id, float("inf")))
                        <= args.stop_rpm
                        for motor_id in (1, 2)
                    )
                    if stopped:
                        if metrics["first_stop_feedback_time"] is None:
                            metrics["first_stop_feedback_time"] = sample_time
                            metrics["first_stop_feedback_wall_time"] = sample_wall_time
                        stop_count += 1
                    else:
                        stop_count = 0
                else:
                    stop_count = 0

                if stop_count >= 3:
                    metrics["stable_stop_feedback_time"] = sample_time
                    metrics["stable_stop_feedback_wall_time"] = sample_wall_time
                    return (
                        "OK", trigger_time, trigger_wall_time,
                        sample_time, sample_wall_time,
                        trigger_mask, raw_at_trigger, metrics,
                    )
            if now - trigger_time >= args.stop_timeout_sec:
                return "STOP_TIMEOUT", trigger_time, trigger_wall_time, None, None, trigger_mask, raw_at_trigger, metrics
    return "INTERRUPTED", trigger_time, trigger_wall_time, None, None, trigger_mask, raw_at_trigger, metrics


def main():
    args = parse_args()
    if args.round_trips < 1:
        raise SystemExit("--round-trips 必须至少为1")
    if not 0.01 <= args.speed_scale <= 1.0:
        raise SystemExit("--speed-scale 范围是0.01到1.0")
    if min(args.pause_sec, args.leg_timeout_sec, args.stop_timeout_sec) <= 0:
        raise SystemExit("暂停和超时时间必须大于0")
    if args.stop_rpm <= 0:
        raise SystemExit("--stop-rpm 必须大于0")
    if any(
        not math.isfinite(value)
        for value in (
            args.speed_scale,
            args.pause_sec,
            args.leg_timeout_sec,
            args.stop_timeout_sec,
            args.stop_rpm,
            args.heading_kp,
            args.heading_max_correction_rpm,
            args.heading_deadzone_deg,
            args.max_heading_error_deg,
        )
        if value is not None
    ):
        raise SystemExit("速度、增益和时间参数必须是有限数值")
    if (
        (args.heading_kp is not None and args.heading_kp < 0)
        or (
            args.heading_max_correction_rpm is not None
            and args.heading_max_correction_rpm < 0
        )
    ):
        raise SystemExit("航向增益和最大纠偏量不能小于0")
    if args.heading_deadzone_deg < 0 or args.max_heading_error_deg <= 0:
        raise SystemExit("航向死区不能小于0，最大航向误差必须大于0")

    # Let Python raise KeyboardInterrupt on Ctrl+C without rclpy first shutting
    # down the context; the finally block needs live ROS publishers to stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BoundaryTestNode()
    log_file = None
    started = False
    control_requested = False
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
        "heading_kp", "heading_max_correction_rpm", "heading_deadzone_deg",
        "event", "leg_index", "round_trip", "direction", "departure_time",
        "expected_sensor_bits", "target_heading_deg", "imu_heading_deg",
        "heading_error_deg", "imu_age_ms", "heading_correction_rpm",
        "command_left_rpm", "command_right_rpm",
        "trigger_sensor_names", "trigger_time", "stop_time", "trigger_to_stop_ms",
        "stop_command_time", "trigger_to_stop_command_ms",
        "left_zero_can_send_time", "right_zero_can_send_time",
        "trigger_to_left_zero_can_send_ms", "trigger_to_right_zero_can_send_ms",
        "trigger_to_both_zero_can_send_ms", "both_zero_can_send_to_topic_ms",
        "first_stop_feedback_time", "stop_command_to_first_feedback_ms",
        "both_zero_can_send_to_first_feedback_ms",
        "stable_stop_feedback_time", "stop_command_to_stable_feedback_ms",
        "velocity_pair_sample_count", "velocity_pair_interval_avg_ms",
        "velocity_pair_interval_max_ms", "velocity_sample_count",
        "sample_time", "sample_motor_id", "sample_actual_rpm",
        "sample_since_zero_can_send_ms", "sample_since_zero_topic_ms",
        "result", "io_raw_at_trigger",
    ]
    writer = None

    try:
        require_runtime_ready(node)
        rtk_max_correction, rtk_correction_ratio = get_rtk_heading_parameters(node)
        if args.heading_max_correction_rpm is None:
            args.heading_max_correction_rpm = rtk_max_correction
        if args.heading_kp is None:
            args.heading_kp = rtk_max_correction * rtk_correction_ratio / 45.0

        print(f"速度比例: {args.speed_scale:.3f}，理论线速度约 {target_speed_mps:.3f} m/s")
        print(f"往返次数: {args.round_trips}，每段停顿: {args.pause_sec:.1f} 秒")
        print(
            f"IMU航向保持: Kp={args.heading_kp:.3f} rpm/度，"
            f"最大纠偏={args.heading_max_correction_rpm:.2f} rpm，"
            f"死区={args.heading_deadzone_deg:.2f}度"
        )
        print(
            "纠偏默认值读取自运行中的 /rtk_nav："
            f"rtk_max_correction={rtk_max_correction:.3f} rpm，"
            f"rtk_correction_ratio={rtk_correction_ratio:.3f}"
        )
        print(f"日志文件: {log_path}")
        print(
            "确认机器人在平整、安全、可控的测试场地；GO 后会静止采集当前 IMU 航向作为目标。"
        )
        if input("输入 GO 开始运动，其他输入取消：").strip() != "GO":
            print("已取消；机器人未收到运动指令。")
            return 0

        os.makedirs(log_dir, exist_ok=True)
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(log_file, fieldnames=fieldnames)
        writer.writeheader()
        log_file.flush()

        target_heading, heading_sample_count, heading_spread = capture_target_heading(node)
        print(
            f"目标航向已锁定: {target_heading:.2f}° "
            f"（{heading_sample_count}个样本，最大波动{heading_spread:.2f}°）"
        )
        control_requested = True
        await_test_control_active(node)
        started = True
        print("底盘测试控制已就绪；前后方向都保持同一个目标航向。")

        leg_index = 0
        for round_trip in range(1, args.round_trips + 1):
            for direction, mask in (
                ("FORWARD", 0b0011),
                ("BACKWARD", 0b1100),
            ):
                require_runtime_ready(
                    node,
                    require_clear_sensors=False,
                    allowed_states=("ENABLE", "HOLD"),
                    keep_test_control=started,
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
                common_row = {
                    "run_timestamp": run_timestamp,
                    "speed_scale": f"{args.speed_scale:.4f}",
                    "target_speed_mps": f"{target_speed_mps:.4f}",
                    "round_trip_count": args.round_trips,
                    "heading_kp": f"{args.heading_kp:.6f}",
                    "heading_max_correction_rpm": f"{args.heading_max_correction_rpm:.6f}",
                    "heading_deadzone_deg": f"{args.heading_deadzone_deg:.3f}",
                    "leg_index": leg_index,
                    "round_trip": round_trip,
                    "direction": direction,
                    "departure_time": datetime.fromtimestamp(departure_wall_time).astimezone().isoformat(timespec="milliseconds"),
                    "expected_sensor_bits": f"0x{mask:02x}",
                    "target_heading_deg": f"{target_heading:.3f}",
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
                result, trigger_time, trigger_wall_time, stop_time, stop_wall_time, seen_mask, raw_at_trigger, metrics = wait_for_stop(
                    node,
                    args,
                    direction,
                    mask,
                    args.leg_timeout_sec,
                    target_heading,
                    writer,
                    common_row,
                    log_file,
                )
                if result == "OK":
                    hold_test_control(node, args.pause_sec)
                else:
                    node.publish_test_speed(0.0, 0.0, 0)

                trigger_names = [
                    SENSOR_NAMES[bit] for bit in range(4) if seen_mask & (1 << bit)
                ]
                zero_command_time = metrics["zero_command_time"]
                first_stop_feedback_time = metrics["first_stop_feedback_time"]
                stable_stop_feedback_time = metrics["stable_stop_feedback_time"]
                latency_fields = {
                    "stop_command_time": datetime.fromtimestamp(
                        metrics["zero_command_wall_time"]
                    ).astimezone().isoformat(timespec="milliseconds")
                    if metrics["zero_command_wall_time"] else "",
                    "trigger_to_stop_command_ms": f"{(zero_command_time - trigger_time) * 1000:.1f}"
                    if trigger_time is not None and zero_command_time is not None else "",
                    "left_zero_can_send_time": datetime.fromtimestamp(
                        metrics["left_zero_can_send_wall_time"]
                    ).astimezone().isoformat(timespec="milliseconds")
                    if metrics["left_zero_can_send_wall_time"] else "",
                    "right_zero_can_send_time": datetime.fromtimestamp(
                        metrics["right_zero_can_send_wall_time"]
                    ).astimezone().isoformat(timespec="milliseconds")
                    if metrics["right_zero_can_send_wall_time"] else "",
                    "trigger_to_left_zero_can_send_ms": f"{(metrics['left_zero_can_send_time'] - trigger_time) * 1000:.1f}"
                    if trigger_time is not None and metrics["left_zero_can_send_time"] is not None else "",
                    "trigger_to_right_zero_can_send_ms": f"{(metrics['right_zero_can_send_time'] - trigger_time) * 1000:.1f}"
                    if trigger_time is not None and metrics["right_zero_can_send_time"] is not None else "",
                    "trigger_to_both_zero_can_send_ms": f"{(metrics['both_zero_can_send_time'] - trigger_time) * 1000:.1f}"
                    if trigger_time is not None and metrics["both_zero_can_send_time"] is not None else "",
                    "both_zero_can_send_to_topic_ms": f"{(zero_command_time - metrics['both_zero_can_send_time']) * 1000:.1f}"
                    if zero_command_time is not None and metrics["both_zero_can_send_time"] is not None else "",
                    "first_stop_feedback_time": datetime.fromtimestamp(
                        metrics["first_stop_feedback_wall_time"]
                    ).astimezone().isoformat(timespec="milliseconds")
                    if metrics["first_stop_feedback_wall_time"] else "",
                    "stop_command_to_first_feedback_ms": f"{(first_stop_feedback_time - zero_command_time) * 1000:.1f}"
                    if first_stop_feedback_time is not None and zero_command_time is not None else "",
                    "both_zero_can_send_to_first_feedback_ms": f"{(first_stop_feedback_time - metrics['both_zero_can_send_time']) * 1000:.1f}"
                    if first_stop_feedback_time is not None and metrics["both_zero_can_send_time"] is not None else "",
                    "stable_stop_feedback_time": datetime.fromtimestamp(
                        metrics["stable_stop_feedback_wall_time"]
                    ).astimezone().isoformat(timespec="milliseconds")
                    if metrics["stable_stop_feedback_wall_time"] else "",
                    "stop_command_to_stable_feedback_ms": f"{(stable_stop_feedback_time - zero_command_time) * 1000:.1f}"
                    if stable_stop_feedback_time is not None and zero_command_time is not None else "",
                    "velocity_pair_sample_count": metrics[
                        "velocity_pair_sample_count"
                    ],
                    "velocity_pair_interval_avg_ms": f"{metrics['velocity_pair_interval_avg_ms']:.1f}"
                    if metrics["velocity_pair_interval_avg_ms"] is not None else "",
                    "velocity_pair_interval_max_ms": f"{metrics['velocity_pair_interval_max_ms']:.1f}"
                    if metrics["velocity_pair_interval_max_ms"] is not None else "",
                    "velocity_sample_count": len(metrics["velocity_sample_events"]),
                    "target_heading_deg": f"{target_heading:.3f}",
                    "heading_error_deg": f"{metrics['last_heading_error_deg']:.3f}"
                    if metrics["last_heading_error_deg"] is not None else "",
                    "heading_correction_rpm": f"{metrics['last_heading_correction_rpm']:.3f}"
                    if metrics["last_heading_correction_rpm"] is not None else "",
                    "command_left_rpm": f"{metrics['last_command_left_rpm']:.3f}"
                    if metrics["last_command_left_rpm"] is not None else "",
                    "command_right_rpm": f"{metrics['last_command_right_rpm']:.3f}"
                    if metrics["last_command_right_rpm"] is not None else "",
                }
                for sample in metrics["velocity_sample_events"]:
                    motor_id = sample["motor_id"]
                    zero_send_time = metrics[
                        "left_zero_can_send_time"
                        if motor_id == 1
                        else "right_zero_can_send_time"
                    ]
                    writer.writerow({
                        **common_row,
                        "event": "VELOCITY_SAMPLE",
                        "trigger_sensor_names": ";".join(trigger_names),
                        "trigger_time": datetime.fromtimestamp(
                            trigger_wall_time
                        ).astimezone().isoformat(timespec="milliseconds")
                        if trigger_wall_time else "",
                        "stop_command_time": latency_fields["stop_command_time"],
                        "velocity_sample_count": len(
                            metrics["velocity_sample_events"]
                        ),
                        "sample_time": datetime.fromtimestamp(
                            sample["wall_time"]
                        ).astimezone().isoformat(timespec="milliseconds"),
                        "sample_motor_id": motor_id,
                        "sample_actual_rpm": f"{sample['actual_rpm']:.4f}",
                        "sample_since_zero_can_send_ms": f"{(sample['monotonic'] - zero_send_time) * 1000:.1f}"
                        if zero_send_time is not None else "",
                        "sample_since_zero_topic_ms": f"{(sample['monotonic'] - zero_command_time) * 1000:.1f}"
                        if zero_command_time is not None else "",
                        "result": "TRACE",
                        "io_raw_at_trigger": f"0x{raw_at_trigger:02x}"
                        if raw_at_trigger is not None else "",
                    })
                writer.writerow({
                    **common_row,
                    "event": "RESULT",
                    "trigger_sensor_names": ";".join(trigger_names),
                    "trigger_time": datetime.fromtimestamp(trigger_wall_time).astimezone().isoformat(timespec="milliseconds") if trigger_wall_time else "",
                    "stop_time": datetime.fromtimestamp(stop_wall_time).astimezone().isoformat(timespec="milliseconds") if stop_wall_time else "",
                    "trigger_to_stop_ms": f"{(stop_time - trigger_time) * 1000:.1f}" if trigger_time and stop_time else "",
                    **latency_fields,
                    "result": result,
                    "io_raw_at_trigger": f"0x{raw_at_trigger:02x}" if trigger_time and raw_at_trigger is not None else "",
                })
                log_file.flush()
                print(f"结果: {result}")
                if trigger_time is not None and zero_command_time is not None:
                    print(
                        "  传感器触发→零速命令话题: "
                        f"{(zero_command_time - trigger_time) * 1000:.1f} ms"
                    )
                elif trigger_time is not None:
                    print("  未观测到 /motor/current_speed 的零速命令")
                if metrics["both_zero_can_send_time"] is not None:
                    print(
                        "  传感器话题采样→左右轮 60FF=0 CAN send 返回: "
                        f"{(metrics['left_zero_can_send_time'] - trigger_time) * 1000:.1f}/"
                        f"{(metrics['right_zero_can_send_time'] - trigger_time) * 1000:.1f} ms"
                    )
                    if zero_command_time is not None:
                        print(
                            "  两轮零速 CAN send 返回→零速话题: "
                            f"{(zero_command_time - metrics['both_zero_can_send_time']) * 1000:.1f} ms"
                        )
                if zero_command_time is not None and first_stop_feedback_time is not None:
                    print(
                        "  零速命令→首次停稳反馈: "
                        f"{(first_stop_feedback_time - zero_command_time) * 1000:.1f} ms"
                    )
                if (
                    metrics["both_zero_can_send_time"] is not None
                    and first_stop_feedback_time is not None
                ):
                    print(
                        "  两轮零速 CAN send 返回→首次停稳反馈: "
                        f"{(first_stop_feedback_time - metrics['both_zero_can_send_time']) * 1000:.1f} ms"
                    )
                if zero_command_time is not None and stable_stop_feedback_time is not None:
                    print(
                        "  零速命令→连续3组新CAN速度采样确认: "
                        f"{(stable_stop_feedback_time - zero_command_time) * 1000:.1f} ms"
                    )
                if metrics["velocity_pair_interval_avg_ms"] is not None:
                    print(
                        "  停止阶段左右轮新CAN反馈组间隔 avg/max: "
                        f"{metrics['velocity_pair_interval_avg_ms']:.1f}/"
                        f"{metrics['velocity_pair_interval_max_ms']:.1f} ms，"
                        f"共 {metrics['velocity_pair_sample_count']} 组"
                    )
                if metrics["velocity_sample_events"]:
                    print(
                        "  左右轮停机速度曲线样点已写入 CSV: "
                        f"{len(metrics['velocity_sample_events'])} 条"
                    )
                if trigger_time is not None and stop_time is not None:
                    print(
                        "  传感器触发→连续反馈确认停稳: "
                        f"{(stop_time - trigger_time) * 1000:.1f} ms"
                    )
                if result != "OK":
                    print("测试中止；专用速度指令已归零。请检查传感器、IMU和底盘状态。")
                    exit_code = 1
                    break
            if exit_code:
                break
        print(f"CSV 已写入: {log_path}")
    except KeyboardInterrupt:
        print("收到 Ctrl+C，正在执行并确认停机。")
        exit_code = 130
    except Exception as exc:
        print(f"测试未能继续: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        old_sigint_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if control_requested:
                try:
                    stop_and_wait_after_interrupt(node, args.stop_rpm, started)
                except Exception as exc:
                    print(
                        f"停止反馈确认过程出错：{exc}；"
                        "零速和解除测试控制请求已尝试发送。",
                        file=sys.stderr,
                    )
            if log_file is not None:
                log_file.close()
            node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                signal.signal(signal.SIGINT, old_sigint_handler)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
