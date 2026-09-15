#!/usr/bin/env python3
"""室内摄像头纠偏测试控制器。"""

import math
import signal
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray


def clamp(value, lower, upper):
    """将数值限制在闭区间内。"""
    return max(float(lower), min(float(upper), float(value)))


def compute_indoor_control(
    angle_deg,
    lateral_m,
    detected,
    confidence,
    heading_valid,
    lateral_valid,
    base_speed,
    heading_gain,
    lateral_gain,
    max_correction,
    min_confidence,
    heading_deadband_deg=0.0,
    lateral_deadband_m=0.0,
):
    """根据一帧视觉结果计算纠偏量和左右轮速度。

    返回值沿用旧版 RS02 底盘的速度符号约定：前进为 ``(-v, +v)``。
    正的转向修正同时增加左右轮命令，使机器人向现有 RTK 控制器的同一
    方向约定转向；滚刷不参与此函数，调用方应明确发送零速。

    返回值依次为：左轮速度、右轮速度、航向纠偏、横向纠偏和最终纠偏。
    """
    try:
        values = tuple(
            float(value)
            for value in (
                angle_deg,
                lateral_m,
                confidence,
                base_speed,
                heading_gain,
                lateral_gain,
                max_correction,
                min_confidence,
                heading_deadband_deg,
                lateral_deadband_m,
            )
        )
    except (TypeError, ValueError, OverflowError):
        return 0.0, 0.0, 0.0, 0.0, 0.0

    if not all(math.isfinite(value) for value in values):
        return 0.0, 0.0, 0.0, 0.0, 0.0
    (
        angle_deg,
        lateral_m,
        confidence,
        base_speed,
        heading_gain,
        lateral_gain,
        max_correction,
        min_confidence,
        heading_deadband_deg,
        lateral_deadband_m,
    ) = values
    if (
        base_speed < 0.0
        or max_correction < 0.0
        or min_confidence < 0.0
        or min_confidence > 1.0
        or heading_deadband_deg < 0.0
        or lateral_deadband_m < 0.0
        or not detected
        or (not heading_valid and not lateral_valid)
        or confidence < min_confidence
    ):
        return 0.0, 0.0, 0.0, 0.0, 0.0

    heading_correction = 0.0
    if heading_valid and abs(angle_deg) > heading_deadband_deg:
        heading_correction = -heading_gain * angle_deg
    lateral_correction = 0.0
    if lateral_valid and abs(lateral_m) > lateral_deadband_m:
        lateral_correction = -lateral_gain * lateral_m
    correction = heading_correction + lateral_correction
    correction = clamp(correction, -max_correction, max_correction)
    # 不允许纠偏把前进中的一侧轮反向，避免室内测试变成原地旋转。
    correction = clamp(correction, -base_speed, base_speed)
    return (
        -base_speed + correction,
        base_speed + correction,
        heading_correction,
        lateral_correction,
        correction,
    )


def compute_indoor_wheel_speeds(
    angle_deg,
    lateral_m,
    detected,
    confidence,
    heading_valid,
    lateral_valid,
    base_speed,
    heading_gain,
    lateral_gain,
    max_correction,
    min_confidence,
    heading_deadband_deg=0.0,
    lateral_deadband_m=0.0,
):
    """兼容旧调用方，仅返回由视觉结果计算出的左右轮速度。"""
    return compute_indoor_control(
        angle_deg,
        lateral_m,
        detected,
        confidence,
        heading_valid,
        lateral_valid,
        base_speed,
        heading_gain,
        lateral_gain,
        max_correction,
        min_confidence,
        heading_deadband_deg,
        lateral_deadband_m,
    )[:2]


class CameraIndoorTestController(Node):
    """接收视觉误差并独立驱动底盘低速直线行驶。"""

    def __init__(self):
        super().__init__('camera_indoor_test_controller')

        # 速度单位与旧版 RS02 motor_driver 一致，直接写入 float，不做脉冲换算。
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('heading_gain', 0.05)
        self.declare_parameter('lateral_gain', 5.0)
        self.declare_parameter('max_correction', 0.8)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('visual_timeout_sec', 0.5)
        self.declare_parameter('heading_deadband_deg', 1.0)
        self.declare_parameter('lateral_deadband_m', 0.03)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('brush_motor_count', 2)

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)
        self.max_correction = float(
            self.get_parameter('max_correction').value
        )
        self.min_confidence = float(
            self.get_parameter('min_confidence').value
        )
        self.visual_timeout_sec = float(
            self.get_parameter('visual_timeout_sec').value
        )
        self.heading_deadband_deg = float(
            self.get_parameter('heading_deadband_deg').value
        )
        self.lateral_deadband_m = float(
            self.get_parameter('lateral_deadband_m').value
        )
        control_frequency = float(
            self.get_parameter('control_frequency').value
        )
        self.brush_motor_count = int(
            self.get_parameter('brush_motor_count').value
        )
        if self.brush_motor_count not in (1, 2):
            raise ValueError('brush_motor_count must be 1 or 2')
        self._validate_parameters(control_frequency)

        self.angle_deg = 0.0
        self.lateral_m = 0.0
        self.detected = False
        self.confidence = 0.0
        self.heading_valid = False
        self.lateral_valid = False
        self.last_angle_time = 0.0
        self.last_confidence_time = 0.0
        self.last_heading_valid_time = 0.0
        self.last_lateral_valid_time = 0.0
        self._last_control_log_time = 0.0

        self.speed_pub = self.create_publisher(
            Float32MultiArray, '/motor_speed_commands', 10
        )
        self.create_subscription(
            Vector3,
            '/grid_line/angle_deviation',
            self.angle_callback,
            10,
        )
        self.create_subscription(
            Float32,
            '/grid_line/detection_confidence',
            self.confidence_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/grid_line/heading_valid',
            self.heading_valid_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/grid_line/lateral_valid',
            self.lateral_valid_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / control_frequency, self.timer_callback
        )
        self.get_logger().info(
            f'室内摄像头纠偏控制器启动：base_speed={self.base_speed:.2f}'
        )

    def _validate_parameters(self, control_frequency):
        """校验控制器参数，避免非法参数产生不可控 CAN 速度。"""
        values = (
            self.base_speed,
            self.heading_gain,
            self.lateral_gain,
            self.max_correction,
            self.min_confidence,
            self.visual_timeout_sec,
            self.heading_deadband_deg,
            self.lateral_deadband_m,
            control_frequency,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('indoor controller parameters must be finite')
        if not 0.0 <= self.base_speed <= 10.0:
            raise ValueError('base_speed must be in [0, 10]')
        if self.max_correction < 0.0 or self.max_correction > self.base_speed:
            raise ValueError('max_correction must be in [0, base_speed]')
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError('min_confidence must be in [0, 1]')
        if self.visual_timeout_sec <= 0.0 or control_frequency <= 0.0:
            raise ValueError('visual_timeout_sec and control_frequency must be > 0')
        if self.heading_deadband_deg < 0.0 or self.lateral_deadband_m < 0.0:
            raise ValueError('deadbands must be non-negative')

    def angle_callback(self, msg):
        """缓存同一检测帧的角度、横向误差和检测标志。"""
        try:
            values = (float(msg.x), float(msg.y), float(msg.z))
        except (AttributeError, TypeError, ValueError):
            self.last_angle_time = 0.0
            return
        if not all(math.isfinite(value) for value in values):
            self.last_angle_time = 0.0
            return
        self.angle_deg, self.lateral_m, detected = values
        self.detected = detected >= 0.5
        self.last_angle_time = time.monotonic()

    def confidence_callback(self, msg):
        """缓存视觉置信度；置信度过期时控制器停车。"""
        try:
            confidence = float(msg.data)
        except (AttributeError, TypeError, ValueError):
            self.last_confidence_time = 0.0
            return
        if not math.isfinite(confidence):
            self.last_confidence_time = 0.0
            return
        self.confidence = confidence
        self.last_confidence_time = time.monotonic()

    def heading_valid_callback(self, msg):
        """缓存视觉航向几何有效标志。"""
        self.heading_valid = bool(msg.data)
        self.last_heading_valid_time = time.monotonic()

    def lateral_valid_callback(self, msg):
        """缓存视觉横向几何有效标志。"""
        self.lateral_valid = bool(msg.data)
        self.last_lateral_valid_time = time.monotonic()

    def _is_fresh(self, timestamp, now):
        """判断单个视觉输入是否在超时时间内。"""
        return (
            timestamp > 0.0
            and 0.0 <= now - timestamp <= self.visual_timeout_sec
        )

    def timer_callback(self):
        """按视觉数据新鲜度发布四路底盘速度，滚刷始终保持停止。"""
        now = time.monotonic()
        angle_fresh = self._is_fresh(self.last_angle_time, now)
        confidence_fresh = self._is_fresh(self.last_confidence_time, now)
        heading_fresh = self._is_fresh(self.last_heading_valid_time, now)
        lateral_fresh = self._is_fresh(self.last_lateral_valid_time, now)
        (
            left_speed,
            right_speed,
            heading_correction,
            lateral_correction,
            correction,
        ) = compute_indoor_control(
            self.angle_deg,
            self.lateral_m,
            self.detected if angle_fresh else False,
            self.confidence if confidence_fresh else 0.0,
            self.heading_valid if heading_fresh else False,
            self.lateral_valid if lateral_fresh else False,
            self.base_speed,
            self.heading_gain,
            self.lateral_gain,
            self.max_correction,
            self.min_confidence,
            self.heading_deadband_deg,
            self.lateral_deadband_m,
        )
        if now - self._last_control_log_time >= 1.0:
            self.get_logger().info(
                '室内直线控制: '
                f'base_speed={self.base_speed:.3f}, '
                f'heading_correction={heading_correction:+.3f}, '
                f'lateral_correction={lateral_correction:+.3f}, '
                f'correction={correction:+.3f}, '
                f'left_speed={left_speed:+.3f}, '
                f'right_speed={right_speed:+.3f}'
            )
            self._last_control_log_time = now
        msg = Float32MultiArray()
        msg.data = [float(left_speed), float(right_speed), 0.0]
        if self.brush_motor_count == 2:
            msg.data.append(0.0)
        self.speed_pub.publish(msg)

    def publish_stop_command(self):
        """在 ROS context 有效时发布一次全零速度。"""
        if not rclpy.ok():
            return False
        try:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0]
            if self.brush_motor_count == 2:
                msg.data.append(0.0)
            self.speed_pub.publish(msg)
            return True
        except Exception as exc:
            self.get_logger().warning(f'发布室内测试停止指令失败: {exc}')
            return False

    def destroy_node(self):
        """退出前发布零速；实际 CAN 停车由 motor_driver 兜底。"""
        self.publish_stop_command()
        if rclpy.ok():
            super().destroy_node()


def main(args=None):
    """初始化并运行室内测试控制器。"""
    # 先由本节点接管信号，确保 Ctrl+C 时 context 尚未关闭即可发布零速。
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = None
    previous_handlers = {}

    def request_stop(_signum, _frame):
        raise KeyboardInterrupt

    try:
        node = CameraIndoorTestController()
        for signal_number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signal_number] = signal.getsignal(signal_number)
            signal.signal(signal_number, request_stop)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            # 多发几次，给订阅端一个短暂的 DDS 传输窗口。
            for _ in range(3):
                if not node.publish_stop_command():
                    break
                time.sleep(0.02)
    finally:
        for signal_number, previous_handler in previous_handlers.items():
            signal.signal(signal_number, previous_handler)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
