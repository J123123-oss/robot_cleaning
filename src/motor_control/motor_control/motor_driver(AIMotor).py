#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AIMOTOR CANopen/CiA402 电机驱动。

公共驱动方法有意保持旧版 motor_control API，使上层无需修改调用方式即可
切换电机驱动。本文件使用 AIMOTOR 手册规定的 11 位标准 CANopen 协议，
而不是之前的 29 位厂商协议。
"""

import math
import struct
import subprocess
import threading
import time
from typing import Optional

import can
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def build_sdo_frame(command: int, index: int, subindex: int,
                    value: bytes = b"") -> bytes:
    """构造快速传输格式的 8 字节 CANopen SDO 帧。

    ``value`` 由调用方按小端格式编码，本函数会将其填充到 CAN 数据长度。
    此辅助函数只负责组装帧格式，不校验对象类型，也不向 CAN 总线发送数据。
    """
    if not 0 <= command <= 0xFF:
        raise ValueError("SDO command must fit in one byte")
    if not 0 <= index <= 0xFFFF:
        raise ValueError("SDO index must fit in two bytes")
    if not 0 <= subindex <= 0xFF:
        raise ValueError("SDO subindex must fit in one byte")
    if len(value) > 4:
        raise ValueError("expedited SDO value cannot exceed four bytes")
    return bytes((command, index & 0xFF, index >> 8, subindex)) + value.ljust(4, b"\x00")


class CanMotorDriver(Node):
    """ROS2 节点以及 AIMOTOR CANopen 主站适配器。"""

    # CANopen 标准通信标识符。驱动使用 11 位标准帧 ID：
    # SDO_RX_BASE 为主站发往从站的请求帧基址（0x600 + 节点 ID），
    # SDO_TX_BASE 为从站返回主站的响应帧基址（0x580 + 节点 ID），
    # EMCY_BASE 为紧急报文基址（0x080 + 节点 ID），NMT 固定使用 0x000。
    SDO_RX_BASE = 0x600
    SDO_TX_BASE = 0x580
    EMCY_BASE = 0x080
    NMT_COB_ID = 0x000

    # AIMOTOR 手册第 6.1 节故障表：
    # （H0B-34 厂家故障码、603Fh CiA402 标准故障码、故障描述、
    # 硬件指示灯模式、报警类型）。同一个标准故障码可能对应多行，
    # 因此不能将此表简化为一对一字典。
    AIMOTOR_FAULT_TABLE = (
        (0x0000, 0x0000, "无故障", "绿", "-"),
        (0x0101, 0x6320, "参数 ID 超范围", "10 红 1 绿", "NO.1"),
        (0x0102, 0x6320, "参数 CRC 错误", "12 红 1 绿", "NO.1"),
        (0x0104, 0x6320, "注册参数 CRC 错误", "12 红 1 绿", "NO.1"),
        (0x0105, 0x6320, "内部程序异常触发看门狗", "11 红 1 绿", "NO.1"),
        (0x0130, 0x6320, "DI 功能重复分配", "12 红 1 绿", "NO.1"),
        (0x0201, 0x2312, "硬件过流", "4 红 1 绿", "NO.1"),
        (0x0208, 0xFF00, "ia/ic 静态电流偏置校准失败", "12 红 1 绿", "NO.1"),
        (0x0207, 0x2311, "软件过流", "4 红 1 绿", "NO.1"),
        (0x0234, 0xFF00, "飞车", "12 红 1 绿", "NO.1"),
        (0x0A33, 0x7306, "编码器数据异常", "9 红 1 绿", "NO.1"),
        (0x0400, 0x3210, "主回路电过压", "3 红 1 绿", "NO.1"),
        (0x0410, 0x3220, "主回路电欠压", "3 红 1 绿", "NO.1"),
        (0x0620, 0x3230, "电机过载", "4 红 1 绿", "NO.1"),
        (0x0650, 0x4210, "散热器过热", "6 红 1 绿", "NO.1"),
        (0x0B00, 0x8611, "位置偏差过大", "2 红 1 绿", "NO.1"),
        (0x0668, 0xFF00, "回零方式不匹配", "8 红 1 绿", "NO.2"),
        (0x0601, 0x8610, "回原点超时", "8 红 1 绿", "NO.2"),
        (0x0900, 0x5442, "紧急停机", "12 红 1 绿", "NO.3"),
        (0x0950, 0x5443, "正向超程警告", "12 红 1 绿", "NO.3"),
        (0x0952, 0x5444, "负向超程警告", "12 红 1 绿", "NO.3"),
        (0x0731, 0x7306, "编码器电池失效", "12 红 1 绿", "NO.2"),
        (0x0733, 0x7306, "编码器多圈计数错误", "12 红 1 绿", "NO.2"),
        (0x0735, 0x7306, "编码器多圈计数溢出", "12 红 1 绿", "NO.2"),
        (0x0730, 0x7307, "编码器电池警告", "12 红 1 绿", "NO.3"),
        (0x0D03, 0x8130, "CAN 通信连接中断", "12 红 1 绿", "NO.2"),
        (0x0941, 0xFF00, "变更参数需重新上电生效", "7 红 1 绿", "NO.3"),
        (0x0942, 0x7600, "参数存储频繁", "12 红 1 绿", "NO.3"),
    )

    # AIMOTOR 手册中 PV/CiA402 速度控制流程使用的对象字典索引。
    # SDO 帧中的索引按低字节在前编码；下列对象的具体数据类型和读写
    # 属性以手册为准，驱动通过这些索引完成模式、状态和速度参数访问。
    # 6040h：控制字（Uint16，RW），驱动器状态机的状态切换和故障复位入口。
    CONTROLWORD_INDEX = 0x6040
    # 6041h：状态字（Uint16，RO），读取准备、使能、运行和故障等状态位。
    STATUSWORD_INDEX = 0x6041
    # 6060h：模式设定（Int8，RW），PV 速度模式的标准值为 3。
    MODES_OF_OPERATION_INDEX = 0x6060
    # 6061h：当前模式显示（Int8，RO），用于确认 6060h 的实际生效模式。
    MODES_OF_OPERATION_DISPLAY_INDEX = 0x6061
    # 6064h：实际位置（Int32，RO），单位 Pul；驱动反馈路径换算为弧度。
    POSITION_ACTUAL_INDEX = 0x6064
    # 606Ch：实际速度（Int32，RO），单位 Pul/s；驱动换算为减速器输出轴 r/min。
    VELOCITY_ACTUAL_INDEX = 0x606C
    # 6077h：实际转矩（Int16/设备标定值，RO）；当前按手册缩放系数解析。
    TORQUE_ACTUAL_INDEX = 0x6077
    # 60FFh：目标速度（Int32，RW），单位 Pul/s；由上层输出轴 r/min 转换后写入。
    TARGET_VELOCITY_INDEX = 0x60FF
    # 6083h：速度模式加速度（Int32，RW），单位 Pul/s^2，在使能前写入。
    PROFILE_ACCELERATION_INDEX = 0x6083
    # 6084h：速度模式减速度（Int32，RW），单位 Pul/s^2，在使能前写入。
    PROFILE_DECELERATION_INDEX = 0x6084

    # 轮廓参数单位为电机侧 Pul/s^2。手册中的 PV 示例以这些参数值
    # 配置 33333 Pul/s 的目标速度。
    DEFAULT_PROFILE_ACCELERATION = 16666
    DEFAULT_PROFILE_DECELERATION = 11111
    # 目标速度和实际速度的换算先使用电机转数，再应用配置的机械减速比，
    # 得到减速器输出轴 r/min。
    DEFAULT_PULSES_PER_MOTOR_REV = 1000
    DEFAULT_MECHANICAL_REDUCTION_RATIO = 40.0
    # 当前四节点机器人默认值：驱动电机 1/2 与滚刷电机 3/4
    # 可能使用不同的齿轮箱减速比。
    DEFAULT_MECHANICAL_REDUCTION_RATIOS = {
        1: 50.0,
        2: 50.0,
        3: 40.0,
        4: 40.0,
    }
    WHEEL_RADIUS = 0.05  # 轮半径，单位 m，用于里程计计算。
    MAX_WHEEL_LINEAR_SPEED_MPS = 0.35  # 安全/参考线速度上限，单位 m/s。

    # SDO 快速传输命令字。命令字同时规定数据方向和有效数据长度：
    # 0x40 为读取请求；0x2F、0x2B、0x23 分别为写入 1、2、4 字节。
    SDO_READ = 0x40
    SDO_WRITE_1 = 0x2F
    SDO_WRITE_2 = 0x2B
    SDO_WRITE_4 = 0x23
    # SDO 读取响应命令字：0x4B 表示有效载荷 2 字节，0x43 表示 4 字节；
    # 0x80 为 SDO 中止响应，后续 4 字节携带中止原因码。
    SDO_READ_2_RESPONSE = 0x4B
    SDO_READ_4_RESPONSE = 0x43
    SDO_ABORT = 0x80

    # CiA402 模式和控制字常量。控制字通过 6040h 写入，状态反馈通过 6041h
    # 读取；初始化时必须按“关闭电压 -> 准备使能 -> 等待使能 -> 使能运行”的
    # 顺序切换，不能把这些值当作普通速度参数。
    # 6060h=3：PV（轮廓速度）模式。
    CANOPEN_VELOCITY_MODE = 0x03
    # 6040h=0x0006：关闭，关闭功率输出并进入“准备使能”状态。
    CONTROL_SHUTDOWN = 0x0006
    # 6040h=0x0007：接通，允许上电但尚未进入可运行状态。
    CONTROL_SWITCH_ON = 0x0007
    # 6040h=0x000F：使能运行，使能功率级并允许执行速度目标。
    CONTROL_ENABLE_OPERATION = 0x000F
    # 6040h=0x0000：禁止电压，撤销驱动器电压使能。
    CONTROL_DISABLE_VOLTAGE = 0x0000
    # 6040h=0x0002：快速停止，执行快速停止路径；停止后的状态取决于 605Ah。
    CONTROL_QUICK_STOP = 0x0002
    # 6040h bit7=1：故障复位，复位 CiA402 故障状态，不能替代故障原因排查。
    CONTROL_FAULT_RESET = 0x0080

    def __init__(self, node_name="can_motor_driver", channel="can1",
                 interface="socketcan", baudrate=500000, motor_ids=None,
                 pulses_per_motor_rev=DEFAULT_PULSES_PER_MOTOR_REV,
                 mechanical_reduction_ratio=DEFAULT_MECHANICAL_REDUCTION_RATIO,
                 mechanical_reduction_ratios=None,
                 profile_acceleration=DEFAULT_PROFILE_ACCELERATION,
                 profile_deceleration=DEFAULT_PROFILE_DECELERATION):
        """创建 ROS2 驱动、CAN 传输、发布器和定时器。

        公共 API 接收的 ``speed`` 单位为输出轴 r/min；驱动在写入 60FFh
        前会将其转换为电机侧 Pul/s。``motor_ids`` 用于选择配置的节点，
        可选的按 ID 减速比映射会覆盖标量备用减速比。
        """
        super().__init__(node_name)

        # CAN 总线传输配置和连接状态。
        self.can_interface = channel
        self.can_bus_interface = interface
        self.can_bitrate = int(baudrate)
        self.bus: Optional[can.Bus] = None
        self.can_initialized = False

        # 运行参数通过 ROS2 参数暴露，使 launch 文件可以调整超时时间、
        # 脉冲换算、减速比和速度轮廓加减速参数。
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("command_timeout_sec", 0.0)
        self.declare_parameter("pulses_per_motor_rev", pulses_per_motor_rev)
        self.declare_parameter(
            "mechanical_reduction_ratio", mechanical_reduction_ratio
        )
        self.declare_parameter("profile_acceleration", profile_acceleration)
        self.declare_parameter("profile_deceleration", profile_deceleration)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        if not math.isfinite(self.command_timeout_sec) or self.command_timeout_sec < 0:
            raise ValueError("command_timeout_sec must be finite and >= 0")

        # 校验后的速度轮廓加减速参数保持 Pul/s^2 单位，用于写入 6083h/6084h。
        self.profile_acceleration = self._validate_profile_parameter(
            self.get_parameter("profile_acceleration").value,
            "profile_acceleration",
        )
        self.profile_deceleration = self._validate_profile_parameter(
            self.get_parameter("profile_deceleration").value,
            "profile_deceleration",
        )

        # 规范化并校验选定的 CANopen 节点 ID。
        if motor_ids is None:
            motor_ids = (1, 2, 3)
        try:
            self.motor_ids = tuple(int(motor_id) for motor_id in motor_ids)
        except (TypeError, ValueError):
            raise ValueError("motor_ids must be an iterable of CANopen node IDs")
        if (
            not self.motor_ids
            or len(set(self.motor_ids)) != len(self.motor_ids)
            or any(not 1 <= motor_id <= 0x7F for motor_id in self.motor_ids)
        ):
            raise ValueError("motor_ids must contain unique CANopen IDs in range 1..127")

        # 指令路径和反馈路径共用的单位换算参数。
        self.pulses_per_motor_rev = self._validate_positive_int_parameter(
            self.get_parameter("pulses_per_motor_rev").value,
            "pulses_per_motor_rev",
        )
        self.encoder_pulses_per_rev = self.pulses_per_motor_rev
        default_reduction_ratio = self._validate_positive_float_parameter(
            self.get_parameter("mechanical_reduction_ratio").value,
            "mechanical_reduction_ratio",
        )
        configured_reduction_ratios = (
            self.DEFAULT_MECHANICAL_REDUCTION_RATIOS
            if mechanical_reduction_ratios is None
            else mechanical_reduction_ratios
        )
        if not isinstance(configured_reduction_ratios, dict):
            raise ValueError("mechanical_reduction_ratios must be a dict keyed by motor ID")
        self.mechanical_reduction_ratios = {}
        for motor_id in self.motor_ids:
            parameter_name = f"mechanical_reduction_ratio_{motor_id}"
            self.declare_parameter(
                parameter_name,
                configured_reduction_ratios.get(motor_id, default_reduction_ratio),
            )
            self.mechanical_reduction_ratios[motor_id] = (
                self._validate_positive_float_parameter(
                    self.get_parameter(parameter_name).value,
                    parameter_name,
                )
            )

        # 每个电机的指令/反馈缓存。速度为输出轴 r/min，位置为弧度，
        # 转矩为设备按自身比例上报的换算值。
        self.motors = [
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
            for motor_id in self.motor_ids
        ]
        # 周期调度状态和停机保护标志。
        self._send_tick = 0
        self._feedback_tick = 0
        self.last_speed_command_time = time.monotonic()
        self._stopping = False

        # 差速底盘里程计状态：距离单位为 m，航向单位为弧度，
        # 慢速本地运动使用的 BASE_SPEED 单位为输出轴 r/min。
        self.wheel_radius = self.WHEEL_RADIUS
        self.wheel_base = 0.3
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.BASE_SPEED = (
            self.MAX_WHEEL_LINEAR_SPEED_MPS
            * 60.0
            / (2.0 * math.pi * self.wheel_radius)
            * 0.2
        )
        self.last_time = self.get_clock().now()

        if not self.create_can_bus():
            self.get_logger().warn("Failed to initialize CAN bus, will retry periodically")

        if self.auto_enable:
            self.initialize_motors()

        # ROS 接口：速度指令输入、周期反馈输出和里程计输出。
        self.subscription = self.create_subscription(
            Float32MultiArray, "motor_speed_commands",
            self.speed_command_callback, 10
        )
        self.velocity_publisher = self.create_publisher(
            Float32MultiArray, "motor_velocities", 10
        )
        self.motor_feedback_publisher = self.create_publisher(
            Float32MultiArray, "motor_feedback", 10
        )
        self.motor_fault_publisher = self.create_publisher(
            Float32MultiArray, "motor_fault_codes", 10
        )
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # CAN 总线和 ROS 接口创建完成后，再启动接收线程。
        self.receive_thread = None
        self.running = True
        self.start_receive_thread()
        self.get_logger().info(
            f"AIMOTOR CANopen driver started: channel={self.can_interface}, "
            f"bitrate={self.can_bitrate}, nodes={self.motor_ids}"
        )

    def _get_motor(self, motor_id: int):
        """返回指定已配置电机 ID 的缓存状态；不存在时返回 ``None``。"""
        for motor in self.motors:
            if motor["id"] == motor_id:
                return motor
        return None

    def _log_velocity_feedback(self, motor_id: int, pulses_per_sec: int) -> None:
        """同时以协议单位和输出轴单位记录目标速度与反馈速度。"""
        motor = self._get_motor(motor_id)
        reduction_ratio = self._get_mechanical_reduction_ratio(motor_id)
        if motor is None or reduction_ratio is None:
            return
        target_speed = float(motor["velocity"])
        target_pulses = int(round(
            target_speed * reduction_ratio / 60.0 * self.pulses_per_motor_rev
        ))
        self.get_logger().debug(
            f"[AIMotor] 电机{motor_id}速度："
            f"设定转速={target_speed:+.3f} r/min（输出轴），"
            f"实际转速={float(motor['actual_velocity']):+.3f} r/min（输出轴），"
            f"设置脉冲数={target_pulses} Pul/s，"
            f"实际反馈脉冲数={pulses_per_sec} Pul/s，"
            f"减速比={reduction_ratio:.3f}"
        )

    def _validate_motor_id(self, motor_id: int) -> bool:
        """检查 ``motor_id`` 是否为有效的 1..127 CANopen 节点 ID。"""
        if not isinstance(motor_id, int) or not 1 <= motor_id <= 0x7F:
            self.get_logger().error(f"Invalid CANopen motor ID: {motor_id}")
            return False
        return True

    @staticmethod
    def _validate_profile_parameter(value, name: str) -> int:
        """校验速度轮廓加速度/减速度是否为有效的正 INT32 数值。"""
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a finite positive integer")
        if (
            not math.isfinite(numeric_value)
            or numeric_value <= 0
            or not numeric_value.is_integer()
            or numeric_value > 0x7FFFFFFF
        ):
            raise ValueError(f"{name} must be a finite positive INT32 value")
        return int(numeric_value)

    @staticmethod
    def _validate_positive_float_parameter(value, name: str) -> float:
        """校验用于单位换算的标量是否为有限正数。"""
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a finite positive number")
        if not math.isfinite(numeric_value) or numeric_value <= 0:
            raise ValueError(f"{name} must be a finite positive number")
        return numeric_value

    @staticmethod
    def _validate_positive_int_parameter(value, name: str) -> int:
        """校验每转脉冲数是否为有限正整数。"""
        try:
            numeric_value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a finite positive integer")
        if (
            not math.isfinite(numeric_value)
            or numeric_value <= 0
            or not numeric_value.is_integer()
            or numeric_value > 0x7FFFFFFF
        ):
            raise ValueError(f"{name} must be a finite positive integer")
        return int(numeric_value)

    def _get_mechanical_reduction_ratio(self, motor_id: int) -> Optional[float]:
        """返回指定电机配置的独立机械减速比。"""
        if not self._validate_motor_id(motor_id):
            return None
        return self.mechanical_reduction_ratios.get(motor_id)

    def create_can_bus(self) -> bool:
        """按 AIMOTOR 默认的 500 kbit/s 配置创建 SocketCAN 总线。"""
        for attempt in range(1, 4):
            try:
                self.bus = can.Bus(
                    interface=self.can_bus_interface,
                    channel=self.can_interface,
                    bitrate=self.can_bitrate,
                )
                self.can_initialized = True
                self.get_logger().info(
                    f"CAN bus {self.can_interface} initialized at {self.can_bitrate} bit/s"
                )
                return True
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to initialize CAN bus (attempt {attempt}/3): {exc}"
                )
                if attempt < 3:
                    time.sleep(0.5)
        self.can_initialized = False
        return False

    def reconnect_can_bus(self):
        """发生错误后重置 SocketCAN 接口并重试创建总线。"""
        if self.can_initialized:
            return
        self.get_logger().info("Retrying CAN bus initialization...")
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception as exc:
                self.get_logger().warn(f"Error shutting down old CAN bus: {exc}")
            self.bus = None

        try:
            subprocess.run(
                ["ip", "link", "set", self.can_interface, "down"],
                capture_output=True, timeout=5.0
            )
            subprocess.run(
                ["ip", "link", "set", self.can_interface, "up", "type", "can",
                 "bitrate", str(self.can_bitrate)],
                capture_output=True, timeout=5.0
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to reset CAN interface via ip link: {exc}")
        time.sleep(0.1)
        self.create_can_bus()

    def send_can_frame(self, can_id: int, data: bytes) -> bool:
        """发送一帧 11 位标准 CAN 帧。"""
        if self.bus is None:
            self.get_logger().error("CAN bus not initialized")
            return False
        try:
            if not 0 <= can_id <= 0x7FF:
                raise ValueError(f"CANopen COB-ID out of range: 0x{can_id:X}")
            payload = bytes(data[:8]).ljust(8, b"\x00")
            message = can.Message(
                arbitration_id=can_id,
                data=payload,
                is_extended_id=False,
            )
            self.bus.send(message)
            time.sleep(0.01)
            return True
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send CAN frame (ID=0x{can_id:03X}): {exc}"
            )
            return False

    def _sdo_write(self, motor_id: int, index: int, subindex: int,
                   command: int, value: bytes = b"") -> bool:
        """向电机节点发送一条经过校验的快速传输 SDO 请求。"""
        if not self._validate_motor_id(motor_id):
            return False
        try:
            frame = build_sdo_frame(command, index, subindex, value)
        except ValueError as exc:
            self.get_logger().error(f"Invalid SDO write: {exc}")
            return False
        return self.send_can_frame(self.SDO_RX_BASE + motor_id, frame)

    def _sdo_read(self, motor_id: int, index: int, subindex: int) -> bool:
        """通过 SDO 上传请求读取对象字典中的一个值。"""
        return self._sdo_write(motor_id, index, subindex, self.SDO_READ)

    def _write_u16(self, motor_id: int, index: int, value: int) -> bool:
        """向对象字典写入一个无符号 16 位数值。"""
        return self._sdo_write(
            motor_id, index, 0, self.SDO_WRITE_2,
            int(value).to_bytes(2, "little", signed=False)
        )

    def _write_i32(self, motor_id: int, index: int, value: int) -> bool:
        """在范围校验后向对象字典写入一个有符号 32 位数值。"""
        if not -0x80000000 <= value <= 0x7FFFFFFF:
            self.get_logger().error(f"INT32 value out of range: {value}")
            return False
        return self._sdo_write(
            motor_id, index, 0, self.SDO_WRITE_4,
            int(value).to_bytes(4, "little", signed=True)
        )

    def send_nmt_command(self, command: int, motor_id: int = 0) -> bool:
        """发送 NMT 命令；节点 0 表示寻址所有节点。"""
        if not 0 <= command <= 0xFF or not 0 <= motor_id <= 0x7F:
            return False
        return self.send_can_frame(self.NMT_COB_ID, bytes((command, motor_id)))

    def motor_clear_fault(self, motor_id: int) -> bool:
        """使用控制字 0x0080 复位 CiA402 故障状态。"""
        return self._write_u16(motor_id, self.CONTROLWORD_INDEX, self.CONTROL_FAULT_RESET)

    def parse_motor_fault(self, can_id: int, data: bytes):
        """解析 EMCY 帧并记录匹配的 AIMOTOR 故障条目。

        前两个数据字节是小端格式的 CiA402 603Fh 故障码，第 3 个字节是
        标准错误寄存器。手册中的 H0B-34 故障码属于独立的厂家对象，
        因此对于重复的 603Fh 故障码只能记录候选项，不能从 EMCY 帧中猜测。
        """
        logger = self.get_logger()
        if not self.EMCY_BASE <= can_id < self.EMCY_BASE + 0x80:
            logger.warn(f"忽略非 EMCY 故障帧：0x{can_id:08X}")
            return

        motor_id = can_id - self.EMCY_BASE
        motor = self._get_motor(motor_id)
        if motor is None:
            logger.warn(f"忽略未知电机的故障帧：电机 ID={motor_id}")
            return
        if not data:
            logger.warn(f"忽略空故障帧：0x{can_id:08X}")
            return
        if len(data) < 3:
            logger.warn(
                f"忽略长度不足的故障帧：0x{can_id:08X}，长度={len(data)}"
            )
            return

        emergency_code = int.from_bytes(data[0:2], "little", signed=False)
        error_register = data[2]
        motor["fault_code"] = emergency_code
        matching_faults = tuple(
            fault for fault in self.AIMOTOR_FAULT_TABLE
            if fault[1] == emergency_code
        )

        logger.error("======================================")
        logger.error(f"电机 {motor_id} EMCY 故障帧：0x{can_id:08X}")
        logger.error(f"CiA402 标准故障码 603Fh：0x{emergency_code:04X}")
        logger.error(f"错误寄存器：0x{error_register:02X}")
        if len(data) > 3:
            logger.error(f"厂家特定数据：{bytes(data[3:]).hex(' ')}")

        if emergency_code == 0x0000:
            logger.info("无故障")
        elif not matching_faults:
            logger.error(
                f"手册 6.1 未定义标准故障码：0x{emergency_code:04X}"
            )
        elif len(matching_faults) == 1:
            manufacturer_code, _, description, led_pattern, alarm_type = (
                matching_faults[0]
            )
            logger.error(f"故障描述：{description}")
            logger.error(f"厂家故障码 H0B-34：0x{manufacturer_code:04X}")
            logger.error(
                f"硬件报警灯：{led_pattern}，报警类型：{alarm_type}"
            )
        else:
            logger.error(
                f"标准故障码 0x{emergency_code:04X} 对应多个厂家故障，"
                "无法仅凭 EMCY 帧唯一确定："
            )
            for (
                manufacturer_code,
                _,
                description,
                led_pattern,
                alarm_type,
            ) in matching_faults:
                logger.error(
                    f"- H0B-34=0x{manufacturer_code:04X}，{description}；"
                    f"硬件报警灯：{led_pattern}，报警类型：{alarm_type}"
                )
            logger.error(
                "请通过 SDO 读取 200B:23（H0B-34）确认具体厂家故障码"
            )

        logger.error("======================================")
        self.publish_motor_fault_codes()

    def publish_motor_fault_codes(self):
        """按电机配置顺序为每个电机发布一个 EMCY 故障码。"""
        message = Float32MultiArray()
        message.data = [float(motor["fault_code"]) for motor in self.motors]
        self.motor_fault_publisher.publish(message)

    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """设置 AIMOTOR 轮廓速度模式（6060h = 3）。

        旧版上层使用 ``2`` 表示速度模式，因此同时接受 2 和 CANopen
        标准值 3，但在线路上统一发送 3。
        """
        if mode not in (2, self.CANOPEN_VELOCITY_MODE):
            self.get_logger().error(f"Unsupported AIMOTOR mode: {mode}")
            return False
        return self._sdo_write(
            motor_id, self.MODES_OF_OPERATION_INDEX, 0,
            self.SDO_WRITE_1, bytes((self.CANOPEN_VELOCITY_MODE,))
        )

    def motor_set_acceleration(self, motor_id: int, acceleration=None) -> bool:
        """将 PV 轮廓加速度以 Pul/s^2 单位写入 6083h。"""
        if acceleration is None:
            acceleration = self.profile_acceleration
        try:
            acceleration = self._validate_profile_parameter(
                acceleration, "profile_acceleration"
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False
        return self._write_i32(
            motor_id, self.PROFILE_ACCELERATION_INDEX, acceleration
        )

    def motor_set_deceleration(self, motor_id: int, deceleration=None) -> bool:
        """将 PV 轮廓减速度以 Pul/s^2 单位写入 6084h。"""
        if deceleration is None:
            deceleration = self.profile_deceleration
        try:
            deceleration = self._validate_profile_parameter(
                deceleration, "profile_deceleration"
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False
        return self._write_i32(
            motor_id, self.PROFILE_DECELERATION_INDEX, deceleration
        )

    def motor_set_current_limit(self, motor_id: int, current_limit: float) -> bool:
        """保留旧版 API；AIMOTOR 手册未定义通用电流限制对象。"""
        self.get_logger().warn(
            f"AIMOTOR CANopen does not define a portable current-limit object; "
            f"ignoring motor {motor_id} value {current_limit}"
        )
        return False

    def motor_set_other_param(self, motor_id: int, param_value: float) -> bool:
        """保留旧版 API，但不再发送已废弃的 0x7022 帧。"""
        self.get_logger().warn(
            f"AIMOTOR CANopen does not define the legacy vendor parameter; "
            f"ignoring motor {motor_id} value {param_value}"
        )
        return False

    def motor_set_speed(self, motor_id: int, speed: float) -> bool:
        """将输出轴 r/min 速度转换为有符号 INT32 Pul/s 后写入 60FFh。"""
        reduction_ratio = self._get_mechanical_reduction_ratio(motor_id)
        if reduction_ratio is None:
            self.get_logger().error(
                f"No mechanical reduction ratio configured for motor {motor_id}"
            )
            return False
        try:
            numeric_speed = float(speed)
        except (TypeError, ValueError):
            self.get_logger().error(f"Invalid target velocity: {speed!r}")
            return False
        if not math.isfinite(numeric_speed):
            self.get_logger().error(f"Invalid target velocity: {speed!r}")
            return False
        target_pulses = int(round(
            numeric_speed * reduction_ratio / 60.0 * self.pulses_per_motor_rev
        ))
        motor = self._get_motor(motor_id)
        if motor is not None:
            # 直接 API 调用和 ROS 指令缓存写入共用同一个目标速度。
            motor["velocity"] = numeric_speed
        result = self._write_i32(motor_id, self.TARGET_VELOCITY_INDEX, target_pulses)
        self.get_logger().debug(
            f"[AIMotor] 电机{motor_id}设定："
            f"目标转速={numeric_speed:+.3f} r/min（输出轴），"
            f"设置脉冲数={target_pulses} Pul/s，"
            f"减速比={reduction_ratio:.3f}，"
            f"写入={'成功' if result else '失败'}"
        )
        return result

    def motor_query_feedback(self, motor_id: int) -> bool:
        """通过 SDO 请求状态、实际位置、实际速度和转矩。"""
        if not self._validate_motor_id(motor_id):
            return False
        results = [
            self._sdo_read(motor_id, self.STATUSWORD_INDEX, 0),
            self._sdo_read(motor_id, self.POSITION_ACTUAL_INDEX, 0),
            self._sdo_read(motor_id, self.VELOCITY_ACTUAL_INDEX, 0),
            self._sdo_read(motor_id, self.TORQUE_ACTUAL_INDEX, 0),
        ]
        return all(results)

    def motor_enable(self, motor_id: int) -> bool:
        """执行 CiA402 使能序列 06h -> 07h -> 0Fh。"""
        if not self._validate_motor_id(motor_id):
            return False
        results = []
        for controlword in (
            self.CONTROL_SHUTDOWN,
            self.CONTROL_SWITCH_ON,
            self.CONTROL_ENABLE_OPERATION,
        ):
            results.append(self._write_u16(motor_id, self.CONTROLWORD_INDEX, controlword))
        return all(results)

    def motor_disable(self, motor_id: int) -> bool:
        """停止运行，并将 CiA402 返回到“禁止上电”状态。"""
        return self._write_u16(
            motor_id, self.CONTROLWORD_INDEX, self.CONTROL_DISABLE_VOLTAGE
        )

    def initialize_motors(self):
        """启动节点，设置 PV 模式及加减速参数，然后逐个使能电机。"""
        self.get_logger().info("Initializing AIMOTOR CANopen nodes...")
        time.sleep(0.2)
        for motor in self.motors:
            motor_id = motor["id"]
            self.send_nmt_command(0x01, motor_id)  # 启动远程节点。
            time.sleep(0.01)
            self.motor_set_mode(motor_id, self.CANOPEN_VELOCITY_MODE)
            time.sleep(0.01)
            if not self.motor_set_acceleration(motor_id):
                self.get_logger().error(
                    f"Failed to set acceleration for AIMOTOR node {motor_id}"
                )
            time.sleep(0.01)
            if not self.motor_set_deceleration(motor_id):
                self.get_logger().error(
                    f"Failed to set deceleration for AIMOTOR node {motor_id}"
                )
            time.sleep(0.01)
            if not self.motor_enable(motor_id):
                self.get_logger().error(f"Failed to enable AIMOTOR node {motor_id}")
            time.sleep(0.01)

    def speed_command_callback(self, msg: Float32MultiArray):
        """缓存从 ROS2 接收的有限输出轴 r/min 速度指令。"""
        expected_length = len(self.motors)
        if len(msg.data) not in (3, expected_length):
            self.get_logger().warn(
                f"Received speed command with incorrect length: {len(msg.data)}, "
                f"expected {expected_length} or legacy 3"
            )
            return
        try:
            speeds = [float(value) for value in msg.data]
        except (TypeError, ValueError):
            self.get_logger().warn("Received non-numeric motor speed command")
            return
        if not all(math.isfinite(speed) for speed in speeds):
            self.get_logger().warn("Received non-finite motor speed command")
            return
        for motor in self.motors:
            motor["velocity"] = 0.0
        for index, speed in enumerate(speeds[:len(self.motors)]):
            self.motors[index]["velocity"] = speed
        self.last_speed_command_time = time.monotonic()

    def send_speed_commands(self):
        """写入缓存速度，并在连续失败后将节点标记为离线。"""
        send_error_threshold = 3
        retry_interval_ticks = 50
        self._send_tick += 1
        for motor in self.motors:
            if not motor["online"]:
                if self._send_tick % retry_interval_ticks != 0:
                    continue
                motor["velocity"] = 0.0
            result = self.motor_set_speed(motor["id"], motor["velocity"])
            if result:
                motor["send_errors"] = 0
                motor["online"] = True
            else:
                motor["send_errors"] += 1
                if motor["send_errors"] >= send_error_threshold:
                    motor["online"] = False
                    self.get_logger().error(
                        f"Motor {motor['id']} failed {send_error_threshold} speed writes; marked offline"
                    )

    def query_motor_feedback(self):
        """向每个电机请求状态、位置、速度和转矩反馈。"""
        for motor in self.motors:
            if not self.motor_query_feedback(motor["id"]):
                self.get_logger().warn(f"Failed to query motor {motor['id']} feedback")

    def _parse_sdo_feedback(self, motor_id: int, data: bytes):
        """将一条 SDO 响应解析到对应的电机缓存。"""
        if len(data) < 8 or data[0] in (self.SDO_ABORT,):
            return
        index = data[1] | (data[2] << 8)
        value = data[4:8]
        motor = self._get_motor(motor_id)
        if motor is None:
            return
        if index == self.STATUSWORD_INDEX:
            motor["status_word"] = int.from_bytes(value[:2], "little")
        elif index == self.MODES_OF_OPERATION_DISPLAY_INDEX:
            motor["mode_display"] = int.from_bytes(value[:1], "little", signed=True)
        elif index == self.POSITION_ACTUAL_INDEX:
            pulses = int.from_bytes(value, "little", signed=True)
            motor["actual_position"] = (
                pulses * 2.0 * math.pi / self.encoder_pulses_per_rev
            )
        elif index == self.VELOCITY_ACTUAL_INDEX:
            pulses_per_sec = int.from_bytes(value, "little", signed=True)
            reduction_ratio = self._get_mechanical_reduction_ratio(motor_id)
            if reduction_ratio is not None:
                motor["actual_velocity"] = (
                    pulses_per_sec
                    * 60.0
                    / (self.pulses_per_motor_rev * reduction_ratio)
                )
                self._log_velocity_feedback(motor_id, pulses_per_sec)
        elif index == self.TORQUE_ACTUAL_INDEX:
            torque_raw = int.from_bytes(value[:2], "little", signed=True)
            motor["actual_torque"] = torque_raw / 10.0

    def _parse_tpdo_feedback(self, can_id: int, data: bytes):
        """解析手册默认映射的状态/位置和速度/转矩 TPDO。"""
        node_id = can_id & 0x7F
        motor = self._get_motor(node_id)
        if motor is None:
            return
        if 0x180 <= can_id <= 0x1FF and len(data) >= 7:
            motor["status_word"] = int.from_bytes(data[0:2], "little")
            position = int.from_bytes(data[3:7], "little", signed=True)
            motor["actual_position"] = (
                position * 2.0 * math.pi / self.encoder_pulses_per_rev
            )
        elif 0x280 <= can_id <= 0x2FF and len(data) >= 4:
            velocity = int.from_bytes(data[0:4], "little", signed=True)
            reduction_ratio = self._get_mechanical_reduction_ratio(node_id)
            if reduction_ratio is not None:
                motor["actual_velocity"] = (
                    velocity
                    * 60.0
                    / (self.pulses_per_motor_rev * reduction_ratio)
                )
                self._log_velocity_feedback(node_id, velocity)
            if len(data) >= 6:
                motor["actual_torque"] = int.from_bytes(
                    data[4:6], "little", signed=True
                ) / 10.0

    def parse_motor_feedback(self, can_id: int, data: bytearray):
        """解析 SDO 响应及手册默认的 TPDO 映射。"""
        if self.SDO_TX_BASE <= can_id <= self.SDO_TX_BASE + 0x7F:
            self._parse_sdo_feedback(can_id - self.SDO_TX_BASE, bytes(data))
        elif 0x180 <= can_id <= 0x2FF:
            self._parse_tpdo_feedback(can_id, bytes(data))

    def uint16_to_float(self, x, x_min, x_max, bits):
        """将无符号整数范围线性映射到 ``[x_min, x_max]``。"""
        span = (1 << bits) - 1
        return (x_max - x_min) * x / span + x_min

    def update_odometry(self):
        """积分差速底盘反馈并发布里程计。"""
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time
        if dt <= 0 or len(self.motors) < 2:
            return
        left_vel = self.motors[0]["actual_velocity"]
        right_vel = self.motors[1]["actual_velocity"]
        rpm_to_mps = 2.0 * math.pi * self.wheel_radius / 60.0
        left_linear = left_vel * rpm_to_mps
        right_linear = right_vel * rpm_to_mps
        linear_velocity = (right_linear + left_linear) / 2.0
        angular_velocity = (right_linear - left_linear) / self.wheel_base
        self.x += linear_velocity * dt * math.cos(self.th)
        self.y += linear_velocity * dt * math.sin(self.th)
        self.th += angular_velocity * dt
        self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        """使用 SI 单位（m/s 和 rad/s）发布当前位姿和速度。"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.th)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        self.odom_publisher.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """将弧度制的 roll/pitch/yaw 转换为 ROS 四元数消息。"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        quaternion = Quaternion()
        quaternion.w = cr * cp * cy + sr * sp * sy
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy
        return quaternion

    def receive_can_frames(self):
        """接收 CAN 帧、分发协议消息，并在异常后恢复总线。"""
        self.get_logger().info("Starting AIMOTOR CANopen receive thread...")
        # 连续接收错误会触发传输层重置，避免短暂 CAN 故障使节点永久卡死。
        consecutive_errors = 0
        while self.running:
            try:
                if not self.can_initialized:
                    self.reconnect_can_bus()
                    time.sleep(1.0)
                    continue
                if self.bus is None:
                    time.sleep(0.1)
                    continue
                message = self.bus.recv(timeout=0.1)
                if message is None:
                    continue
                consecutive_errors = 0
                can_id = message.arbitration_id
                if self.EMCY_BASE <= can_id < self.EMCY_BASE + 0x80:
                    self.parse_motor_fault(can_id, message.data)
                elif self.SDO_TX_BASE <= can_id < self.SDO_TX_BASE + 0x80:
                    self.parse_motor_feedback(can_id, message.data)
                elif 0x180 <= can_id < 0x300:
                    self.parse_motor_feedback(can_id, message.data)
                elif 0x700 <= can_id < 0x780:
                    motor = self._get_motor(can_id - 0x700)
                    if motor is not None and message.data:
                        motor["online"] = message.data[0] in (0x05, 0x7F)
            except Exception as exc:
                consecutive_errors += 1
                self.get_logger().error(
                    f"CAN receive error ({consecutive_errors}/5): {exc}"
                )
                if consecutive_errors >= 5:
                    self.can_initialized = False
                    if self.bus is not None:
                        try:
                            self.bus.shutdown()
                        except Exception:
                            pass
                        self.bus = None
                    consecutive_errors = 0
                time.sleep(1.0)
        self.get_logger().info("Stopped CANopen receive thread")

    def start_receive_thread(self):
        """启动异步消费 CAN 反馈的守护线程。"""
        self.receive_thread = threading.Thread(
            target=self.receive_can_frames, daemon=True
        )
        self.receive_thread.start()

    def timer_callback(self):
        """执行 10 Hz 的指令发送、反馈查询、里程计和消息发布周期。"""
        if (
            self.command_timeout_sec > 0
            and time.monotonic() - self.last_speed_command_time > self.command_timeout_sec
        ):
            for motor in self.motors:
                motor["velocity"] = 0.0

        self.send_speed_commands()
        self._feedback_tick += 1
        if self._feedback_tick >= 5:
            self._feedback_tick = 0
            self.query_motor_feedback()
        self.update_odometry()

        velocity_msg = Float32MultiArray()
        velocity_msg.data = [float(motor["actual_velocity"]) for motor in self.motors]
        self.velocity_publisher.publish(velocity_msg)

        feedback_msg = Float32MultiArray()
        feedback_data = []
        for motor in self.motors:
            feedback_data.extend([
                float(motor["id"]),
                motor["actual_position"],
                motor["actual_velocity"],
                motor["actual_torque"],
                motor["actual_temperature"],
            ])
        feedback_msg.data = feedback_data
        self.motor_feedback_publisher.publish(feedback_msg)

    def stop_all_motors(self):
        """停止所有目标速度，然后使每个电机进入失能状态。"""
        if self._stopping:
            return
        self._stopping = True
        self.running = False
        for motor in self.motors:
            motor["velocity"] = 0.0
            self.motor_set_speed(motor["id"], 0.0)
            time.sleep(0.01)
        for motor in self.motors:
            self.motor_disable(motor["id"])
            time.sleep(0.01)

    def destroy_node(self):
        """停止电机、等待接收线程、关闭 CAN，然后销毁 ROS 节点。"""
        self.stop_all_motors()
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception as exc:
                self.get_logger().warn(f"CAN bus shutdown failed: {exc}")
            self.bus = None
        if rclpy.ok():
            super().destroy_node()


def main(args=None):
    """初始化 ROS2、运行驱动节点，并执行确定性的资源清理。"""
    rclpy.init(args=args)
    motor_driver = None
    try:
        motor_driver = CanMotorDriver()
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Unexpected error: {exc}")
    finally:
        if motor_driver is not None:
            motor_driver.stop_all_motors()
            if rclpy.ok():
                motor_driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
