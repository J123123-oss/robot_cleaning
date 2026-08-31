#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import can
import time
import math
from typing import Optional
import threading


def build_sdo_frame(command: int, index: int, subindex: int,
                    value: bytes = b'') -> bytes:
    """Build an eight-byte expedited SDO frame.

    The first four bytes are the command specifier, object index in
    little-endian order, and subindex.  Expedited object data occupies bytes
    4 through 7 and is also encoded little-endian by the caller.
    """
    if not 0 <= command <= 0xFF:
        raise ValueError("SDO command must fit in one byte")
    if not 0 <= index <= 0xFFFF:
        raise ValueError("SDO index must fit in two bytes")
    if not 0 <= subindex <= 0xFF:
        raise ValueError("SDO subindex must fit in one byte")
    if len(value) > 4:
        raise ValueError("expedited SDO value cannot exceed four bytes")

    frame = bytes((command, index & 0xFF, index >> 8, subindex)) + value
    return frame.ljust(8, b'\x00')


def decode_sdo_int32(data: bytes) -> int:
    """Decode the signed INT32 value in bytes 4 through 7 of an SDO frame."""
    if len(data) < 8:
        raise ValueError("SDO response must contain eight data bytes")
    return int.from_bytes(data[4:8], byteorder='little', signed=True)


class CanMotorDriver(Node):
    def __init__(self, node_name='can_motor_driver', channel='can0',
                 interface='socketcan', baudrate=1000000,
                 velocity_ratio=10000.0):

        super().__init__(node_name)

        
        # CAN配置
        self.can_interface = channel  # can0
        self.can_bus_interface = interface
        self.can_bitrate = baudrate
        self.bus: Optional[can.Bus] = None
        self.can_initialized = False
        # 上层速度值到电机脉冲数的比例；默认1个上层单位对应10000脉冲。
        self.velocity_ratio = float(velocity_ratio)
        if not math.isfinite(self.velocity_ratio) or self.velocity_ratio <= 0:
            raise ValueError("velocity_ratio must be greater than zero")
        
        # 电机运行状态。
        # 当前CANopen对象字典能够写入目标速度、读取实际速度和故障状态，
        # 因此这里只保存这些实际参与控制或能够从总线得到的字段。
        self.motors = [
            {
                "id": motor_id,                 # CANopen节点ID：1左轮、2右轮、3毛刷
                "velocity": 0.0,                # 目标速度，上层值会乘以velocity_ratio
                "actual_velocity": 0.0,         # 0x606C读取的实际速度，已换算为上层值
                "fault_code": 0,                # EMCY或0x2601读取到的故障状态
                "send_errors": 0,               # 连续发送失败次数
                "online": True                  # 最近一次速度命令是否发送成功
            }
            for motor_id in (1, 2, 3)
        ]
        self._send_tick = 0  # 发送周期计数，用于离线电机重试退避
        
        # CANopen标准帧和对象字典。
        # CANopen默认使用11位标准帧，下面的COB-ID基值需要加上电机节点ID。
        # 例如节点1的请求ID为0x601，响应ID为0x581，EMCY ID为0x081。
        self.SDO_RX_BASE = 0x600       # SDO下载请求：主站 -> 节点
        self.SDO_TX_BASE = 0x580       # SDO上传响应：节点 -> 主站
        self.EMCY_BASE = 0x80          # EMCY紧急错误：节点 -> 主站

        # CiA 402控制对象。SDO数据帧的字节1~2为索引低字节在前，字节3为子索引。
        self.CONTROLWORD_INDEX = 0x6040          # 控制字UINT16：0x000F使能，0x0006失能
        self.MODES_OF_OPERATION_INDEX = 0x6060   # 运行模式INT8：0x03为速度模式
        self.TARGET_VELOCITY_INDEX = 0x60FF      # 目标速度INT32：电机定义的脉冲数
        self.ACTUAL_VELOCITY_INDEX = 0x606C      # 实际速度INT32：电机定义的脉冲数
        self.POLARITY_INDEX = 0x607E             # 方向UINT8：0逆时针正转，1顺时针反转
        self.ACCELERATION_INDEX = 0x6083         # 加速度UINT32：DEC值，低字节在前
        self.DECELERATION_INDEX = 0x6084         # 减速度UINT32：DEC值，低字节在前
        self.ERROR_CODE_INDEX = 0x2601           # 厂家错误状态：读取2或4字节，按厂家协议解释

        # SDO命令字节（command specifier），位于数据帧Byte0。
        self.SDO_READ = 0x40                 # 上传读取请求：Byte4~7填0
        self.SDO_WRITE_1 = 0x2F              # 加速写入1字节：Byte4有效
        self.SDO_WRITE_2 = 0x2B              # 加速写入2字节：Byte4~5有效
        self.SDO_WRITE_4 = 0x23              # 加速写入4字节：Byte4~7有效
        self.SDO_WRITE_RESPONSE = 0x60       # 写入成功响应：索引、子索引和数据回显
        self.SDO_READ_2_RESPONSE = 0x4B     # 读取2字节成功：数据位于Byte4~5
        self.SDO_READ_4_RESPONSE = 0x43     # 读取4字节成功：数据位于Byte4~7
        self.SDO_ABORT = 0x80                # SDO中止响应：中止码位于Byte4~7
        self.CANOPEN_VELOCITY_MODE = 0x03   # CiA 402 Profile Velocity Mode
        
        # 机器人参数
        self.wheel_radius = 0.05  # 轮子半径（米）
        self.wheel_base = 0.3     # 轮距（米）
        self.encoder_resolution = 4096  # 编码器分辨率（每转脉冲数）
        
        # 里程计参数
        self.x = 0.0  # 机器人位置x坐标
        self.y = 0.0  # 机器人位置y坐标
        self.th = 0.0  # 机器人方向角度

        self.BASE_SPEED = 2.0 # 导航目标速度（dps）= self.BASE_SPEED*10
        # Sensor 
        self.front_left = None
        self.front_right = None
        self.mid_left = None
        self.mid_right = None
        self.back_left = None
        self.back_right = None

        
        # 上次时间戳
        self.last_time = self.get_clock().now()
        
        # 初始化CAN总线
        if not self.create_can_bus():
            self.get_logger().warn("Failed to initialize CAN bus, will retry periodically")
            
        # 初始化电机
        # self.initialize_motors()
        
        # 创建订阅者，用于接收速度命令
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speed_commands',
            self.speed_command_callback,
            10)
            
        # 创建发布者，用于发布电机速度
        self.velocity_publisher = self.create_publisher(
            Float32MultiArray, 
            'motor_velocities', 
            10)
            
        # 创建发布者，用于发布电机状态（位置、速度、扭矩等）
        self.motor_feedback_publisher = self.create_publisher(
            Float32MultiArray,
            'motor_feedback',
            10)
            
        # 创建发布者，用于发布三路电机故障码
        self.motor_fault_publisher = self.create_publisher(
            Float32MultiArray,
            'motor_fault_codes',
            10)
            
        # 创建发布者，用于发布里程信息
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10)
        
        # 定时器，定期发送速度命令和发布电机状态
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # 启动接收线程
        self.receive_thread = None
        self.running = True
        self.start_receive_thread()
        
        self.get_logger().info('Motor Control Node has been started')

    def create_can_bus(self) -> bool:
        """初始化CAN总线（最多重试3次，间隔0.5s）"""
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            try:
                self.bus = can.Bus(
                    interface=self.can_bus_interface,
                    channel=self.can_interface,
                    bitrate=self.can_bitrate,
                )
                self.can_initialized = True
                self.get_logger().info(f'CAN bus {self.can_interface} initialized successfully')
                return True
            except Exception as e:
                self.get_logger().error(
                    f'Failed to initialize CAN bus (attempt {attempt}/{max_retries}): {e}'
                )
                if attempt < max_retries:
                    time.sleep(0.5)

        self.can_initialized = False
        return False

    def reconnect_can_bus(self):
        """重试CAN总线初始化（含接口down/up复位，清除TX buffer和错误计数器）"""
        if self.can_initialized:
            return
        self.get_logger().info("Retrying CAN bus initialization...")

        # 1. 先关闭旧 bus（如果还存在）
        if self.bus is not None:
            try:
                self.bus.shutdown()
                self.get_logger().info("Old CAN bus shutdown before reconnect")
            except Exception as e:
                self.get_logger().warn(f"Error shutting down old CAN bus: {e}")
            finally:
                self.bus = None

        # 2. 复位 CAN 接口：清除内核 socketcan TX/RX buffer 和错误计数器
        try:
            import subprocess
            subprocess.run(
                ["ip", "link", "set", self.can_interface, "down"],
                capture_output=True, timeout=5.0
            )
            self.get_logger().info(f"CAN interface {self.can_interface} down for reset")
            subprocess.run(
                ["ip", "link", "set", self.can_interface, "up", "type", "can",
                 "bitrate", str(self.can_bitrate)],
                capture_output=True, timeout=5.0
            )
            self.get_logger().info(f"CAN interface {self.can_interface} up after reset")
        except Exception as e:
            self.get_logger().warn(f"Failed to reset CAN interface via ip link: {e}")
            # 继续尝试创建 bus，可能依然有效

        # 3. 等待硬件稳定
        time.sleep(0.1)

        # 4. 重建 CAN bus
        self.create_can_bus()

    def send_can_frame(self, can_id: int, data: bytes) -> bool:
        """发送CAN帧"""
        if not self.bus:
            self.get_logger().error("CAN bus not initialized")
            return False

        try:
            # CANopen使用标准11位帧，SDO数据固定为8字节。
            if len(data) < 8:
                data = data.ljust(8, b'\x00')
            elif len(data) > 8:
                data = data[:8]

            if not 0 <= can_id <= 0x7FF:
                raise ValueError(f"CANopen COB-ID out of range: 0x{can_id:X}")

            msg = can.Message(arbitration_id=can_id, data=data,
                              is_extended_id=False)
            self.bus.send(msg)
            time.sleep(0.01)
            return True
        except Exception as e:
            err_str = str(e)
            can_id_str = f"0x{can_id:03X}"
            self.get_logger().error(f"Failed to send CAN frame (ID={can_id_str}): {err_str}")
            # 不重置 can_initialized、不关闭 bus——单路电机断线不应影响其他电机
            return False

    def _get_motor(self, motor_id: int):
        """Return the configured motor with the requested CANopen node ID."""
        for motor in self.motors:
            if motor["id"] == motor_id:
                return motor
        return None

    def _validate_motor_id(self, motor_id: int) -> bool:
        """Check that a motor ID is a valid CANopen node ID."""
        if not isinstance(motor_id, int) or not 1 <= motor_id <= 0x7F:
            self.get_logger().error(f"Invalid CANopen motor ID: {motor_id}")
            return False
        return True

    def _sdo_write(self, motor_id: int, index: int, subindex: int,
                   command: int, value: bytes = b'') -> bool:
        """Send one expedited SDO request to a motor node.

        ``motor_id`` is appended to 0x600 to produce the standard SDO
        request COB-ID.  The motor's 0x580 response is handled asynchronously
        by ``parse_sdo_response`` in the receive thread.
        """
        if not self._validate_motor_id(motor_id):
            return False
        try:
            data = build_sdo_frame(command, index, subindex, value)
        except ValueError as exc:
            self.get_logger().error(f"Invalid SDO write: {exc}")
            return False
        return self.send_can_frame(self.SDO_RX_BASE + motor_id, data)

    def _sdo_read(self, motor_id: int, index: int, subindex: int) -> bool:
        """Send an SDO upload request for one object dictionary entry."""
        return self._sdo_write(motor_id, index, subindex, self.SDO_READ)

    def _encode_uint32(self, value, name: str) -> Optional[bytes]:
        """Encode a non-negative DEC value as four little-endian bytes."""
        try:
            numeric_value = int(round(float(value)))
        except (OverflowError, TypeError, ValueError):
            self.get_logger().error(f"Invalid {name}: {value!r}")
            return None
        if not 0 <= numeric_value <= 0xFFFFFFFF:
            self.get_logger().error(f"{name} out of uint32 range: {value!r}")
            return None
        return numeric_value.to_bytes(4, byteorder='little', signed=False)

    def _encode_int32(self, value, name: str) -> Optional[bytes]:
        """Encode a signed DEC value as four little-endian bytes."""
        try:
            numeric_value = int(round(float(value)))
        except (OverflowError, TypeError, ValueError):
            self.get_logger().error(f"Invalid {name}: {value!r}")
            return None
        if not -0x80000000 <= numeric_value <= 0x7FFFFFFF:
            self.get_logger().error(f"{name} out of int32 range: {value!r}")
            return None
        return numeric_value.to_bytes(4, byteorder='little', signed=True)

    def motor_clear_fault(self, motor_id: int) -> bool:
        """Request a CANopen fault reset with controlword value 0x0080."""
        value = (0x80).to_bytes(2, byteorder='little')
        return self._sdo_write(motor_id, self.CONTROLWORD_INDEX, 0,
                                self.SDO_WRITE_2, value)

    def parse_motor_fault(self, can_id: int, data: bytes):
        """Parse a CANopen EMCY frame with COB-ID 0x080 plus node ID.

        The first two data bytes are the little-endian emergency error code;
        byte 2 is the error register.  Both are saved for diagnostics, while
        ``fault_code`` keeps the existing ROS fault topic compatible.
        """
        # EMCY ID的低7位对应节点ID；数据帧Byte0~1为紧急错误码，
        # Byte2为错误寄存器，Byte3~7为厂家定义的错误状态附加数据。
        motor_id = can_id - self.EMCY_BASE
        motor = self._get_motor(motor_id)
        if motor is None or len(data) < 3:
            return

        emergency_code = int.from_bytes(data[0:2], byteorder='little')
        error_register = data[2]
        motor["fault_code"] = emergency_code

        self.get_logger().error(
            f"电机 {motor_id} EMCY故障：紧急错误码=0x{emergency_code:04X}, "
            f"错误寄存器=0x{error_register:02X}"
        )
        self.publish_motor_fault_codes()

    def publish_motor_fault_codes(self):
        fault_codes = [float(m["fault_code"]) for m in self.motors]
        fault_msg = Float32MultiArray()
        fault_msg.data = fault_codes
        self.motor_fault_publisher.publish(fault_msg)

    def motor_set_mode(self, motor_id: int, mode: int) -> bool:
        """Set object 0x6060 to CANopen Profile Velocity Mode (value 3)."""
        # 上层旧接口使用2表示速度模式；新电机要求写入CANopen值3。
        if mode not in (2, self.CANOPEN_VELOCITY_MODE):
            self.get_logger().error(f"Unsupported motor mode: {mode}")
            return False
        return self._sdo_write(
            motor_id, self.MODES_OF_OPERATION_INDEX, 0,
            self.SDO_WRITE_1, bytes((self.CANOPEN_VELOCITY_MODE,))
        )

    def motor_set_speed(self, motor_id: int, speed: float) -> bool:
        """Write target velocity 0x60FF as a signed pulse count.

        The existing upper-layer speed value is multiplied by
        ``velocity_ratio`` (default 10000) before it is encoded as INT32
        little-endian data bytes.
        """
        try:
            speed_value = float(speed)
        except (TypeError, ValueError):
            self.get_logger().error(f"Invalid target velocity: {speed!r}")
            return False
        if not math.isfinite(speed_value):
            self.get_logger().error(f"Invalid target velocity: {speed!r}")
            return False
        target_pulses = speed_value * self.velocity_ratio
        value = self._encode_int32(target_pulses, "target velocity")
        if value is None:
            return False
        return self._sdo_write(
            motor_id, self.TARGET_VELOCITY_INDEX, 0,
            self.SDO_WRITE_4, value
        )

    def motor_set_direction(self, motor_id: int, reverse: bool = False) -> bool:
        """Write polarity 0x607E: 0 is forward, 1 reverses rotation."""
        value = bytes((1 if reverse else 0,))
        return self._sdo_write(
            motor_id, self.POLARITY_INDEX, 0, self.SDO_WRITE_1, value
        )

    def motor_set_acceleration(self, motor_id: int, acceleration) -> bool:
        """Write the non-negative UINT32 acceleration DEC value to 0x6083."""
        value = self._encode_uint32(acceleration, "acceleration")
        if value is None:
            return False
        return self._sdo_write(
            motor_id, self.ACCELERATION_INDEX, 0, self.SDO_WRITE_4, value
        )

    def motor_set_deceleration(self, motor_id: int, deceleration) -> bool:
        """Write the non-negative UINT32 deceleration DEC value to 0x6084."""
        value = self._encode_uint32(deceleration, "deceleration")
        if value is None:
            return False
        return self._sdo_write(
            motor_id, self.DECELERATION_INDEX, 0, self.SDO_WRITE_4, value
        )

    def motor_query_feedback(self, motor_id: int) -> bool:
        """Read actual velocity 0x606C as a signed four-byte pulse value."""
        return self._sdo_read(motor_id, self.ACTUAL_VELOCITY_INDEX, 0)

    def motor_query_error_code(self, motor_id: int) -> bool:
        """Read manufacturer error status object 0x2601 from one motor."""
        return self._sdo_read(motor_id, self.ERROR_CODE_INDEX, 0)

    def motor_enable(self, motor_id: int) -> bool:
        """Enable one motor with controlword 0x000F."""
        value = (0x000F).to_bytes(2, byteorder='little')
        return self._sdo_write(
            motor_id, self.CONTROLWORD_INDEX, 0,
            self.SDO_WRITE_2, value
        )
        
    def motor_disable(self, motor_id: int) -> bool:
        """Disable one motor with controlword 0x0006."""
        value = (0x0006).to_bytes(2, byteorder='little')
        return self._sdo_write(
            motor_id, self.CONTROLWORD_INDEX, 0,
            self.SDO_WRITE_2, value
        )

    def initialize_motors(self):
        """初始化所有电机"""
        self.get_logger().info("Initializing motors...")
        time.sleep(3.0)  # 等待CAN接口就绪，与jifeng系统保持一致

        for motor in self.motors:
            motor_id = motor["id"]
            self.get_logger().info(
                f"Initializing CANopen motor (ID={motor_id})..."
            )
            self.motor_set_mode(motor_id, self.CANOPEN_VELOCITY_MODE)
            time.sleep(0.01)
            self.motor_enable(motor_id)
            time.sleep(0.01)

    def speed_command_callback(self, msg: Float32MultiArray):
        """处理速度命令回调函数"""
        if len(msg.data) != 3:  # 3个电机的速度命令
            self.get_logger().warn(f"Received speed command with incorrect length: {len(msg.data)}, expected: 3")
            return
            
        # 更新电机速度目标值
        for i in range(min(len(self.motors), len(msg.data))):
            self.motors[i]["velocity"] = float(msg.data[i])
        
        self.get_logger().debug(f"Updated motor velocity targets: {[m['velocity'] for m in self.motors]}")

    def send_speed_commands(self):
        """发送速度命令给所有在线电机，离线电机周期性重试"""
        SEND_ERROR_THRESHOLD = 3       # 连续失败 N 次标记离线
        RETRY_INTERVAL_TICKS = 50      # 离线电机每 N 个周期重试一次（10Hz → 5s）

        self._send_tick += 1

        for motor in self.motors:
            if not motor["online"]:
                # 离线电机：周期性重试，看是否恢复
                if self._send_tick % RETRY_INTERVAL_TICKS != 0:
                    continue
                # 重试前清零速度，避免积压速度指令
                motor["velocity"] = 0.0

            result = self.motor_set_speed(motor["id"], motor["velocity"])
            if result:
                if motor["send_errors"] > 0:
                    self.get_logger().info(f"电机 {motor['id']} 发送恢复，send_errors 清零")
                motor["send_errors"] = 0
                motor["online"] = True
            else:
                motor["send_errors"] += 1
                if motor["send_errors"] >= SEND_ERROR_THRESHOLD and motor["online"]:
                    motor["online"] = False
                    self.get_logger().error(
                        f"电机 {motor['id']} 连续 {SEND_ERROR_THRESHOLD} 次发送失败，标记离线"
                    )

    def query_motor_feedback(self):
        """主动查询所有电机的反馈数据"""
        for motor in self.motors:
            result = self.motor_query_feedback(motor["id"])
            if not result:
                self.get_logger().error(f"Failed to query motor {motor['id']} feedback")

    def parse_motor_feedback(self, can_id: int, data: bytes):
        """Parse the 0x606C SDO response used as actual motor velocity.

        This wrapper preserves the old method name used by the driver while
        delegating common SDO header decoding to ``parse_sdo_response``.
        """
        self.parse_sdo_response(can_id, data)

    def parse_sdo_response(self, can_id: int, data: bytes):
        """Parse CANopen SDO responses for speed, faults, and aborts.

        Bytes 1-3 identify the object dictionary entry.  A 0x43 response
        carries a four-byte value at bytes 4-7; a 0x4B response carries a
        two-byte value at bytes 4-5.  For error status 0x0001, the next two
        bytes are treated as the manufacturer's extended error code.
        """
        # SDO响应格式：Byte0命令字，Byte1~2索引（小端），Byte3子索引，
        # Byte4~7为对象数据或SDO中止码。
        motor_id = can_id - self.SDO_TX_BASE
        motor = self._get_motor(motor_id)
        if motor is None or len(data) < 8:
            return

        command = data[0]
        index = int.from_bytes(data[1:3], byteorder='little')
        subindex = data[3]

        if command == self.SDO_ABORT:
            # 中止响应的数据区是32位错误码，按小端格式读取。
            abort_code = int.from_bytes(data[4:8], byteorder='little')
            self.get_logger().error(
                f"电机 {motor_id} SDO读取/写入失败："
                f"index=0x{index:04X}, subindex=0x{subindex:02X}, "
                f"abort=0x{abort_code:08X}"
            )
            return

        if command == self.SDO_WRITE_RESPONSE:
            # 0x60表示写入成功；写入命令的回应不需要更新运行状态。
            return

        if index == self.ACTUAL_VELOCITY_INDEX and command == self.SDO_READ_4_RESPONSE:
            # 0x606C是有符号32位脉冲值，换算后保持上层原有速度接口单位。
            actual_pulses = decode_sdo_int32(data)
            motor["actual_velocity"] = actual_pulses / self.velocity_ratio
            motor["online"] = True
            return

        if index != self.ERROR_CODE_INDEX or command not in (
                self.SDO_READ_2_RESPONSE,
                self.SDO_READ_4_RESPONSE,
                self.SDO_WRITE_4):
            return

        if command == self.SDO_READ_2_RESPONSE:
            # 0x4B响应只使用Byte4~5作为16位错误状态值。
            error_status = int.from_bytes(data[4:6], byteorder='little')
        else:
            # 0x43或厂家资料列出的0x23响应使用Byte4~7作为32位状态值。
            error_status = int.from_bytes(data[4:8], byteorder='little')
            # 0x0001 indicates that the following two bytes are extended.
            if (error_status & 0xFFFF) == 0x0001:
                error_status = int.from_bytes(data[6:8], byteorder='little')

        motor["fault_code"] = error_status
        self.publish_motor_fault_codes()

    def update_odometry(self):
        """更新里程信息"""
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return

        # 获取左右轮的实际速度（驱动层已按velocity_ratio换算）
        left_vel = self.motors[0]["actual_velocity"]  # 左轮电机在索引0
        right_vel = self.motors[1]["actual_velocity"]  # 右轮电机在索引1

        # 将角速度转换为线速度（m/s）
        left_vel_linear = left_vel * self.wheel_radius
        right_vel_linear = right_vel * self.wheel_radius

        # 计算机器人线速度和角速度
        linear_velocity = (right_vel_linear + left_vel_linear) / 2.0
        angular_velocity = (right_vel_linear - left_vel_linear) / self.wheel_base

        # 计算位移和角度变化
        delta_x = linear_velocity * dt * 10  # 增加一个缩放因子来调整里程
        delta_y = 0.0
        delta_th = angular_velocity * dt

        # 更新机器人位置（基于差动驱动模型）
        self.x += delta_x * math.cos(self.th) - delta_y * math.sin(self.th)
        self.y += delta_x * math.sin(self.th) + delta_y * math.cos(self.th)
        self.th += delta_th

        # 发布里程计消息
        self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        """发布里程计消息"""
        current_time = self.get_clock().now()
        
        # 创建Odometry消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # 设置位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.th)

        # 设置速度
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        # 设置协方差矩阵（暂时设置为0，可以根据实际传感器精度调整）
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36

        # 发布里程计消息
        self.odom_publisher.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """从欧拉角创建四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q

    def receive_can_frames(self):
        """接收CAN帧的线程函数"""
        self.get_logger().info("Starting CAN frame receiving thread...")
        consecutive_errors = 0
        MAX_CONSECUTIVE_ERRORS = 5

        while self.running:
            try:
                if not self.can_initialized:
                    self.reconnect_can_bus()
                    consecutive_errors = 0
                    time.sleep(1.0)
                    continue

                if self.bus:
                    msg = self.bus.recv(timeout=0.1)
                    if msg is not None:
                        consecutive_errors = 0  # 成功收到帧，清零错误计数
                        can_id = msg.arbitration_id

                        # CANopen EMCY：0x80 + 节点ID。
                        if self.EMCY_BASE < can_id <= self.EMCY_BASE + 0x7F:
                            self.parse_motor_fault(can_id, msg.data)
                        # CANopen SDO响应：0x580 + 节点ID。
                        elif (
                            self.SDO_TX_BASE < can_id
                            <= self.SDO_TX_BASE + 0x7F
                        ):
                            self.parse_sdo_response(can_id, msg.data)
                else:
                    time.sleep(0.1)
            except Exception as e:
                consecutive_errors += 1
                err_str = str(e)
                self.get_logger().error(
                    f"CAN receive error (#{consecutive_errors}/{MAX_CONSECUTIVE_ERRORS}): {err_str}"
                )
                if consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    self.get_logger().error(
                        f"CAN receive: {consecutive_errors} consecutive errors, "
                        "forcing CAN bus reset (bus-off recovery)"
                    )
                    self.can_initialized = False
                    # 关闭旧 bus 以确保 reconnect 做完整的 down/up 复位
                    if self.bus is not None:
                        try:
                            self.bus.shutdown()
                        except Exception:
                            pass
                        finally:
                            self.bus = None
                    consecutive_errors = 0
                time.sleep(1.0)
        self.get_logger().info("Stopped CAN frame receiving thread")

    def start_receive_thread(self):
        """启动接收线程"""
        self.receive_thread = threading.Thread(target=self.receive_can_frames, daemon=True)
        self.receive_thread.start()

    def timer_callback(self):
        """定时器回调函数，发送速度命令并发布电机状态"""
        # 发送速度命令给所有电机
        self.send_speed_commands()

        # CANopen没有旧协议的周期反馈帧，需要主动读取0x606C。
        self.query_motor_feedback()
        
        # 更新里程信息
        self.update_odometry()
        
        # 发布电机速度
        velocity_msg = Float32MultiArray()
        velocity_msg.data = [float(m["actual_velocity"]) for m in self.motors]
        self.velocity_publisher.publish(velocity_msg)
        
        # 保留原反馈消息布局：[电机ID, 位置, 速度, 扭矩, 温度]。
        # 新CANopen指令未提供位置、扭矩、温度，因此这些字段发布为0.0。
        feedback_msg = Float32MultiArray()
        feedback_data = []
        for motor in self.motors:
            feedback_data.extend([
                float(motor["id"]),               # 电机ID
                0.0,                               # 当前协议未读取位置
                motor["actual_velocity"],         # 0x606C实际速度
                0.0,                               # 当前协议未读取扭矩
                0.0                                # 当前协议未读取温度
            ])
        feedback_msg.data = feedback_data
        self.motor_feedback_publisher.publish(feedback_msg)

    def destroy_node(self):
        """节点销毁时停止所有电机"""
        self.get_logger().info("Stopping all motors...")
        self.running = False  # 停止接收线程
        
        # 发送速度为0的命令
        for motor in self.motors:
            motor["velocity"] = 0.0
        self.send_speed_commands()
        time.sleep(0.01)
            
        # 发送关闭命令
        for motor in self.motors:
            self.motor_disable(motor["id"])
            time.sleep(0.01)
            
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
            
        # 关闭CAN总线
        if self.bus:
            self.bus.shutdown()
            self.get_logger().info("CAN bus shutdown")
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_driver = CanMotorDriver()
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if rclpy.ok():
            if 'motor_driver' in locals():
                motor_driver.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
