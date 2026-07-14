#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头遥控器解析节点（串口文本协议）

协议格式：
  [HH:MM:SS.ms] [Web] MANUAL   — 启用 Web 遥控器独占控制
  [HH:MM:SS.ms] [Web] AUTO     — 退出 Web 遥控器，释放控制权
  [HH:MM:SS.ms] [Web] <angle>  — 方向角指令 (0-360°)
    0°=右转  90°=后退  180°=左转  270°=前进
    其余角度通过差速实现合成运动

速度方向约定（与 motor_control.py 一致）：
  前进: left=-v, right=+v    后退: left=+v, right=-v
  左转: left=+v, right=+v    右转: left=-v, right=-v
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Bool, Float32MultiArray
import serial
import threading
import time
import math
import re
import json


class CameraRemoteController(Node):
    """摄像头遥控器解析节点：串口读取 → 角度解析 → 差速转换 → ROS2 发布"""

    def __init__(self):
        super().__init__("camera_remote_control_node")

        # ── ROS2 参数声明 ──
        self.declare_parameter("port", "/dev/camera_remote")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("max_speed", 5.0)
        self.declare_parameter("publish_rate", 0.05)
        self.declare_parameter("angle_timeout", 20.0)
        self.declare_parameter("disable_dtr_rts", True)
        self.declare_parameter("esp32_boot_wait", 2.0)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.angle_timeout = self.get_parameter("angle_timeout").get_parameter_value().double_value
        self.disable_dtr_rts = self.get_parameter("disable_dtr_rts").get_parameter_value().bool_value
        self.esp32_boot_wait = self.get_parameter("esp32_boot_wait").get_parameter_value().double_value

        # ── 串口状态 ──
        self.serial_conn = None
        self.serial_init_flag = False
        self.is_connected = False

        # ── 控制模式状态（线程安全） ──
        self.lock = threading.Lock()
        self.web_control_active = False  # MANUAL→True, AUTO→False
        self._pending_control_active = None  # 待发布的状态变更（None=无变更，由定时器消费）
        self.current_angle = None        # 最新有效角度 (0-360)
        self.last_angle_time = 0.0       # 最新角度接收时间戳
        self.target_left_speed = 0.0     # 目标左轮速度
        self.target_right_speed = 0.0    # 目标右轮速度

        # ── 线程控制 ──
        self.running = False
        self.parse_thread = None
        self._stopped = False

        # ── 统计计数 ──
        self.frame_count = 0
        self.error_count = 0
        self.timeout_count = 0

        # ── 正则匹配 ──
        self.angle_pattern = re.compile(r'(\d+)')

        # ── ROS2 发布器 ──
        self.speed_pub = self.create_publisher(Vector3, "/web_rc/motor_speed", 10)
        self.control_active_pub = self.create_publisher(Bool, "/web_rc/control_active", 10)
        self.state_pub = self.create_publisher(String, "/web_rc/state", 10)

        # ── ROS2 订阅器 ──
        self.battery_sub = self.create_subscription(
            Float32MultiArray,
            "/battery_data",
            self._battery_callback,
            10
        )
        self.battery_percentage = None  # 最新电量百分比

        # ── 初始化串口 + 解析线程 ──
        # 延迟初始化：先让 ROS2 完成节点注册（所有 publisher/subscriber/timer），
        # 再连接串口，避免串口异常阻塞 __init__ 导致话题不可见。
        self._init_timer = self.create_timer(0.1, self._delayed_start)

        # ── 定时发布速度（匹配 motor_control 的 20Hz 控制频率） ──
        self.timer = self.create_timer(self.publish_rate, self._publish_speed)
        # 低速状态发布（1Hz）
        self.state_timer = self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"✅ 摄像头遥控器节点启动 | 串口:{self.port} | 波特率:{self.baudrate} | "
            f"最大速度:{self.max_speed} | 发布频率:{self.publish_rate}Hz | "
            f"超时:{self.angle_timeout}s | DTR/RTS禁用:{self.disable_dtr_rts} | "
            f"ESP32等待:{self.esp32_boot_wait}s"
        )

    # ==================== 串口管理 ====================

    def _open_serial(self):
        """打开串口；先固定 DTR/RTS，避免 ESP32 被 USB-UART 控制线复位。"""
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baudrate
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 0.005
        ser.write_timeout = 0.5
        ser.rtscts = False
        ser.dsrdtr = False

        if self.disable_dtr_rts:
            ser.dtr = False
            ser.rts = False

        ser.open()

        if self.disable_dtr_rts:
            ser.setDTR(False)
            ser.setRTS(False)

        if self.esp32_boot_wait > 0:
            self.get_logger().info(f"⏳ 串口已打开，等待 ESP32 应用稳定（{self.esp32_boot_wait:.1f}s）...")
            end_time = time.time() + self.esp32_boot_wait
            while time.time() < end_time:
                if self.parse_thread is not None and not self.running:
                    break
                time.sleep(max(0.0, min(0.1, end_time - time.time())))

        try:
            ser.reset_input_buffer()
        except serial.SerialException:
            pass

        return ser

    def _delayed_start(self):
        """延迟启动：ROS2 节点注册完成后再连接串口，避免 __init__ 阻塞"""
        self._init_timer.cancel()
        self.get_logger().info("⏳ 节点已注册，开始连接串口...")
        self._init_serial()
        self._start_parse_thread()

    def _init_serial(self):
        """初始化串口（非阻塞模式，与 remote_control.py 一致）"""
        try:
            self.serial_conn = self._open_serial()
            self.serial_init_flag = True
            self.is_connected = True
            self.get_logger().info(f"✅ 串口 {self.port} 初始化成功，波特率:{self.baudrate}")
        except Exception as e:
            self.serial_init_flag = False
            self.is_connected = False
            self.get_logger().error(f"❌ 串口初始化失败: {e}")

    def _reconnect_serial(self) -> bool:
        """串口自动重连（含 serial_init_flag 复位）"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = self._open_serial()
            self.is_connected = True
            self.serial_init_flag = True
            self.get_logger().warn(f"✅ 串口重连成功: {self.port}")
            return True
        except Exception as e:
            self.is_connected = False
            self.serial_init_flag = False
            self.get_logger().error(f"❌ 串口重连失败: {e}")
            return False

    # ==================== 解析线程 ====================

    def _start_parse_thread(self):
        """启动解析线程"""
        self.running = True
        self.parse_thread = threading.Thread(
            target=self._parse_serial_data,
            name="camera_remote_parse_thread",
            daemon=True
        )
        self.parse_thread.start()
        self.get_logger().info("✅ 串口数据解析线程已启动")

    def _parse_serial_data(self):
        """串口数据读取与解析（[Web] 帧分隔符，抗粘包/断包）"""
        last_data_time = time.time()
        buffer = ""

        while self.running:
            if not self.is_connected or not (self.serial_conn and self.serial_conn.is_open):
                if not self._reconnect_serial():
                    time.sleep(0.5)
                    continue

            try:
                ser = self.serial_conn
                if not ser or not ser.is_open:
                    self.is_connected = False
                    continue

                bytes_waiting = ser.in_waiting
                if bytes_waiting > 0:
                    raw_data = ser.read(bytes_waiting)
                    last_data_time = time.time()

                    # 过滤空闲噪声：剔除全零字节（Linux 浮空引脚噪声）
                    raw_data = raw_data.replace(b'\x00', b'')
                    if len(raw_data) == 0:
                        continue

                    # 打印原始串口数据
                    try:
                        raw_text = raw_data.decode('utf-8', errors='replace')
                    except Exception:
                        raw_text = raw_data.decode('latin-1', errors='replace')
                    self.get_logger().info(f"[Serial] raw({len(raw_data)}B): {raw_text.strip()!r}")

                    try:
                        text = raw_data.decode('utf-8', errors='ignore')
                    except Exception:
                        text = raw_data.decode('latin-1', errors='ignore')

                    buffer += text

                    # ── 帧解析：[Web] 作为每条消息的起始标记 ──
                    while True:
                        idx = buffer.find('[Web]')
                        if idx == -1:
                            # 无帧头，保留尾部（可能是不完整的帧头前缀）
                            if len(buffer) > 256:
                                buffer = buffer[-128:]
                            break

                        # 丢弃 [Web] 之前的垃圾数据
                        if idx > 0:
                            buffer = buffer[idx:]

                        # 查找消息结束位置：下一个 [Web] 或 \n
                        end_web = buffer.find('[Web]', 5)
                        end_nl = buffer.find('\n', 5)

                        if end_web == -1 and end_nl == -1:
                            # 消息不完整（还在接收中），等待更多数据
                            if len(buffer) > 512:
                                buffer = buffer[-256:]
                            break

                        if end_web == -1:
                            end_pos = end_nl
                        elif end_nl == -1:
                            end_pos = end_web
                        else:
                            end_pos = min(end_web, end_nl)

                        # 提取消息体（跳过 '[Web]' 前缀）
                        msg_body = buffer[5:end_pos].strip()
                        # 保留结束标记给下一轮
                        buffer = buffer[end_pos:]

                        if msg_body:
                            self._parse_message(msg_body)

                else:
                    if time.time() - last_data_time > 0.05:
                        time.sleep(0.002)

            except (serial.SerialException, OSError, TypeError) as e:
                if not self.running:
                    break
                self.error_count += 1
                self.get_logger().error(f"❌ 串口读取异常: {e}")
                self.is_connected = False
                buffer = ""
                time.sleep(0.1)
            except Exception as e:
                if not self.running:
                    break
                self.error_count += 1
                self.get_logger().error(f"❌ 解析线程异常: {e}")
                time.sleep(0.1)

    def _parse_message(self, msg_body: str):
        """解析 [Web] 后的消息体：MANUAL / AUTO / STOP / 角度数值"""
        upper = msg_body.upper().strip()

        if upper == 'MANUAL':
            self._set_control_active(True)
            return

        if upper == 'AUTO':
            self._set_control_active(False)
            return

        if upper == 'STOP':
            with self.lock:
                self.target_left_speed = 0.0
                self.target_right_speed = 0.0
                self.last_angle_time = time.time()  # 刷新时间戳，防止超时误告警
            return

        # 尝试提取角度数值
        match = self.angle_pattern.search(msg_body)
        if not match:
            return

        try:
            angle = int(match.group(1))
        except (ValueError, IndexError):
            return

        if angle < 0 or angle > 360:
            return

        if angle == 360:
            angle = 0

        now = time.time()
        with self.lock:
            self.current_angle = angle
            self.last_angle_time = now
            self.frame_count += 1

            left, right = self._angle_to_speeds(angle)
            self.target_left_speed = left
            self.target_right_speed = right

    def _set_control_active(self, active: bool):
        """切换 Web 遥控器控制权状态（仅设标志，由定时器主线程发布）"""
        with self.lock:
            changed = (self.web_control_active != active)
            self.web_control_active = active
            if not active:
                self.target_left_speed = 0.0
                self.target_right_speed = 0.0
            if changed:
                self._pending_control_active = active

        if changed:
            self.get_logger().info(
                f"{'🟢' if active else '🔴'} Web遥控器控制权: "
                f"{'MANUAL（独占控制）' if active else 'AUTO（释放控制）'}"
            )

    # ==================== 角度 → 速度映射 ====================

    def _angle_to_speeds(self, angle_deg: int):
        """
        将 0-360° 方向角转换为左右轮目标速度。

        数学模型（差速底盘运动学）：
          0°=右转  → left=-v, right=-v
          90°=后退 → left=+v, right=-v
         180°=左转  → left=+v, right=+v
         270°=前进 → left=-v, right=+v

        通用公式（推导验证）：
          left  = (sin θ - cos θ) × max_speed
          right = (-sin θ - cos θ) × max_speed

        中间角度（如 45°）自动产生合成运动（如右后方向）。
        缩放保护：任一电机速度超出 max_speed 时等比例缩放，保持方向比例不变。
        """
        rad = math.radians(angle_deg)
        sin_val = math.sin(rad)
        cos_val = math.cos(rad) *0.5

        raw_left = (sin_val - cos_val) * self.max_speed
        raw_right = (-sin_val - cos_val) * self.max_speed

        max_mag = max(abs(raw_left), abs(raw_right))
        if max_mag > self.max_speed and max_mag > 0.001:
            scale = self.max_speed / max_mag
            raw_left *= scale
            raw_right *= scale

        return raw_left, raw_right

    # ==================== ROS2 定时发布 ====================

    def _publish_speed(self):
        """定时发布电机速度（20Hz）"""
        with self.lock:
            active = self.web_control_active
            left = self.target_left_speed
            right = self.target_right_speed
            last_time = self.last_angle_time

        # 非 MANUAL 模式 → 输出零速
        if not active:
            left = 0.0
            right = 0.0
        else:
            now = time.time()
            # MANUAL 模式下超时保护：超过阈值未收到新角度则输出零速
            if last_time > 0 and (now - last_time) > self.angle_timeout:
                if left != 0.0 or right != 0.0:
                    self.timeout_count += 1
                    if self.timeout_count % 20 == 1:
                        self.get_logger().warn(
                            f"⚠️ 遥控器角度数据超时 ({now - last_time:.1f}s > {self.angle_timeout}s)，"
                            f"输出零速"
                        )
                left = 0.0
                right = 0.0
            else:
                self.timeout_count = 0

        msg = Vector3()
        msg.x = float(left)
        msg.y = float(right)
        msg.z = 0.0
        if rclpy.ok():
            try:
                self.speed_pub.publish(msg)
            except Exception:
                pass

        # 消费待发布的状态变更（由解析线程设置，主线程发布，避免跨线程 publish 丢失）
        with self.lock:
            pending = self._pending_control_active
            self._pending_control_active = None
        if pending is not None:
            ctrl_msg = Bool()
            ctrl_msg.data = pending
            if rclpy.ok():
                try:
                    self.control_active_pub.publish(ctrl_msg)
                except Exception:
                    pass

    def _publish_state(self):
        """1Hz 发布遥控器状态（角度、连接、控制权、统计）"""
        with self.lock:
            active = self.web_control_active
            angle = self.current_angle
            frame_count = self.frame_count
            last_time = self.last_angle_time
            left = self.target_left_speed
            right = self.target_right_speed

        now = time.time()
        age = (now - last_time) if last_time > 0 else -1.0

        state = {
            "web_control_active": active,
            "angle": angle,
            "angle_age_s": round(age, 2),
            "left_speed": round(left, 2),
            "right_speed": round(right, 2),
            "is_connected": self.is_connected,
            "frame_count": frame_count,
            "error_count": self.error_count,
        }

        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False)
        if rclpy.ok():
            try:
                self.state_pub.publish(msg)
            except Exception:
                pass
        # self._serial_write(f"电量：1%\n\r")
    # ==================== 电池数据订阅 ====================

    def _battery_callback(self, msg: Float32MultiArray):
        """订阅 /battery_data，提取电量百分比并通过串口发送"""
        if len(msg.data) < 1:
            return
        percentage = msg.data[0]
        self.battery_percentage = percentage

        # 通过串口发送电量信息（线程安全）
        self._serial_write(f"电量：{percentage:.0f}%\n\r")

    def _serial_write(self, text: str):
        """线程安全串口写入"""
        # with self.lock:
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(text.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"❌ 串口写入失败: {e}")

    # ==================== 生命周期 ====================

    def stop(self):
        """优雅停止，释放资源"""
        if self._stopped:
            return
        self._stopped = True
        self.running = False
        # 退出前释放控制权 + 发布零速
        self._set_control_active(False)
        zero_msg = Vector3()
        zero_msg.x = 0.0
        zero_msg.y = 0.0
        zero_msg.z = 0.0
        if rclpy.ok():
            try:
                self.speed_pub.publish(zero_msg)
            except Exception as e:
                self.get_logger().debug(f"跳过零速发布，ROS context 已不可用: {e}")
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join(timeout=1.0)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.get_logger().info("✅ 摄像头遥控器节点已停止，串口资源已释放")

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraRemoteController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⚠️ 收到中断信号")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
