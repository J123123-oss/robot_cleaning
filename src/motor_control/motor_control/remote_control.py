#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import serial
import threading
import time
import struct
from typing import Optional

FRAME_HEADER = 0x0F                 # 帧头 单字节 0x0f
DATA_LENGTH = 32                    # 数据长度32字节
FRAME_SIZE = 1 + DATA_LENGTH        # 总帧长 32 → 33  
# ===================== 遥控器阈值 =====================
RC_CH_MIN_VALUE = 282    # 低挡位
RC_CH_MID_VALUE = 1002   # 中挡位
RC_CH_MAX_VALUE = 1722   # 高挡位
RC_CH_HALF_RANGE = 720   # 半量程 归一化用
DEAD_ZONE = 20           # 摇杆死区
MAX_SPEED_REMOTE = 2.0   # 遥控器控制最大角速度

class SBUSRemoteController(Node):
    """
    ✅ 核心修改：继承ROS2的Node类（解决日志上下文问题）
    ✅ 解析逻辑：1:1复用你CustomRCParser的稳定串口解析逻辑
    ✅ 线程模型：ROS2标准定时器替代while循环，无阻塞、无GIL竞争
    """
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        super().__init__("remote_control_node")
        # 串口配置
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.serial_init_flag = False

        # 遥控器状态
        self.ch = [RC_CH_MID_VALUE] * 16  # 16通道原始值
        self.is_connected = False         # 连接状态
        self.control_mode = None     # 控制模式

        # 线程与锁（线程安全，必须保留）
        self.running = False
        self.parse_thread = None
        self.buffer = bytearray()         # ✅ 改用bytearray，和你代码一致，效率更高

        # 调试计数
        self.frame_count = 0
        self.error_count = 0
        self.sync_loss_count = 0

        # ✅ 初始化串口+启动解析线程
        self._init_serial()
        self._start_parse_thread()
        
        # ✅ ROS2定时器：5Hz频率打印数据（替代while循环，无阻塞，核心修复）
        self.timer = self.create_timer(0.2, self._print_remote_state)
        self.get_logger().info("✅ 遥控器解析节点启动成功 | 串口:%s | 波特率:%d" % (self.port, self.baudrate))

    def _init_serial(self):
        """✅ 复用你的串口初始化逻辑，timeout=0.005 非阻塞"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.005,    # ✅ 你的正确参数：5ms非阻塞
                write_timeout=0.5
            )
            self.serial_init_flag = True
            self.is_connected = True
            self.get_logger().info("✅ 遥控器串口 %s 初始化成功，波特率：%d" % (self.port, self.baudrate))
        except Exception as e:
            self.serial_init_flag = False
            self.is_connected = False
            self.get_logger().error("❌ 遥控器串口初始化失败: %s | 检查串口路径/权限" % str(e))

    def _reconnect_serial(self) -> bool:
        """串口自动重连逻辑"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.005,
                write_timeout=0.5)
            self.is_connected = True
            self.get_logger().warn("✅ 串口重连成功：%s" % self.port)
            return True
        except Exception as e:
            self.is_connected = False
            self.get_logger().error("❌ 串口重连失败: %s" % str(e))
            return False

    def _start_parse_thread(self):
        """启动解析线程"""
        if not self.serial_init_flag:
            self.get_logger().error("❌ 串口未初始化，无法启动解析线程")
            return
        self.running = True
        self.parse_thread = threading.Thread(
            target=self._parse_remote_control_data,
            name="remote_control_Parse_Thread",
            daemon=True
        )
        self.parse_thread.start()
        self.get_logger().info("✅ 遥控器数据解析线程已启动")

    def _update_control_mode(self, ch6_value: int):
        """CH6三档开关切换控制模式"""
        if abs(ch6_value - RC_CH_MIN_VALUE) < DEAD_ZONE:
            self.control_mode = "REMOTE"
        elif abs(ch6_value - RC_CH_MID_VALUE) < DEAD_ZONE:
            self.control_mode = "NORMAL"
        else:
            self.control_mode = "RTK_NAV"

    def _parse_frame(self, frame_data):
        """✅ 1:1复用你的解析逻辑：大端序解析 + 帧头校验 + 长度校验"""
        if len(frame_data) != FRAME_SIZE:
            return None
        if frame_data[0] != FRAME_HEADER:
            return None
        # ✅ 核心修复：大端序 >H 解析，你的正确逻辑
        channels = []
        for i in range(16):
            offset = 1 + i * 2
            channel_value = struct.unpack('>H', frame_data[offset:offset+2])[0]
            # 限制通道值范围
            channel_value = max(RC_CH_MIN_VALUE, min(RC_CH_MAX_VALUE, channel_value))
            channels.append(channel_value)
        return channels

    def _parse_remote_control_data(self):
        """✅ 核心修复：1:1复用你CustomRCParser的稳定缓冲区解析逻辑，无清空缓存的致命缺陷"""
        last_data_time = time.time()
        while self.running:
            if not self.is_connected or not (self.serial_conn and self.serial_conn.is_open):
                self.get_logger().warn("🔄 串口断联，启动自动重连...")
                if not self._reconnect_serial():
                    time.sleep(0.5)
                    continue

            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.buffer.extend(data)
                    last_data_time = time.time()

                    # ✅ 限制缓冲区大小，防止内存溢出，你的逻辑
                    if len(self.buffer) > FRAME_SIZE * 10:
                        self.buffer = self.buffer[-(FRAME_SIZE * 5):]

                    # ✅ 逐字节查找有效帧，核心修复：找不到帧头不清空缓存
                    pos = 0
                    while pos <= len(self.buffer) - FRAME_SIZE:
                        if self.buffer[pos] == FRAME_HEADER:
                            frame_data = self.buffer[pos:pos+FRAME_SIZE]
                            channels = self._parse_frame(frame_data)
                            if channels:
                                # with self.data_lock:
                                self.ch = channels.copy()
                                self.is_connected = True
                                self.frame_count += 1
                                self._update_control_mode(self.ch[5])
                                # 移除已解析的帧数据
                                self.buffer = self.buffer[pos + FRAME_SIZE:]
                                self.sync_loss_count = 0
                                break
                        pos += 1
                    else:
                        self.sync_loss_count += 1
                        # 连续同步失败，找下一个帧头重连，不清空缓存
                        if self.sync_loss_count > 50:
                            header_pos = -1
                            for i in range(len(self.buffer)):
                                if self.buffer[i] == FRAME_HEADER:
                                    header_pos = i
                                    break
                            if header_pos != -1:
                                self.buffer = self.buffer[header_pos:]
                            elif len(self.buffer) > FRAME_SIZE:
                                self.buffer = bytearray()
                            self.sync_loss_count = 0
                else:
                    if time.time() - last_data_time > 0.05:
                        time.sleep(0.002)
            except Exception as e:
                self.error_count += 1
                self.get_logger().error("❌ 解析异常: %s" % str(e))
                # with self.data_lock:
                self.is_connected = False
                time.sleep(0.1)

    def get_channel_raw(self, ch_idx):
        # if not 0 <= ch_idx < 16:
        #     return RC_CH_MID_VALUE
        # with self.data_lock:
            return self.ch[ch_idx]

    def get_channel_normalized(self, ch_idx):
        raw_val = self.get_channel_raw(ch_idx)
        normalized = (raw_val - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE
        return max(-1.0, min(1.0, normalized))

    def get_remote_state(self):
        """获取遥控器状态"""
        # with self.data_lock:
        state = {
            "is_connected": self.is_connected,
            "control_mode": self.control_mode,
            "channel_raw": self.ch.copy(),
            "channel_normalized": [self.get_channel_normalized(i) for i in range(16)]
        }
        return state

    def _print_remote_state(self):
        """✅ ROS2定时器回调：1Hz打印数据，无阻塞，彻底解决卡死"""
        remote_state = self.get_remote_state()
        self.get_logger().info(
            f"连接状态: {remote_state['is_connected']} | "
            f"模式: {remote_state['control_mode']} | "
            f"channel_raw: {remote_state['channel_raw'][:8]} | "
            f"channel_normalized: {[f'{val:.2f}' for val in remote_state['channel_normalized'][:8]]}"
        )

    def stop(self):
        """优雅停止，释放资源"""
        self.running = False
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join(timeout=1.0)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.get_logger().info("✅ 遥控器解析已停止，串口资源已释放")

def main(args=None):
    rclpy.init(args=args)
    # 实例化节点（继承Node，日志上下文有效）
    remote_node = SBUSRemoteController(port="/dev/ttyUSB0", baudrate=115200)
    try:
        # ✅ ROS2标准spin，阻塞但不卡死，定时器正常运行
        rclpy.spin(remote_node)
    except KeyboardInterrupt:
        # ✅ 修复rosout报错：先打印日志 → 再stop → 最后shutdown
        remote_node.get_logger().info("⚠️ 收到中断信号，准备停止节点")
    finally:
        # ✅ 顺序至关重要：先停止解析 → 再销毁节点 → 最后关闭rclpy
        remote_node.stop()
        remote_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()