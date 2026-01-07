#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import serial
import threading
import time
import struct
from typing import Optional

FRAME_HEADER_SBUS = 0x0F                 # SBUS帧头
FRAME_TAIL1_SBUS = 0x00                  # SBUS帧尾1
FRAME_TAIL2_SBUS = 0x48                  # SBUS校验位
FRAME_LENGTH_SBUS = 33                   # SBUS帧长度（1+30+2）
CHANNEL_COUNT_SBUS = 15                  # 15通道
# 遥控器通道阈值（三挡开关专用）
RC_CH_MIN_VALUE = 282    # 低挡位
RC_CH_MID_VALUE = 1002   # 中挡位
RC_CH_MAX_VALUE = 1722   # 高挡位
RC_CH_HALF_RANGE = 720   # 半量程
DEAD_ZONE_SBUS = 20      # 摇杆死区
MAX_SPEED_REMOTE = 2.0   # 遥控器控制最大角速度（rad/s，可调整）


class SBUSRemoteController:
    """
    SBUS遥控器解析类（单类实现，方便后期调用）
    支持SBUS数据接收、16通道解析、按键状态判断、通道值归一化等功能
    """
    def __init__(self, port="/dev/ttyv1", baudrate=115200, node=None):
        """
        初始化SBUS遥控器
        :param port: SBUS串口设备路径
        :param baudrate: SBUS波特率（默认100000）
        :param node: ROS2节点实例（用于日志输出，可选）
        """
        # 串口配置
        self.port = port
        self.baudrate = baudrate
        self.serial_dev = None
        self.serial_init_flag = False

        # ROS2日志相关
        self.node = node
        self._logger = self.node.get_logger() if self.node is not None else None

        # 遥控器状态
        self.ch = [RC_CH_MID_VALUE] * 16  # 16个通道原始值，初始化为中位
        self.is_connected = False        # 遥控器连接状态

        # 线程与锁（保证数据线程安全）
        self.parse_thread = None
        self.running = False
        self.data_lock = threading.Lock()

        self.control_mode = "NORMAL"
        self.buffer = b""  # 帧缓存

        # 初始化串口并启动解析线程
        self._init_serial()
        self._start_parse_thread()

    def _init_serial(self):
        """初始化SBUS串口"""
        try:
            self.serial_dev = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.5
            )
            self.serial_init_flag = True
            self._log_info(f"SBUS串口 {self.port} 初始化成功")
        except Exception as e:
            self.serial_init_flag = False
            self._log_error(f"SBUS串口初始化失败: {e}")

    def _log_error(self, msg):
        """简单的日志输出（你可以替换为实际的日志系统）"""
        print(f"[ERROR] {msg}")
    
    def _reconnect_serial(self, is_initial: bool = False) -> bool:
        """
        串口重连逻辑（需要你根据实际串口配置实现）
        :param is_initial: 是否是初始化阶段
        :return: 重连成功返回True，失败返回False
        """
        # 这里需要你补充实际的串口重连代码
        # 示例逻辑：
        try:
            if self.serial_dev and self.serial_dev.is_open:
                self.serial_dev.close()
            
            # 重新打开串口（请替换为你的串口配置）
            self.serial_dev = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.5)
            self.is_connected = True
            return True
        except Exception as e:
            self._log_error(f"串口重连失败: {e}")
            self.is_connected = False
            return False

    def _start_parse_thread(self):
        """启动SBUS数据解析线程"""
        if not self.serial_init_flag:
            self._log_warn("串口未初始化，无法启动解析线程")
            return

        self.running = True
        self.parse_thread = threading.Thread(
            target=self._parse_sbus_data,
            name="SBUS_Parse_Thread",
            daemon=True
        )
        self.parse_thread.start()
        self._log_info("SBUS数据解析线程已启动")

    def _update_control_mode(self, ch6_value: int):
        """根据CH6三挡开关状态更新控制模式"""
        if abs(ch6_value - RC_CH_MIN_VALUE) < DEAD_ZONE_SBUS:
            self.control_mode = "REMOTE"
        elif abs(ch6_value - RC_CH_HALF_RANGE) < DEAD_ZONE_SBUS:
            self.control_mode = "NORMAL"
        else:
            self.control_mode = "RTK_NAV"

    def _parse_sbus_frame(self, frame: bytes) -> Optional[list]:
        """解析SBUS帧数据（复用参考代码逻辑）"""
        # 校验帧长度、帧头和尾标
        if len(frame) != FRAME_LENGTH_SBUS:
            return None
        if frame[0] != FRAME_HEADER_SBUS or frame[-2] != FRAME_TAIL1_SBUS:
            return None

        # 解析32字节数据为15个16bit通道值（小端模式）
        channels = []
        for i in range(15):  # 16个通道
            byte_start = 1 + 2 * i
            byte_end = byte_start + 2
            # 防止索引越界
            if byte_end > len(frame):
                return None
            # 小端模式解析2字节为16位无符号整数
            value = struct.unpack("<H", frame[byte_start:byte_end])[0]
            # 限制通道值在有效范围
            value = max(RC_CH_MIN_VALUE, min(RC_CH_MAX_VALUE, value))
            channels.append(value)
        return channels



    def _parse_sbus_data(self):
        """SBUS数据解析核心逻辑（线程执行）- 已更新为新解析逻辑"""
        while self.running:
            # 检测串口连接状态：断联则自动重连
            if not self.is_connected or not (self.serial_dev and self.serial_dev.is_open):
                self._log_error("串口断联，启动自动重连...")
                # 运行中重连（无限重试）
                if not self._reconnect_serial(is_initial=False):
                    self._log_error("运行中重连失败，停止读取循环")
                    break

            try:
                if not self.serial_dev or not self.serial_dev.is_open:
                    time.sleep(0.5)
                    continue

                # 读取批量数据（而非固定长度）
                data = self.serial_dev.read(1024)
                if not data:
                    time.sleep(0.001)
                    continue
                
                # 将新数据加入缓存
                self.buffer += data

                # 循环查找并解析完整帧
                while len(self.buffer) >= FRAME_LENGTH_SBUS:
                    # 查找帧头位置
                    header_idx = self.buffer.find(bytes([FRAME_HEADER_SBUS]))
                    if header_idx == -1:
                        # 未找到帧头，清空缓存
                        self.buffer = b""
                        break
                    
                    # 帧头位置+帧长度超过缓存长度，保留剩余数据继续等待
                    if header_idx + FRAME_LENGTH_SBUS > len(self.buffer):
                        self.buffer = self.buffer[header_idx:]
                        break
                    
                    # 提取完整帧并从缓存中移除
                    frame = self.buffer[header_idx:header_idx + FRAME_LENGTH_SBUS]
                    self.buffer = self.buffer[header_idx + FRAME_LENGTH_SBUS:]
                    
                    # 解析帧数据
                    channels = self._parse_sbus_frame(frame)
                    if channels:
                        # 线程安全更新通道数据
                        with self.data_lock:
                            self.ch = channels.copy()
                            self.is_connected = True
                        
                        # 解析CH6状态，切换控制模式（CH6对应索引5）
                        self._update_control_mode(self.ch[5])

            except Exception as e:
                self._log_error(f"SBUS数据解析异常: {e}")
                with self.data_lock:
                    self.is_connected = False
                time.sleep(0.1)

    def get_channel_raw(self, ch_idx):
        """
        获取指定通道的原始值
        :param ch_idx: 通道索引（0-15）
        :return: 通道原始值（282-1722），索引无效返回中位值
        """
        if not 0 <= ch_idx < 16:
            self._log_warn(f"通道索引 {ch_idx} 无效，范围0-15")
            return RC_CH_MID_VALUE

        with self.data_lock:
            return self.ch[ch_idx]

    def get_channel_normalized(self, ch_idx):
        """
        获取指定通道的归一化值（-1.0 ~ 1.0）
        :param ch_idx: 通道索引（0-15）
        :return: 归一化后的值
        """
        raw_val = self.get_channel_raw(ch_idx)
        # 计算归一化值，超出范围时截断
        normalized = (raw_val - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE
        return max(-1.0, min(1.0, normalized))

    def is_channel_min(self, ch_idx):
        """
        判断指定通道是否处于最小值
        :param ch_idx: 通道索引（0-15）
        :return: True/False
        """
        raw_val = self.get_channel_raw(ch_idx)
        return abs(raw_val - RC_CH_MIN_VALUE) <= 10  # 允许10的误差

    def is_channel_mid(self, ch_idx):
        """
        判断指定通道是否处于中位值
        :param ch_idx: 通道索引（0-15）
        :return: True/False
        """
        raw_val = self.get_channel_raw(ch_idx)
        return abs(raw_val - RC_CH_MID_VALUE) <= 10  # 允许10的误差

    def is_channel_max(self, ch_idx):
        """
        判断指定通道是否处于最大值
        :param ch_idx: 通道索引（0-15）
        :return: True/False
        """
        raw_val = self.get_channel_raw(ch_idx)
        return abs(raw_val - RC_CH_MAX_VALUE) <= 10  # 允许10的误差

    def get_remote_state(self):
        """
        获取完整遥控器状态
        :return: 字典，包含连接状态、所有通道原始值、归一化值
        """
        with self.data_lock:
            state = {
                "is_connected": self.is_connected,
                "channel_raw": self.ch.copy(),
                "channel_normalized": [self.get_channel_normalized(i) for i in range(16)]
            }
        return state

    def _log_info(self, msg):
        """日志输出-信息"""
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg):
        """日志输出-警告"""
        if self._logger:
            self._logger.warn(msg)

    def _log_error(self, msg):
        """日志输出-错误"""
        if self._logger:
            self._logger.error(msg)

    def stop(self):
        """停止解析线程并释放串口资源"""
        self.running = False
        if self.parse_thread and self.parse_thread.is_alive():
            self.parse_thread.join(timeout=1.0)

        if self.serial_dev and self.serial_dev.is_open:
            self.serial_dev.close()

        self._log_info("SBUS遥控器解析已停止，资源已释放")

# -------------------------- 测试调用示例（ROS2环境） --------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Node("sbus_remote_test_node")

    # 实例化SBUS遥控器（传入节点实例用于日志输出）
    sbus_remote = SBUSRemoteController(port="/dev/ttyv1", baudrate=115200, node=None)
    print("Start!")
    try:
        # 循环获取遥控器状态
        while rclpy.ok():
            remote_state = sbus_remote.get_remote_state()
            # 打印关键信息
            node.get_logger().info(
                f"连接状态: {remote_state['is_connected']} | "
                f"通道0原始值: {remote_state['channel_raw'][0]} | "
                f"通道0归一化值: {remote_state['channel_normalized'][0]:.2f} | "
                f"通道5是否最大: {sbus_remote.is_channel_max(5)}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，停止测试")
    finally:
        # 停止遥控器解析
        sbus_remote.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



            # # 读取串口数据
            # if self.rc_uart_dev is not None and self.rc_uart_dev.is_open:
            #     try:
            #         read_size = self.rc_uart_dev.readinto(rx_buf)
            #         rx_buf = list(rx_buf)  # 转换为列表方便操作
            #     except Exception as e:
            #         self.get_logger().error(f"串口读取失败: {e}")
            #         continue

            #     # 校验数据有效性（帧头0x0F，帧尾0x00，长度25）
            #     if read_size == rx_msg.size and read_size == 25:
            #         if rx_buf[0] == 0x0F and rx_buf[24] == 0x00:
            #             # 读取传感器状态（模拟32程序的中断保护）
            #             sensor = self.io_sensor_msg

            #             # 解析16个通道数据（完全对应32程序的位运算逻辑）
            #             self.rc_slave.ch[0] = ((rx_buf[2] << 8) + (rx_buf[1])) & 0x07FF
            #             self.rc_slave.ch[1] = ((rx_buf[3] << 5) + (rx_buf[2] >> 3)) & 0x07FF
            #             self.rc_slave.ch[2] = ((rx_buf[5] << 10) + (rx_buf[4] << 2) + (rx_buf[3] >> 6)) & 0x07FF
            #             self.rc_slave.ch[3] = ((rx_buf[6] << 7) + (rx_buf[5] >> 1)) & 0x07FF
            #             self.rc_slave.ch[4] = ((rx_buf[7] << 4) + (rx_buf[6] >> 4)) & 0x07FF
            #             self.rc_slave.ch[5] = ((rx_buf[9] << 9) + (rx_buf[8] << 1) + (rx_buf[7] >> 7)) & 0x07FF
            #             self.rc_slave.ch[6] = ((rx_buf[10] << 6) + (rx_buf[9] >> 2)) & 0x07FF
            #             self.rc_slave.ch[7] = ((rx_buf[11] << 3) + (rx_buf[10] >> 5)) & 0x07FF
            #             self.rc_slave.ch[8] = ((rx_buf[13] << 8) + (rx_buf[12])) & 0x07FF
            #             self.rc_slave.ch[9] = ((rx_buf[14] << 5) + (rx_buf[13] >> 3)) & 0x07FF
            #             self.rc_slave.ch[10] = ((rx_buf[16] << 10) + (rx_buf[15] << 2) + (rx_buf[14] >> 6)) & 0x07FF
            #             self.rc_slave.ch[11] = ((rx_buf[17] << 7) + (rx_buf[16] >> 1)) & 0x07FF
            #             self.rc_slave.ch[12] = ((rx_buf[18] << 4) + (rx_buf[17] >> 4)) & 0x07FF
            #             self.rc_slave.ch[13] = ((rx_buf[20] << 9) + (rx_buf[19] << 1) + (rx_buf[18] >> 7)) & 0x07FF
            #             self.rc_slave.ch[14] = ((rx_buf[21] << 6) + (rx_buf[20] >> 2)) & 0x07FF
            #             self.rc_slave.ch[15] = ((rx_buf[22] << 3) + (rx_buf[21] >> 5)) & 0x07FF