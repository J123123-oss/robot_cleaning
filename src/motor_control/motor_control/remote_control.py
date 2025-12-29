#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import serial
import threading
import time

class SBUSRemoteController:
    """
    SBUS遥控器解析类（单类实现，方便后期调用）
    支持SBUS数据接收、16通道解析、按键状态判断、通道值归一化等功能
    """
    # SBUS常量定义
    SBUS_FRAME_LENGTH = 25  # SBUS帧长度
    SBUS_HEADER = 0x0F      # SBUS帧头
    SBUS_FOOTER = 0x00      # SBUS帧尾
    # 遥控器通道极值（对应原始SBUS数据范围）
    RC_CH_MIN = 282
    RC_CH_MID = 1002
    RC_CH_MAX = 1722
    RC_CH_HALF_RANGE = 720  # (RC_CH_MID - RC_CH_MIN) = (RC_CH_MAX - RC_CH_MID)

    def __init__(self, port="/dev/ttyUSB0", baudrate=100000, node=None):
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
        self.ch = [self.RC_CH_MID] * 16  # 16个通道原始值，初始化为中位
        self.is_connected = False        # 遥控器连接状态

        # 线程与锁（保证数据线程安全）
        self.parse_thread = None
        self.running = False
        self.data_lock = threading.Lock()

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
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO,
                timeout=0.1,
                write_timeout=0.5
            )
            self.serial_init_flag = True
            self._log_info(f"SBUS串口 {self.port} 初始化成功")
        except Exception as e:
            self.serial_init_flag = False
            self._log_error(f"SBUS串口初始化失败: {e}")

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

    def _parse_sbus_data(self):
        """SBUS数据解析核心逻辑（线程执行）"""
        sbus_buf = bytearray(self.SBUS_FRAME_LENGTH)
        while self.running:
            if not self.serial_dev or not self.serial_dev.is_open:
                time.sleep(0.1)
                continue

            try:
                # 读取完整SBUS帧
                read_len = self.serial_dev.readinto(sbus_buf)
                if read_len != self.SBUS_FRAME_LENGTH:
                    continue

                # 校验帧头帧尾
                if sbus_buf[0] != self.SBUS_HEADER or sbus_buf[-1] != self.SBUS_FOOTER:
                    continue

                # 解析16个通道数据（SBUS标准位解析逻辑，与原始32程序兼容）
                ch_data = [0] * 16
                ch_data[0] = ((sbus_buf[2] << 8) + sbus_buf[1]) & 0x07FF
                ch_data[1] = ((sbus_buf[3] << 5) + (sbus_buf[2] >> 3)) & 0x07FF
                ch_data[2] = ((sbus_buf[5] << 10) + (sbus_buf[4] << 2) + (sbus_buf[3] >> 6)) & 0x07FF
                ch_data[3] = ((sbus_buf[6] << 7) + (sbus_buf[5] >> 1)) & 0x07FF
                ch_data[4] = ((sbus_buf[7] << 4) + (sbus_buf[6] >> 4)) & 0x07FF
                ch_data[5] = ((sbus_buf[9] << 9) + (sbus_buf[8] << 1) + (sbus_buf[7] >> 7)) & 0x07FF
                ch_data[6] = ((sbus_buf[10] << 6) + (sbus_buf[9] >> 2)) & 0x07FF
                ch_data[7] = ((sbus_buf[11] << 3) + (sbus_buf[10] >> 5)) & 0x07FF
                ch_data[8] = ((sbus_buf[13] << 8) + sbus_buf[12]) & 0x07FF
                ch_data[9] = ((sbus_buf[14] << 5) + (sbus_buf[13] >> 3)) & 0x07FF
                ch_data[10] = ((sbus_buf[16] << 10) + (sbus_buf[15] << 2) + (sbus_buf[14] >> 6)) & 0x07FF
                ch_data[11] = ((sbus_buf[17] << 7) + (sbus_buf[16] >> 1)) & 0x07FF
                ch_data[12] = ((sbus_buf[18] << 4) + (sbus_buf[17] >> 4)) & 0x07FF
                ch_data[13] = ((sbus_buf[20] << 9) + (sbus_buf[19] << 1) + (sbus_buf[18] >> 7)) & 0x07FF
                ch_data[14] = ((sbus_buf[21] << 6) + (sbus_buf[20] >> 2)) & 0x07FF
                ch_data[15] = ((sbus_buf[22] << 3) + (sbus_buf[21] >> 5)) & 0x07FF

                # 线程安全更新通道数据
                with self.data_lock:
                    self.ch = ch_data.copy()
                    self.is_connected = True

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
            return self.RC_CH_MID

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
        normalized = (raw_val - self.RC_CH_MID) / self.RC_CH_HALF_RANGE
        return max(-1.0, min(1.0, normalized))

    def is_channel_min(self, ch_idx):
        """
        判断指定通道是否处于最小值
        :param ch_idx: 通道索引（0-15）
        :return: True/False
        """
        raw_val = self.get_channel_raw(ch_idx)
        return abs(raw_val - self.RC_CH_MIN) <= 10  # 允许10的误差

    def is_channel_mid(self, ch_idx):
        """
        判断指定通道是否处于中位值
        :param ch_idx: 通道索引（0-15）
        :return: True/False
        """
        raw_val = self.get_channel_raw(ch_idx)
        return abs(raw_val - self.RC_CH_MID) <= 10  # 允许10的误差

    def is_channel_max(self, ch_idx):
        """
        判断指定通道是否处于最大值
        :param ch_idx: 通道索引（0-15）
        :return: True/False
        """
        raw_val = self.get_channel_raw(ch_idx)
        return abs(raw_val - self.RC_CH_MAX) <= 10  # 允许10的误差

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
    sbus_remote = SBUSRemoteController(
        port="/dev/ttyUSB0",
        baudrate=100000,
        node=node
    )

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