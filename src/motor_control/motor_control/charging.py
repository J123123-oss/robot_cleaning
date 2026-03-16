#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import Float32MultiArray, Int16
from std_srvs.srv import Trigger
from custom_msgs.srv import ChargeControl

# -------------------------- Modbus CRC16 校验函数（严格匹配文档公式） --------------------------
def crc16_modbus(data: bytes) -> bytes:
    wcrc = 0xFFFF
    for byte in data:
        temp = byte & 0x00FF
        wcrc ^= temp
        for _ in range(8):
            if wcrc & 0x0001:
                wcrc >>= 1
                wcrc ^= 0xA001
            else:
                wcrc >>= 1
    # 低位在前，高位在后（文档要求）
    crc_l = wcrc & 0xFF
    crc_h = (wcrc >> 8) & 0xFF
    return bytes([crc_l, crc_h])

# -------------------------- 接收端故障码映射表 --------------------------
FAULT_CODE_MAP = {
    0x00: "无故障",  # 修复 KeyError:0
    0x01: "预留",
    0x02: "恒流区输出过流保护",
    0x03: "恒流区输出欠流保护",
    0x04: "VRECT过压硬件保护（全程）",
    0x05: "VRECT欠压软件保护",
    0x06: "输出过压软件保护",
    0x07: "电池异常保护，电池电压不在正常区间",
    0x08: "接收端过温保护",
    0x09: "接收端满充保护",
    0x0A: "接收端零距离启动软件保护(开继电器之前)",
    0x0B: "接收端远距离启动软件保护(开继电器之前)",
    0x0C: "预留",
    0x0D: "接收端过温不启动保护",
    0x0E: "输出过压硬件保护",
    0x0F: "接收端短路保护",
    0x10: "接收端VRECT稳压失败保护",
    0x11: "事中FOD保护",
    0x12: "接收端自动配对失败",
    0x13: "接收端远距离启动软件保护",
    0x14: "拨号无响应保护",
    0x15: "拨号启动失败保护",
    0x16: "预充区输出过流保护",
    0x17: "预充区输出欠流保护",
    0x18: "VRECT欠压硬件保护(开继电器之前)",
    0x19: "接收端欠温保护",
    0x1A: "抛负载保护",
    0x1B: "电池故障保护（BMS）",
    0x1C: "电池满充保护（BMS）",
    0x1D: "空载启动保护",
    0x1E: "预留",
    0x1F: "VRECT过压硬件保护(开继电器之后)",
    0x20: "VRECT欠压硬件保护(开继电器之后)",
    0x21: "接收端远距离启动软件保护(开继电器之前)",
    0x22: "接收端欠温不启动",
    0x23: "进恒压区保护",
    0x24: "模拟pin配地址失败保护",
    0x25: "Vrect过压软件保护（全程）",
    0x26: "接收端模拟PIN未收到地址保护",
}

# -------------------------- 充电控制节点类（三字节地址模式专属） --------------------------
class Charging485Node(Node):
    def __init__(self):
        super().__init__('charging_485_node')
        # 1. 声明并获取串口参数（支持Launch配置）
        self.declare_parameter('serial_port', '/dev/charging')
        self.declare_parameter('baud_rate', 19200)  # 文档默认波特率
        self.declare_parameter('slave_addr', 0x01)  # 从机地址出厂默认01
        self.declare_parameter('timeout', 0.5)      # 串口超时时间
        # 三字节地址模式-发射端固定地址（指令要求：addr=[0x10,0x04,0x5D]）
        self.THREE_BYTE_ADDR = [0x10, 0x04, 0x5D]

        # 解析参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.slave_addr = self.get_parameter('slave_addr').value
        self.timeout = self.get_parameter('timeout').value

        # 2. 初始化串口+线程锁（保证串口操作原子性）
        self.ser = None
        self.mutex = threading.Lock()
        self._init_serial()

        # 3. ROS2 话题发布（电压电流/故障码）
        self.volt_curr_pub = self.create_publisher(Float32MultiArray, 'charging_volt_curr', 10)
        self.fault_code_pub = self.create_publisher(Int16, 'charging_fault_code', 10)

        # 4. ROS2 服务创建（启停充电/查询数据）
        self.srv_start_charge = self.create_service(ChargeControl, 'start_charging', self._start_charge_cb)
        self.srv_stop_charge = self.create_service(ChargeControl, 'stop_charging', self._stop_charge_cb)
        self.srv_query_volt_curr = self.create_service(Trigger, 'query_volt_curr', self._query_volt_curr_cb)
        self.srv_query_fault = self.create_service(Trigger, 'query_fault_code', self._query_fault_code_cb)

        # 5. 定时器：1s定时查询（避免串口拥堵，匹配485通信速率）
        self.timer = self.create_timer(10.0, self._timer_query_data)
        self.get_logger().info(
            f"充电485节点启动成功【三字节地址模式】\n"
            f"串口：{self.serial_port} | 从机地址：0x{self.slave_addr:02X} | 发射端地址：{[hex(x) for x in self.THREE_BYTE_ADDR]}"
        )

    def _init_serial(self):
        """初始化485串口（严格8N1，匹配文档协议）"""
        try:
            with self.mutex:
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )
                if not self.ser.is_open:
                    self.ser.open()
        except serial.SerialException as e:
            self.get_logger().fatal(f"串口初始化失败：{str(e)}")
            rclpy.shutdown()

    def _send_cmd(self, cmd: list) -> bytes:
        """发送指令：拼接CRC + 清空缓冲区 + 串口收发（线程安全）"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("串口未打开，指令发送失败")
            return b''
        try:
            # 拼接指令字节+自动计算CRC
            cmd_bytes = bytes(cmd)
            crc_bytes = crc16_modbus(cmd_bytes)
            send_bytes = cmd_bytes + crc_bytes
            # 串口操作（加锁）
            with self.mutex:
                self.ser.flushInput()  # 清空接收缓冲区，避免脏数据
                self.ser.write(send_bytes)
                resp = self.ser.read(16)  # 最大响应长度16字节，匹配文档
            # 调试日志
            self.get_logger().debug(f"发送指令：{[hex(b) for b in send_bytes]}")
            if resp:
                self.get_logger().debug(f"接收响应：{[hex(b) for b in resp]}")
            return resp
        except Exception as e:
            self.get_logger().error(f"指令发送异常：{str(e)}")
            return b''

    def _parse_volt_curr(self, resp: bytes) -> tuple[float, float]:
        """解析电压电流（文档协议：01 03 04 [电压高] [电压低] [电流高] [电流低] CRC）"""
        volt, curr = 0.0, 0.0
        # 校验响应格式：长度≥8 + 从机地址匹配 + 功能码03 + 数据长度04
        if len(resp) >= 8 and resp[0] == self.slave_addr and resp[1] == 0x03 and resp[2] == 0x04:
            # 校验CRC
            if crc16_modbus(resp[:-2]) == resp[-2:]:
                # 解析16位寄存器值，×0.01为实际值（文档默认倍率）
                volt_raw = (resp[3] << 8) | resp[4]
                curr_raw = (resp[5] << 8) | resp[6]
                volt = volt_raw * 0.01
                curr = curr_raw * 0.01
                # self.get_logger().info(f"电压电流解析成功 | {volt:.1f}V | {curr:.2f}A")
            else:
                self.get_logger().warn("电压电流响应-CRC校验失败")
        else:
            self.get_logger().warn("电压电流响应-格式错误")
        return volt, curr

    def _parse_fault_code(self, resp: bytes) -> tuple[int, str]:
        """解析故障码（寄存器0x05，2字节数据）"""
        fault_code = 0x0000
        fault_msg = FAULT_CODE_MAP[0x0000]
        # 校验响应格式：长度≥7 + 从机地址匹配 + 功能码03 + 数据长度02
        if len(resp) >= 7 and resp[0] == self.slave_addr and resp[1] == 0x03 and resp[2] == 0x02:
            if crc16_modbus(resp[:-2]) == resp[-2:]:
                fault_code = (resp[3] << 8) | resp[4]
                fault_msg = FAULT_CODE_MAP.get(fault_code, f"未定义故障码: {hex(fault_code)}")
                # self.get_logger().info(f"故障码解析成功 | 0x{fault_code:04X} | {fault_msg}")
            else:
                self.get_logger().warn("故障码响应-CRC校验失败")
        else:
            self.get_logger().warn("故障码响应-格式错误")
        return fault_code, fault_msg

    def _timer_query_data(self):
        """定时器自动查询：电压电流+故障码，并发布话题"""
        # 1. 查询电压电流（文档指令：01 03 00 01 00 02 + CRC）
        cmd_volt_curr = [self.slave_addr, 0x03, 0x00, 0x01, 0x00, 0x02]
        volt, curr = self._parse_volt_curr(self._send_cmd(cmd_volt_curr))
        # 发布电压电流
        msg_volt = Float32MultiArray()
        msg_volt.data = [volt, curr]
        self.volt_curr_pub.publish(msg_volt)

        # 2. 查询故障码（指令：01 03 00 05 00 01 + CRC）
        cmd_fault = [self.slave_addr, 0x03, 0x00, 0x05, 0x00, 0x01]
        fault_code, _ = self._parse_fault_code(self._send_cmd(cmd_fault))
        # 发布故障码
        msg_fault = Int16()
        msg_fault.data = fault_code
        self.fault_code_pub.publish(msg_fault)

        # 3. 充满自动停止（锂电池48V体系：电压≥54.0V 且 电流≤0.5A，匹配实际充电逻辑）
        if volt >= 54.0 and curr <= 0.5:
            self.get_logger().info(f"检测到充电完成 | {volt:.1f}V/{curr:.2f}A，自动停止充电")
            fake_req = ChargeControl.Request()
            fake_res = ChargeControl.Response()
            self._stop_charge_cb(fake_req, fake_res)

    # -------------------------- 三字节地址模式-开始充电（核心适配） --------------------------
    def _start_charge_cb(self, req, res):
        """
        三字节地址开始充电指令格式（文档严格定义）：
        [从机地址, 0x10, 0x30, 0x01, 0x00, 0x02, 0x04, 发射端地址1, 发射端地址2, 发射端地址3, 0xB1]
        对应：01 10 30 01 00 02 04 10 04 5D B1 + 自动CRC
        """
        try:
            # 构造三字节地址模式-开始充电核心指令
            cmd_start = [
                self.slave_addr, 0x10, 0x30, 0x01,  # 从机地址+功能码+寄存器地址(3001)
                0x00, 0x02, 0x04,                    # 寄存器数量(2个)+数据长度(4字节)
                *self.THREE_BYTE_ADDR,               # 发射端三字节地址[0x10,0x04,0x5D]
                0xB1                                 # 开始充电指令位（文档固定）
            ]
            # 发送指令并校验响应（0x10功能码响应为指令前6字节）
            resp = self._send_cmd(cmd_start)
            if resp and resp[:6] == bytes(cmd_start[:6]):
                res.success = True
                res.message = f"三字节地址模式-开始充电成功 | 发射端地址：{[hex(x) for x in self.THREE_BYTE_ADDR]}"
            else:
                res.success = False
                res.message = f"三字节地址模式-开始充电失败 | 响应异常：{[hex(b) for b in resp]}"
        except Exception as e:
            res.success = False
            res.message = f"三字节地址模式-开始充电异常：{str(e)}"
        return res

    # -------------------------- 三字节地址模式-停止充电（核心适配） --------------------------
    def _stop_charge_cb(self, req, res):
        """
        三字节地址停止充电指令格式（文档严格定义）：
        [从机地址, 0x10, 0x30, 0x01, 0x00, 0x02, 0x04, 发射端地址1, 发射端地址2, 发射端地址3, 0xB2]
        对应：01 10 30 01 00 02 04 10 04 5D B2 + 自动CRC
        """
        try:
            # 构造三字节地址模式-停止充电核心指令
            cmd_stop = [
                self.slave_addr, 0x10, 0x30, 0x01,  # 从机地址+功能码+寄存器地址(3001)
                0x00, 0x02, 0x04,                    # 寄存器数量(2个)+数据长度(4字节)
                *self.THREE_BYTE_ADDR,               # 发射端三字节地址[0x10,0x04,0x5D]
                0xB2                                 # 停止充电指令位（文档固定）
            ]
            # 发送指令并校验响应（0x10功能码响应为指令前6字节）
            resp = self._send_cmd(cmd_stop)
            if resp and resp[:6] == bytes(cmd_stop[:6]):
                res.success = True
                res.message = f"三字节地址模式-停止充电成功 | 发射端地址：{[hex(x) for x in self.THREE_BYTE_ADDR]}"
            else:
                res.success = False
                res.message = f"三字节地址模式-停止充电失败 | 响应异常：{[hex(b) for b in resp]}"
        except Exception as e:
            res.success = False
            res.message = f"三字节地址模式-停止充电异常：{str(e)}"
        return res

    # -------------------------- 手动查询电压电流 --------------------------
    def _query_volt_curr_cb(self, req, res):
        cmd = [self.slave_addr, 0x03, 0x00, 0x01, 0x00, 0x02]
        volt, curr = self._parse_volt_curr(self._send_cmd(cmd))
        if volt > 0 or curr > 0:
            res.success = True
            res.message = f"电压电流查询成功 | {volt:.1f}V | {curr:.2f}A"
        else:
            res.success = False
            res.message = "电压电流查询失败，未解析到有效数据"
        return res

    # -------------------------- 手动查询故障码 --------------------------
    def _query_fault_code_cb(self, req, res):
        cmd = [self.slave_addr, 0x03, 0x00, 0x05, 0x00, 0x01]
        fault_code, fault_msg = self._parse_fault_code(self._send_cmd(cmd))
        res.success = True
        res.message = f"故障码查询成功 | 0x{fault_code:04X} | {fault_msg}"
        return res

    # -------------------------- 节点销毁-关闭串口 --------------------------
    def destroy_node(self):
        with self.mutex:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("485串口已安全关闭")
        super().destroy_node()

# -------------------------- 主函数 --------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Charging485Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("充电485节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()