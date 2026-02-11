#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import serial
import time
import threading
from std_msgs.msg import Float32MultiArray, Int16
from std_srvs.srv import Trigger
from custom_msgs.srv import ChargeControl

# -------------------------- Modbus CRC16 校验函数 --------------------------
# 严格匹配协议中CRC16_CCITT公式，低位在前、高位在后
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
    # 低位在前，高位在后
    crc_l = wcrc & 0xFF
    crc_h = (wcrc >> 8) & 0xFF
    return bytes([crc_l, crc_h])

# -------------------------- 故障码映射表（根据《故障代码表》补充） --------------------------
# FAULT_CODE_MAP = {
#     0x0000: "无故障",
#     0x0001: "过压故障",
#     0x0002: "欠压故障",
#     0x0003: "过流故障",
#     0x0004: "过热故障",
#     0x0005: "通信故障",
#     0x0006: "充电超时",
#     0x0007: "电池异常",
#     # 可根据实际故障代码表扩展
# }

# -------------------------- 充电控制节点类 --------------------------
class Charging485Node(Node):
    def __init__(self):
        super().__init__('charging_485_node')
        # 1. 声明并获取串口参数（支持Launch配置）
        self.declare_parameter('serial_port', '/dev/ttyS9')
        self.declare_parameter('baud_rate', 9600)  # 485常用波特率9600
        self.declare_parameter('slave_addr', 0x01)  # 从机地址默认01
        self.declare_parameter('timeout', 0.5)      # 串口超时时间

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.slave_addr = self.get_parameter('slave_addr').value
        self.timeout = self.get_parameter('timeout').value

        # 2. 初始化串口
        self.ser = None
        self.mutex = threading.Lock()  # 线程安全锁
        self._init_serial()

        # 3. ROS2 话题发布
        self.volt_curr_pub = self.create_publisher(
            Float32MultiArray,
            'charging_volt_curr',
            10
        )
        # 新增：故障码发布话题
        self.fault_code_pub = self.create_publisher(
            Int16,
            'charging_fault_code',
            10
        )

        # 4. ROS2 服务创建：提供指令调用接口
        self.srv_start_charge = self.create_service(ChargeControl, 'start_charging', self._start_charge_cb)
        self.srv_stop_charge = self.create_service(ChargeControl, 'stop_charging', self._stop_charge_cb)
        self.srv_query_volt_curr = self.create_service(Trigger, 'query_volt_curr', self._query_volt_curr_cb)
        # 新增：故障码查询服务
        self.srv_query_fault = self.create_service(Trigger, 'query_fault_code', self._query_fault_code_cb)

        # 5. 定时器：定时查询电压电流和故障码（默认500ms）
        self.timer = self.create_timer(0.5, self._timer_query_data)
        self.get_logger().info(f"充电485解析节点启动成功 | 串口：{self.serial_port} | 从机地址：0x{self.slave_addr:02X}")

    def _init_serial(self):
        """初始化485串口（RS485通常为8N1）"""
        try:
            with self.mutex:
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout
                )
                if not self.ser.is_open:
                    self.ser.open()
        except serial.SerialException as e:
            self.get_logger().fatal(f"串口初始化失败：{str(e)}")
            rclpy.shutdown()

    def _send_cmd(self, cmd: list) -> bytes:
        """发送指令：拼接CRC + 串口发送 + 接收响应"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().error("串口未打开，发送指令失败")
            return b''
        try:
            # 拼接指令字节并计算CRC
            cmd_bytes = bytes(cmd)
            crc_bytes = crc16_modbus(cmd_bytes)
            send_bytes = cmd_bytes + crc_bytes
            # 发送并清空接收缓冲区
            with self.mutex:
                self.ser.flushInput()
                self.ser.write(send_bytes)
                self.get_logger().debug(f"发送指令：{[hex(b) for b in send_bytes]}")
                # 接收响应（根据协议，响应长度不超过16字节）
                resp = self.ser.read(16)
            if resp:
                self.get_logger().debug(f"接收响应：{[hex(b) for b in resp]}")
            return resp
        except Exception as e:
            self.get_logger().error(f"指令发送失败：{str(e)}")
            return b''

    def _parse_volt_curr(self, resp: bytes) -> tuple[float, float]:
        """解析电压电流响应（协议：01 03 04 [电压高] [电压低] [电流高] [电流低] CRC）"""
        volt, curr = 0.0, 0.0
        # 校验响应格式：长度≥8 + 从机地址匹配 + 功能码03 + 数据长度04
        if len(resp) >= 8 and resp[0] == self.slave_addr and resp[1] == 0x03 and resp[2] == 0x04:
            # 解析16位寄存器值，单位：根据实际协议调整（示例：电压*0.01V，电流*0.01A）
            volt_raw = (resp[3] << 8) | resp[4]
            curr_raw = (resp[5] << 8) | resp[6]
            volt = volt_raw * 0.01
            curr = curr_raw * 0.01
            # 校验CRC
            if crc16_modbus(resp[:-2]) == resp[-2:]:
                self.get_logger().info(f"解析成功 | 电压：{volt:.1f}V | 电流：{curr:.2f}A")
            else:
                self.get_logger().warn("电压电流响应CRC校验失败")
        else:
            self.get_logger().warn("电压电流响应格式错误")
        return volt, curr

    def _parse_fault_code(self, resp: bytes) -> bytes:
        """解析故障码响应（寄存器0x05，长度2字节）"""
        fault_code = 0x0000
        
        # 校验响应格式：长度≥7 + 从机地址匹配 + 功能码03 + 数据长度02
        if len(resp) >= 7 and resp[0] == self.slave_addr and resp[1] == 0x03 and resp[2] == 0x02:
            # 解析16位故障码
            if crc16_modbus(resp[:-2]) == resp[-2:]:
                fault_code = (resp[3] << 8) | resp[4]
            # 校验CRC
                # return fault_code
            #     # 查找故障码含义
            #     fault_msg = FAULT_CODE_MAP.get(fault_code, f"未定义故障码: 0x{fault_code:04X}")
            #     self.get_logger().info(f"故障码解析成功 | 故障码：0x{fault_code:04X} | 含义：{fault_msg}")
            # else:
            #     self.get_logger().warn("故障码响应CRC校验失败")
        else:
            self.get_logger().warn("故障码响应格式错误")
        
        # return fault_code, fault_msg
        return fault_code

    # -------------------------- 定时器回调：定时查询电压电流+故障码 --------------------------
    def _timer_query_data(self):
        """定时器自动查询电压电流和故障码并发布话题"""
        # 1. 查询电压电流
        cmd_volt_curr = [self.slave_addr, 0x03, 0x00, 0x01, 0x00, 0x02]
        resp_volt_curr = self._send_cmd(cmd_volt_curr)
        volt, curr = self._parse_volt_curr(resp_volt_curr)
        # 发布电压电流话题
        msg_volt_curr = Float32MultiArray()
        msg_volt_curr.data = [volt, curr]
        self.volt_curr_pub.publish(msg_volt_curr)

        # 2. 查询故障码
        cmd_fault = [self.slave_addr, 0x03, 0x00, 0x05, 0x00, 0x01]  # 寄存器0x05，长度1（2字节）
        resp_fault = self._send_cmd(cmd_fault)
        fault_code, fault_msg = self._parse_fault_code(resp_fault)
        # 发布故障码话题
        msg_fault = Int16()
        msg_fault.data = fault_code
        self.fault_code_pub.publish(msg_fault)

    # -------------------------- 服务回调：开始充电 --------------------------
    def _start_charge_cb(self, req, res):
        """开始充电服务回调（修复参数错误：req, res）"""
        mode = 1  # 默认单字节地址模式
        addr = [0x01]  # 修正：addr应为列表格式
        try:
            if mode == 0:  # 默认地址模式
                cmd = [self.slave_addr, 0x06, 0x00, 0x00, 0x00, 0xB1]
            elif mode == 1:  # 单字节地址模式（addr长度必须为1）
                if len(addr) != 1:
                    res.success = False
                    res.message = "单字节地址模式需要1个地址字节"
                    return res
                cmd = [self.slave_addr, 0x06, 0x00, 0x00, addr[0], 0xB1]
            elif mode == 2:  # 三字节地址模式（addr长度必须为3）
                if len(addr) != 3:
                    res.success = False
                    res.message = "三字节地址模式需要3个地址字节"
                    return res
                cmd = [self.slave_addr, 0x10, 0x30, 0x01, 0x00, 0x02, 0x04] + addr + [0xB1]
            else:
                res.success = False
                res.message = "模式错误：0=默认/1=单字节/2=三字节"
                return res
            # 发送指令
            self._send_cmd(cmd)
            res.success = True
            res.message = f"开始充电成功 | 模式：{mode} | 地址：{[hex(a) for a in addr]}"
        except Exception as e:
            res.success = False
            res.message = f"开始充电失败：{str(e)}"
        return res

    # -------------------------- 服务回调：停止充电 --------------------------
    def _stop_charge_cb(self, req, res):
        """停止充电服务回调（修复参数错误：req, res）"""
        mode = 1  # 默认单字节地址模式
        addr = [0x01]  # 修正：addr应为列表格式
        try:
            if mode == 0:  # 默认地址模式
                cmd = [self.slave_addr, 0x06, 0x00, 0x00, 0x00, 0xB2]
            elif mode == 1:  # 单字节地址模式
                if len(addr) != 1:
                    res.success = False
                    res.message = "单字节地址模式需要1个地址字节"
                    return res
                cmd = [self.slave_addr, 0x06, 0x00, 0x00, addr[0], 0xB2]
            elif mode == 2:  # 三字节地址模式
                if len(addr) != 3:
                    res.success = False
                    res.message = "三字节地址模式需要3个地址字节"
                    return res
                cmd = [self.slave_addr, 0x10, 0x30, 0x01, 0x00, 0x02, 0x04] + addr + [0xB2]
            else:
                res.success = False
                res.message = "模式错误：0=默认/1=单字节/2=三字节"
                return res
            # 发送指令
            self._send_cmd(cmd)
            res.success = True
            res.message = f"停止充电成功 | 模式：{mode} | 地址：{[hex(a) for a in addr]}"
        except Exception as e:
            res.success = False
            res.message = f"停止充电失败：{str(e)}"
        return res

    # -------------------------- 服务回调：查询电压电流 --------------------------
    def _query_volt_curr_cb(self, req, res):
        """手动查询电压电流服务回调（修复参数错误：req, res）"""
        cmd = [self.slave_addr, 0x03, 0x00, 0x01, 0x00, 0x02]
        resp = self._send_cmd(cmd)
        volt, curr = self._parse_volt_curr(resp)
        if volt > 0 or curr > 0:
            res.success = True
            res.message = f"查询成功 | 电压：{volt:.1f}V | 电流：{curr:.2f}A"
        else:
            res.success = False
            res.message = "查询失败，未解析到有效数据"
        return res

    # -------------------------- 新增：服务回调：查询故障码 --------------------------
    def _query_fault_code_cb(self, req, res):
        """手动查询故障码服务回调"""
        # 构造故障码查询指令：[从机地址, 功能码03, 寄存器0005高, 寄存器0005低, 数量0001高, 数量0001低]
        cmd = [self.slave_addr, 0x03, 0x00, 0x05, 0x00, 0x01]
        resp = self._send_cmd(cmd)
        fault_code, fault_msg = self._parse_fault_code(resp)
        
        if fault_code is not None:
            res.success = True
            res.message = f"故障码查询成功 | 故障码：0x{fault_code:04X} | 含义：{fault_msg}"
        else:
            res.success = False
            res.message = "故障码查询失败，未解析到有效数据"
        return res

    # -------------------------- 节点销毁 --------------------------
    def destroy_node(self):
        """节点销毁时关闭串口"""
        with self.mutex:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("485串口已关闭")
        super().destroy_node()

# -------------------------- 主函数 --------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Charging485Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()