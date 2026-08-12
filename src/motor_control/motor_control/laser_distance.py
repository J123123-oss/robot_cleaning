#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
# 使用UInt16MultiArray（支持0-65535无符号整数）
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension  
import serial
import time
import threading
import struct
import os
from datetime import datetime

# Modbus CRC16校验函数.
def mb_crc_calculate(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

class LaserDistanceNode(Node):
    def __init__(self):
        super().__init__('laser_distance_node')
            
        # 1. 声明并获取串口参数（支持launch配置）
        self.declare_parameter('serial_port', '/dev/laser')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('laser_log_dir', '/home/ztl/robot_cleaning/motor_start_log/laser_log')

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.laser_log_dir = self.get_parameter('laser_log_dir').get_parameter_value().string_value

        # 初始化激光调试日志文件（直接文件写入，绕过 rclpy logging 冲突）
        os.makedirs(self.laser_log_dir, exist_ok=True)
        self._laser_log_path = os.path.join(self.laser_log_dir, 'laser_debug.log')
        self._laser_log_fd = open(self._laser_log_path, 'a')  # 追加模式
        self._laser_log("激光调试日志启动")
        
        # 2. 初始化串口
        self.ser = None
        self.laser_distance = [0, 0]  # 存储两路激光距离（0-65535）
        self.laser_status = 0  # bit0=1路无回应, bit1=2路无回应
        self.last_laser_status_log_time = 0.0
        self.mutex = threading.Lock()
        
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1,  # 100ms超时
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f"串口已成功打开: {self.serial_port}, 波特率: {self.baud_rate}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"打开串口失败: {str(e)}")
            rclpy.shutdown()
            return
        
        # 3. 创建单话题发布者（UInt16MultiArray，支持0-65535）
        self.distance_pub = self.create_publisher(
            UInt16MultiArray,
            'laser_distance',  # 单话题名称
            10
        )
        
        # 4. 创建定时器（100ms读取一次数据）
        self.timer = self.create_timer(0.1, self.read_laser_data)

    def _laser_log(self, msg):
        """写激光调试日志到文件，每次立即刷新"""
        now = datetime.now()
        line = f"{now.strftime('%Y-%m-%d %H:%M:%S')}.{now.microsecond // 1000:03d} {msg}\n"
        self._laser_log_fd.write(line)
        self._laser_log_fd.flush()

    def send_laser_command(self, cmd):
        """发送激光传感器指令（0x01/0x02）"""
        if not self.ser or not self.ser.is_open:
            return False
        
        try:
            # 构造发送缓冲区（根据实际传感器协议调整）
            send_buf = bytearray(8)
            send_buf[0] = cmd  # 指令标识
            send_buf[1] = 0x03
            send_buf[2] = 0x00
            send_buf[3] = 0x00
            send_buf[4] = 0x00
            send_buf[5] = 0x02
            # 计算并填充CRC
            crc = mb_crc_calculate(send_buf[:6])
            send_buf[6] = crc & 0xFF
            send_buf[7] = (crc >> 8) & 0xFF
            
            # 发送数据
            self.ser.write(send_buf)
            # self._laser_log(f"发送指令 0x{cmd:02X}: {send_buf.hex(' ')}")
            return True
        except Exception as e:
            self.get_logger().warn(f"发送指令0x{cmd:02X}失败: {str(e)}")
            return False

    def read_serial_response(self, expected_cmd, timeout=0.1):
        """读取并解析串口响应"""
        rx_buf = bytearray()
        start_time = time.time()
        
        # 清空串口缓冲区，避免残留数据干扰
        self.ser.flushInput()
        
        # 读取数据直到超时或获取完整数据包
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                rx_buf += self.ser.read(self.ser.in_waiting)
                # 完整的Modbus响应包是9字节（地址1+功能码1+字节数1+数据4+CRC2）
                if len(rx_buf) >= 9:
                    # 验证从机地址（指令标识）
                    if rx_buf[0] == expected_cmd:
                        # 验证CRC：计算前7字节（地址+功能码+字节数+数据）的CRC
                        crc_calculated = mb_crc_calculate(rx_buf[:7])
                        # 接收的CRC是最后2字节（低位在前，高位在后）
                        crc_received = (rx_buf[8] << 8) | rx_buf[7]
                        if crc_calculated == crc_received:
                            # 解析寄存器值（0-65535范围）
                            register2 = (rx_buf[5] << 8) | rx_buf[6]
                            # 强制限定数值在0-5000范围
                            register2 = register2 if register2 <= 5000 else 5000
                            # self._laser_log(
                            #     f"收到响应 0x{expected_cmd:02X}: 原始={rx_buf.hex(' ')}, "
                            #     f"距离={register2} mm"
                            # )
                            return register2
                        else:
                            self.get_logger().warn(f"0x{expected_cmd:02X}数据CRC校验失败: 计算值0x{crc_calculated:04X}, 接收值0x{crc_received:04X}")
                    break
            time.sleep(0.001)
        
        return None

    def read_laser_data(self):
        """读取两路激光传感器数据"""
        status = 0
        # 读取第一路激光数据（0x01指令,修改为01）
        if self.send_laser_command(0x02):
            distance1 = self.read_serial_response(0x02)
            if distance1 is not None:
                with self.mutex:
                    self.laser_distance[1] = distance1
            else:
                status |= 0x02
            #     self.get_logger().info(f"激光1距离: {distance1} mm")
            # else:
            #     self.get_logger().warn("激光1数据读取失败")
        else:
            status |= 0x02
        
        # 短暂延时，避免两路指令冲突
        time.sleep(0.05)
        
        # 读取第二路激光数据（0x02指令）
        if self.send_laser_command(0x01):
            distance2 = self.read_serial_response(0x01)
            if distance2 is not None:
                with self.mutex:
                    self.laser_distance[0] = distance2
            else:
                status |= 0x01
            #     self.get_logger().info(f"激光2距离: {distance2} mm")
            # else:
            #     self.get_logger().warn("激光2数据读取失败")
        else:
            status |= 0x01

        # with self.mutex:
        #     self.laser_status = status
        # now = time.time()
        # if status and now - self.last_laser_status_log_time >= 60.0:
        #     self.get_logger().warn(f"激光传感器无回应: status=0x{status:02X}")
        #     self.last_laser_status_log_time = now
        
        # 发布合并后的距离数据
        self.publish_distance_data()

    def publish_distance_data(self):
        """发布合并到单话题的两路激光数据（UInt16MultiArray）"""
        msg = UInt16MultiArray()
        
        # 设置数组维度信息（可选，但建议配置）
        dim = MultiArrayDimension()
        dim.label = "laser_distance"
        dim.size = 3  # 两路数据 + 状态位
        dim.stride = 3
        msg.layout.dim.append(dim)
        msg.layout.data_offset = 0
        
        with self.mutex:
            # 直接赋值两路无符号整数，无需边界检查（UInt16天然支持0-65535）
            msg.data = [self.laser_distance[0], self.laser_distance[1], self.laser_status]
        
        # 发布到单话题
        self.distance_pub.publish(msg)
        self.get_logger().debug(f"发布合并激光数据（毫米）: {msg.data}")

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        # 关闭激光调试日志文件
        self._laser_log("激光调试日志关闭")
        self._laser_log_fd.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaserDistanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
