#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import time
import threading
import struct

# Modbus CRC16校验函数
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
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # 2. 初始化串口
        self.ser = None
        self.laser_distance = [0, 0]  # 存储两路激光距离
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
        
        # 3. 创建发布者
        self.distance_pub = self.create_publisher(
            Int16MultiArray,
            'laser_distance',
            10
        )
        
        # 4. 创建定时器（100ms读取一次数据）
        self.timer = self.create_timer(0.1, self.read_laser_data)

    def send_laser_command(self, cmd):
        """发送激光传感器指令（0x01/0x02）"""
        if not self.ser or not self.ser.is_open:
            return False
        
        try:
            # 构造发送缓冲区（根据实际传感器协议调整）
            send_buf = bytearray(8)
            send_buf[0] = cmd  # 指令标识
            
            # 计算并填充CRC
            crc = mb_crc_calculate(send_buf[:6])
            send_buf[6] = crc & 0xFF
            send_buf[7] = (crc >> 8) & 0xFF
            
            # 发送数据
            self.ser.write(send_buf)
            return True
        except Exception as e:
            self.get_logger().warn(f"发送指令0x{cmd:02X}失败: {str(e)}")
            return False

    def read_serial_response(self, expected_cmd, timeout=0.1):
        """读取并解析串口响应"""
        rx_buf = bytearray()
        start_time = time.time()
        
        # 读取数据直到超时或获取完整数据包
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                rx_buf += self.ser.read(self.ser.in_waiting)
                # 检查是否获取到完整的9字节数据包
                if len(rx_buf) >= 9:
                    # 验证指令标识
                    if rx_buf[0] == expected_cmd:
                        # 验证CRC
                        crc = mb_crc_calculate(rx_buf[:9])
                        if crc == 0:
                            # 解析距离值 (rx_buf[5] << 8 | rx_buf[6])
                            distance = (rx_buf[5] << 8) | rx_buf[6]
                            return distance
                        else:
                            self.get_logger().warn(f"0x{expected_cmd:02X}数据CRC校验失败")
                    break
            time.sleep(0.001)
        
        return None

    def read_laser_data(self):
        """读取两路激光传感器数据"""
        # 读取第一路激光数据（0x01指令）
        if self.send_laser_command(0x01):
            distance1 = self.read_serial_response(0x01)
            if distance1 is not None:
                with self.mutex:
                    self.laser_distance[0] = distance1
                self.get_logger().debug(f"激光1距离: {distance1} mm")
        
        # 读取第二路激光数据（0x02指令）
        if self.send_laser_command(0x02):
            distance2 = self.read_serial_response(0x02)
            if distance2 is not None:
                with self.mutex:
                    self.laser_distance[1] = distance2
                self.get_logger().debug(f"激光2距离: {distance2} mm")
        
        # 发布距离数据
        self.publish_distance_data()

    def publish_distance_data(self):
        """发布解析后的距离数据"""
        msg = Int16MultiArray()
        msg.layout.dim.append(Int16MultiArray._DIMENSION_MESSAGE_TYPE())
        msg.layout.dim[0].size = 2
        msg.layout.dim[0].stride = 1
        msg.layout.dim[0].label = "laser_distance"
        
        with self.mutex:
            msg.data = [self.laser_distance[0], self.laser_distance[1]]
        
        self.distance_pub.publish(msg)

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
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