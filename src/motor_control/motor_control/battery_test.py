#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Float32MultiArray  # 保留电池数据发布（可选，也可移除）

class BatteryParser(Node):
    def destroy_node(self):
        """清理资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()

    def __init__(self):
        super().__init__('battery_parser_node')
        
        # 参数配置
        self.declare_parameter('serial_port', '/dev/ttyS4')
        self.declare_parameter('baudrate', 19200)
        
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 电池相关配置（更新为新指令：0B 04 00 00 00 03 B0 A1）
        # self.battery_cmd = bytes.fromhex("0B 04 00 00 00 03 B0 A1")
        # self.battery_cmd = bytes.fromhex("01 04 00 00 00 03 B0 0B")
        self.battery_cmd = bytes.fromhex("01 06 00 10 01 90 89 F3")  #修改无线充电流为4A
        self.battery_polling_interval = 1.0 # 电池查询间隔
        self.last_battery_poll_time = 0
        
        self.ser = None
        self.reconnect_interval = 1.0  # 重连间隔
        self.last_reconnect_time = 0
        self.buffer = bytearray()  # 串口接收缓冲区

        # 数据更新监控参数
        self.last_data_time = self.get_clock().now()
        self.data_timeout = 3.0  # 数据超时时间(秒)

        # 初始化串口
        try:
            if not self.init_serial():
                self.get_logger().warn("Failed to initialize serial connection. Will retry in main loop.")
        except Exception as e:
            self.get_logger().error("Exception occurred while initializing serial connection: %s" % str(e))
            self.get_logger().warn("Will retry in main loop.")

        # 电池数据发布器（可选，如需订阅可保留，仅打印可移除）
        self.battery_pub = self.create_publisher(Float32MultiArray, '/battery_data', 1)
        
        # 统一轮询定时器
        self.polling_timer = self.create_timer(0.005, self.polling_callback)
        
        # 主循环
        self.main_loop_timer = self.create_timer(0.005, self.main_loop)

    def init_serial(self):
        """初始化/重新初始化串口连接"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info("Successfully connected to %s" % self.port)
            return True
        except Exception as e:
            self.get_logger().error("Serial connection failed: %s" % str(e))
            return False

    def safe_serial_write(self, data):
        """安全的串口数据写入"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
                return True
            return False
        except Exception as e:
            self.get_logger().warn("Serial write failed: %s" % str(e))
            self.init_serial()  # 尝试重新连接
            return False

    def polling_callback(self):
        """电池轮询回调函数，发送电池查询指令"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 轮询电池
        if current_time - self.last_battery_poll_time >= self.battery_polling_interval:
            send_success = self.safe_serial_write(self.battery_cmd)
            if send_success:
                cmd_hex = ' '.join(format(x, '02x') for x in self.battery_cmd)
                self.get_logger().info(f"Sent battery query command: {cmd_hex}")
            else:
                self.get_logger().warn("Failed to send battery query command")
            self.last_battery_poll_time = current_time

    def parse_battery_response(self, data):
        """解析电池返回数据，打印原始数据和解析结果"""
        # 1. 打印完整的返回原始数据（16进制格式，便于调试）
        raw_data_hex = ' '.join(format(x, '02x') for x in data)
        self.get_logger().info(f"------------------------")
        self.get_logger().info(f"Received battery raw data: {raw_data_hex}")
        self.get_logger().info(f"Received battery raw data (decimal): {list(data)}")
        self.get_logger().info(f"------------------------")
        
        # 2. 基础格式校验
        # 响应格式: 设备地址(1) + 功能码(1) + 数据长度(1) + 数据(6字节) + CRC(2)
        if len(data) < 9 or data[0] != 0x0B or data[1] != 0x04:
            self.get_logger().error(f"Invalid battery frame: "
                                  f"length={len(data)}, "
                                  f"device addr=0x{data[0]:02x} (expected 0x0B), "
                                  f"func code=0x{data[1]:02x} (expected 0x04)")
            return None
        
        # 3. 数据长度校验（3个寄存器，每个2字节，共6字节数据）
        if data[2] != 0x06:
            self.get_logger().error(f"Battery data length mismatch: expected 6, actual={data[2]}")
            return None
        
        expected_frame_length = 1 + 1 + 1 + 6 + 2  # 地址+功能码+长度+数据+CRC
        if len(data) != expected_frame_length:
            self.get_logger().error(f"Battery frame length mismatch: expected={expected_frame_length}, actual={len(data)}")
            return None
        
        # 4. CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        recv_crc_hex = ' '.join(format(x, '02x') for x in recv_crc)
        calc_crc_hex = ' '.join(format(x, '02x') for x in calc_crc)
        if recv_crc != calc_crc:
            self.get_logger().error(f"Battery CRC check failed: "
                                  f"received CRC={recv_crc_hex}, "
                                  f"calculated CRC={calc_crc_hex}")
            return None
        
        # 5. 解析具体数据（大端模式：高位在前，低位在后）
        battery_data = {}
        
        # 5.1 电池剩余容量百分比（寄存器0，无符号16位，单位0.01%）
        capacity_h = data[3]
        capacity_l = data[4]
        capacity_raw = (capacity_h << 8) | capacity_l
        capacity_percent = capacity_raw * 0.01
        battery_data['capacity_percent'] = capacity_percent
        
        # 5.2 电池组总电流（寄存器1，有符号16位，单位0.01A，补码处理）
        current_h = data[5]
        current_l = data[6]
        current_raw = (current_h << 8) | current_l
        # 处理有符号数（16位补码：超过0x7FFF即为负数）
        if current_raw > 0x7FFF:
            current_raw -= 0x10000
        total_current = current_raw * 0.01
        battery_data['total_current'] = total_current
        
        # 5.3 电池组总电压（寄存器2，无符号16位，单位0.01V）
        voltage_h = data[7]
        voltage_l = data[8]
        voltage_raw = (voltage_h << 8) | voltage_l
        total_voltage = voltage_raw * 0.01
        battery_data['total_voltage'] = total_voltage
        
        # 6. 打印解析后的详细结果（格式化输出，便于阅读）
        self.get_logger().info(f"✅ Battery data parsed successfully:")
        self.get_logger().info(f"  - 电池剩余容量百分比: {capacity_percent:.2f}% "
                              f"(原始值: 0x{capacity_h:02x}{capacity_l:02x} = {capacity_raw})")
        self.get_logger().info(f"  - 电池组总电流: {total_current:.2f}A "
                              f"(原始值: 0x{current_h:02x}{current_l:02x} = {current_raw}) "
                              f"（正数=充电，负数=放电）")
        self.get_logger().info(f"  - 电池组总电压: {total_voltage:.2f}V "
                              f"(原始值: 0x{voltage_h:02x}{voltage_l:02x} = {voltage_raw})")
        
        return battery_data

    def main_loop(self):
        """主循环：读取串口数据，处理完整帧"""
        try:
            # 检查数据是否超时
            elapsed_time = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
            if elapsed_time > self.data_timeout:
                self.get_logger().warn(f"Data timeout ({elapsed_time:.2f}s since last data), resetting connection")
                self.last_data_time = self.get_clock().now()
                self.init_serial()
            
            # 读取串口数据
            if self.ser and self.ser.is_open:
                # 读取所有可用数据
                bytes_available = self.ser.in_waiting
                if bytes_available > 0:
                    data = self.ser.read(min(bytes_available, 1024))
                    if data:
                        self.buffer += data
                        self.get_logger().debug(f"Buffer updated, current size: {len(self.buffer)} bytes")

                # 处理完整帧
                processed_frames = 0
                while len(self.buffer) >= 11 and processed_frames < 10:  # 电池帧固定最小11字节
                    # 查找帧头 - 匹配电池设备地址(0x0B)
                    header_pos = -1
                    for i in range(len(self.buffer)):
                        if self.buffer[i] == 0x0B:
                            header_pos = i
                            break
                    
                    if header_pos == -1:
                        self.get_logger().debug("No battery frame header found, clearing buffer")
                        self.buffer.clear()
                        break
                    
                    # 丢弃帧头前的无效数据
                    if header_pos > 0:
                        self.get_logger().debug(f"Discarding {header_pos} bytes of invalid data before frame header")
                        del self.buffer[:header_pos]
                    
                    # 检查是否有足够的数据处理完整帧
                    if len(self.buffer) < 11:
                        self.get_logger().debug("Insufficient data for complete battery frame, waiting for more")
                        break
                    
                    # 提取完整帧并处理
                    func_code = self.buffer[1]
                    if func_code == 0x04:
                        expected_frame_length = 11  # 电池帧固定长度11
                        frame = self.buffer[:expected_frame_length]
                        del self.buffer[:expected_frame_length]
                        
                        # 解析电池数据（自动打印原始数据和解析结果）
                        parsed_data = self.parse_battery_response(frame)
                        if parsed_data:
                            self.publish_battery_data(parsed_data)
                            self.last_data_time = self.get_clock().now()
                    else:
                        self.get_logger().debug(f"Unknown function code: 0x{func_code:02x}, skipping byte")
                        del self.buffer[0]
                        continue
                    
                    processed_frames += 1
            
            # 检查串口连接状态，自动重连
            if not self.ser or not self.ser.is_open:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.last_reconnect_time > self.reconnect_interval:
                    self.get_logger().info("Attempting to reconnect to serial port")
                    if self.init_serial():
                        self.last_reconnect_time = current_time
        except Exception as e:
            self.get_logger().error(f"Main loop error: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.init_serial()

    def publish_battery_data(self, battery_data):
        """发布电池数据（可选，如需其他节点订阅可保留）"""
        msg = Float32MultiArray()
        msg.data = [
            battery_data['capacity_percent'],
            battery_data['total_current'],
            battery_data['total_voltage']
        ]
        self.battery_pub.publish(msg)

    @staticmethod
    def calculate_modbus_crc(data):
        """计算Modbus CRC16校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryParser()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()