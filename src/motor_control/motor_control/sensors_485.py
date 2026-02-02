#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import UInt8, Float32MultiArray  # 新增Float32MultiArray发布电池数据

class Sensors_485(Node):
    def destroy_node(self):
        """清理资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()

    def __init__(self):
        super().__init__('Sensors_485_node')
        
        # 参数配置
        self.declare_parameter('serial_port', '/dev/ttyS4')
        self.declare_parameter('baudrate', 9600)
        
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.battery_ID = 0x0B # 电池设备地址   0B  /  01
        self.io_cmd = bytes.fromhex("01 02 00 00 00 08 79 CC")  # IO采集命令
        self.battery_cmd = bytes.fromhex(f"{self.battery_ID:02x} 04 00 00 00 03 B0 A1")  # 电池查询命令
        # self.battery_cmd = bytes.fromhex(f"01 04 00 00 00 03 B0 0B")  # 电池查询命令   self.battery_ID = 0x01
        self.ser = None
        self.reconnect_interval = 1.0  # 重连间隔
        self.last_reconnect_time = 0
        self.buffer = bytearray()  # 串口接收缓冲区

        # 数据更新监控参数
        self.last_data_time = self.get_clock().now()  # 上次数据更新时间
        self.data_timeout = 3.0  # 数据超时时间(秒)

        # 轮询状态管理
        self.io_polling_interval = 0.05   # IO查询间隔（100Hz）
        self.last_io_poll_time = 0
        # 新增：电池轮询配置（稍慢于IO，避免串口冲突）
        self.battery_polling_interval = 10.0
        self.last_battery_poll_time = 0

        # 初始化串口
        try:
            if not self.init_serial():
                self.get_logger().warn("Failed to initialize serial connection. Will retry in main loop.")
        except Exception as e:
            self.get_logger().error("Exception occurred while initializing serial connection: %s" % str(e))
            self.get_logger().warn("Will retry in main loop.")

        # 发布器配置
        self.io_pub = self.create_publisher(UInt8, '/io_data', 1)  # IO数据发布器
        self.battery_pub = self.create_publisher(Float32MultiArray, '/battery_data', 1)  # 新增：电池数据发布器
        
        # 统一的轮询定时器，避免冲突
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
        """统一的轮询回调函数，分时轮询IO和电池，避免串口冲突"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 轮询IO（高频）
        if current_time - self.last_io_poll_time >= self.io_polling_interval:
            self.safe_serial_write(self.io_cmd)
            self.get_logger().debug("Sent IO query command: %s" % ' '.join(format(x, '02x') for x in self.io_cmd))
            self.last_io_poll_time = current_time
        
        # 新增：轮询电池（低频，分时发送避免冲突）
        if current_time - self.last_battery_poll_time >= self.battery_polling_interval:
            self.safe_serial_write(self.battery_cmd)
            self.get_logger().debug("Sent battery query command: %s" % ' '.join(format(x, '02x') for x in self.battery_cmd))
            self.last_battery_poll_time = current_time

    def parse_io_response(self, data):
        """解析IO返回数据"""
        # self.get_logger().info("Received IO data: %s" % ' '.join(format(x, '02x') for x in data))
        
        # 检查响应是否符合Modbus RTU协议格式
        if len(data) < 4 or data[1] != 0x02:
            self.get_logger().debug("Invalid IO frame: length=%d, func_code=0x%02x" % (len(data), data[1] if len(data) > 1 else 0))
            return None

        # 检查数据长度是否匹配
        expected_length = 5 + data[2]  # 5 = 地址(1) + 功能码(1) + 长度字节(1) + CRC(2)
        if len(data) != expected_length:
            self.get_logger().debug("IO frame length mismatch: expected=%d, actual=%d" % (expected_length, len(data)))
            return None

        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        if recv_crc != calc_crc:
            self.get_logger().warn("IO: CRC check failed")
            return None

        # 解析IO数据
        io_data_byte = data[3]
        self.get_logger().debug("Parsed IO state: 0x%02x (%d)" % (io_data_byte, io_data_byte))
        return {'io_state': io_data_byte}

    def parse_battery_response(self, data):
        """新增：解析电池返回数据，返回格式化的电池参数"""
        # self.get_logger().info("Received battery data: %s" % ' '.join(format(x, '02x') for x in data))
        
        # 1. 基础格式校验（设备地址self.battery_ID，功能码0x04）
        if len(data) < 9 or data[0] != self.battery_ID or data[1] != 0x04:
            self.get_logger().debug("Invalid battery frame: length=%d, addr=0x%02x, func_code=0x%02x" % 
                                  (len(data), data[0] if len(data) > 0 else 0, data[1] if len(data) > 1 else 0))
            return None
        
        # 2. 数据长度校验（3个寄存器=6字节数据）
        if data[2] != 0x06:
            self.get_logger().debug("Battery data length mismatch: expected=6, actual=%d" % data[2])
            return None
        
        expected_frame_length = 11  # 地址(1)+功能码(1)+长度(1)+数据(6)+CRC(2)
        if len(data) != expected_frame_length:
            self.get_logger().debug("Battery frame length mismatch: expected=%d, actual=%d" % (expected_frame_length, len(data)))
            return None
        
        # 3. CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        if recv_crc != calc_crc:
            self.get_logger().warn("Battery: CRC check failed")
            return None
        
        # 4. 解析具体电池参数（大端模式，高位在前）
        battery_data = {}
        
        # 4.1 电池剩余容量百分比（无符号16位，0.01%单位）
        capacity_raw = (data[3] << 8) | data[4]
        battery_data['capacity_percent'] = capacity_raw * 0.01
        
        # 4.2 电池组总电流（有符号16位，补码，0.01A单位）
        current_raw = (data[5] << 8) | data[6]
        if current_raw > 0x7FFF:
            current_raw -= 0x10000
        battery_data['total_current'] = current_raw * 0.01
        
        # 4.3 电池组总电压（无符号16位，0.01V单位）
        voltage_raw = (data[7] << 8) | data[8]
        battery_data['total_voltage'] = voltage_raw * 0.01
        if battery_data['total_voltage'] < 10.0 or battery_data['total_voltage'] > 55.0:
            return None
        
        # 打印解析结果（便于调试）
        self.get_logger().info(f"Parsed battery: {battery_data['capacity_percent']:.2f}% | "
                              f"{battery_data['total_current']:.2f}A | {battery_data['total_voltage']:.2f}V")
        return battery_data

    def main_loop(self):
        """主循环：处理串口数据，解析IO和电池帧"""
        try:
            # 检查数据是否超时
            elapsed_time = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
            if elapsed_time > self.data_timeout:
                self.get_logger().warn("Data timeout (%.2f seconds since last data), resetting connection" % elapsed_time)
                self.last_data_time = self.get_clock().now()
                self.init_serial()
            
            # 读取串口数据
            if self.ser and self.ser.is_open:
                # 读取所有可用数据
                bytes_available = self.ser.in_waiting
                if bytes_available > 0:
                    self.get_logger().debug("Bytes available: %d" % bytes_available)
                    data = self.ser.read(min(bytes_available, 1024))
                    if data:
                        self.buffer += data
                        self.get_logger().debug("Buffer size after read: %d" % len(self.buffer))

                # 处理完整帧
                processed_frames = 0
                while len(self.buffer) >= 5 and processed_frames < 10:
                    # 查找帧头 - 匹配IO(0x01)、电池(self.battery_ID)设备地址（新增self.battery_ID）
                    header_pos = -1
                    for i in range(len(self.buffer)):
                        if self.buffer[i] in [0x01, self.battery_ID]:  # 新增电池地址self.battery_ID
                            header_pos = i
                            break
                    
                    if header_pos == -1:
                        self.get_logger().debug("No frame header found, clearing buffer")
                        self.buffer.clear()
                        break
                    
                    # 丢弃帧头前的无效数据
                    if header_pos > 0:
                        self.get_logger().debug("Discarding %d bytes before frame header" % header_pos)
                        del self.buffer[:header_pos]
                    
                    # 检查是否有足够的数据确定帧长度
                    if len(self.buffer) < 3:
                        self.get_logger().debug("Insufficient data to determine frame length: %d bytes" % len(self.buffer))
                        break
                    
                    # 根据功能码处理不同设备数据
                    func_code = self.buffer[1]
                    expected_frame_length = 0
                    parsed_data = None
                    
                    if func_code == 0x02:  # IO响应
                        data_length = self.buffer[2]
                        expected_frame_length = 5 + data_length
                        if len(self.buffer) >= expected_frame_length:
                            frame = self.buffer[:expected_frame_length]
                            del self.buffer[:expected_frame_length]
                            parsed_data = self.parse_io_response(frame)
                            if parsed_data:
                                self.publish_io_data(parsed_data)
                                self.last_data_time = self.get_clock().now()
                    
                    elif func_code == 0x04:  # 新增：电池响应（功能码0x04）
                        expected_frame_length = 11  # 电池帧固定长度11
                        if len(self.buffer) >= expected_frame_length:
                            frame = self.buffer[:expected_frame_length]
                            del self.buffer[:expected_frame_length]
                            parsed_data = self.parse_battery_response(frame)
                            if parsed_data:
                                self.publish_battery_data(parsed_data)  # 发布电池数据
                                self.last_data_time = self.get_clock().now()
                    
                    else:
                        # 未知功能码，跳过一个字节
                        self.get_logger().debug("Unknown function code: 0x%02x, skipping byte" % func_code)
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
            self.get_logger().error("Main loop error: %s" % str(e))
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.init_serial()

    def publish_io_data(self, io_data):
        """发布IO数据"""
        msg = UInt8()
        msg.data = io_data['io_state']
        self.io_pub.publish(msg)

    def publish_battery_data(self, battery_data):
        """新增：发布电池数据"""
        msg = Float32MultiArray()
        # 数据顺序：[剩余容量百分比, 总电流, 总电压]
        msg.data = [
            battery_data['capacity_percent'],
            battery_data['total_current'],
            battery_data['total_voltage']
        ]
        self.battery_pub.publish(msg)

    @staticmethod
    def calculate_crc(data):
        """Modbus CRC16校验（保留原有，兼容旧逻辑）"""
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
    
    @staticmethod
    def calculate_modbus_crc(data):
        """计算Modbus CRC校验（复用，保证校验一致性）"""
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
    node = Sensors_485()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()