#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import UInt8  # 发布IO数据

class IMUParser(Node):
    def destroy_node(self):
        """清理资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()

    def __init__(self):
        super().__init__('imu_parser_node')
        
        # 参数配置
        self.declare_parameter('serial_port', '/dev/ttyv1')
        self.declare_parameter('baudrate',115200)
        
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.device_addr = 0x50
        self.rx_frame_length = 7
        self.io_cmd = bytes.fromhex("01 02 00 00 00 08 79 CC")  # 修改IO采集命令为8位数据 (00 08 表示8个bit)
        self.ser = None
        self.reconnect_interval = 1.0  # 重连间隔
        self.last_reconnect_time = 0
        self.buffer = bytearray()  # 添加缓冲区初始化

        # 数据更新监控参数
        self.last_data_time = self.get_clock().now()  # 上次数据更新时间
        self.data_timeout = 3.0  # 数据超时时间(秒)

        # 添加轮询状态管理
        self.io_polling_interval = 0.02   # IO查询间隔，提高频率到100Hz
        self.last_io_poll_time = 0
        

        # 初始化串口
        try:
            if not self.init_serial():
                self.get_logger().warn("Failed to initialize serial connection. Will retry in main loop.")
        except Exception as e:
            self.get_logger().error("Exception occurred while initializing serial connection: %s" % str(e))
            self.get_logger().warn("Will retry in main loop.")

        # 发布IO数据
        self.io_pub = self.create_publisher(UInt8, '/io_data', 1)  # 修改为UInt8
        
        # 统一的轮询定时器，避免冲突（提高频率以支持更精确的IO轮询）
        self.polling_timer = self.create_timer(0.005, self.polling_callback)
        
        # 主循环 (提高频率以更快处理数据)
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
        """统一的轮询回调函数，避免设备冲突，提高IO查询频率"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 检查是否需要查询IO - 更频繁地查询IO
        if current_time - self.last_io_poll_time >= self.io_polling_interval:
            # 查询IO
            self.safe_serial_write(self.io_cmd)
            self.get_logger().debug("Sent IO query command: %s" % ' '.join(format(x, '02x') for x in self.io_cmd))
            self.last_io_poll_time = current_time

    def parse_io_response(self, data):
        """解析IO返回数据"""
        self.get_logger().debug("Received IO data: %s" % ' '.join(format(x, '02x') for x in data))
        
        # 检查响应是否符合Modbus RTU协议格式
        # 响应格式: 设备地址(1) + 功能码(1) + 数据长度(1) + 数据(N) + CRC(2)
        if len(data) < 4 or data[1] != 0x02:  # 功能码应为02, 最小长度为4（地址+功能码+长度+数据+CRC）
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
        # 数据格式: 01 02 01 XX YY ZZ (假设读取8个bit，数据长度为1)
        # XX 是1字节的IO状态数据，其中每个位代表一个IO口状态
        io_data_byte = data[3]  # 读取单个字节
        
        self.get_logger().debug("Parsed IO state: 0x%02x (%d)" % (io_data_byte, io_data_byte))
        return {'io_state': io_data_byte}

    def main_loop(self):
        """主循环"""
        try:
            # 检查数据是否超时
            elapsed_time = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
            if elapsed_time > self.data_timeout:
                self.get_logger().warn("Data timeout (%.2f seconds since last data), resetting connection" % elapsed_time)
                self.last_data_time = self.get_clock().now()  # 重置时间，避免连续触发
                self.init_serial()  # 重置连接
            
            # 读取串口数据
            if self.ser and self.ser.is_open:
                # 创建一个缓冲区来存储数据
                if not hasattr(self, 'buffer'):
                    self.buffer = bytearray()
                
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
                while len(self.buffer) >= 5 and processed_frames < 10:  # 至少需要5个字节才能确定帧长度
                    # 查找帧头 - 尝试匹配可能的设备地址
                    header_pos = -1
                    for i in range(len(self.buffer)):
                        # 检查是否是有效的Modbus地址 (01或0x50)
                        if self.buffer[i] in [0x01, 0x50]:
                            header_pos = i
                            break
                    
                    if header_pos == -1:
                        # 没有找到帧头，清空缓冲区
                        self.get_logger().debug("No frame header found, clearing buffer")
                        self.buffer.clear()
                        break
                    
                    # 丢弃帧头前的无效数据
                    if header_pos > 0:
                        self.get_logger().debug("Discarding %d bytes before frame header" % header_pos)
                        del self.buffer[:header_pos]
                    
                    # 检查是否有足够的数据来确定帧长度
                    if len(self.buffer) < 3:
                        # 等待更多数据
                        self.get_logger().debug("Insufficient data to determine frame length: %d bytes" % len(self.buffer))
                        break
                    
                    # 根据功能码确定帧长度
                    func_code = self.buffer[1]
                    if func_code == 0x02:  # IO响应
                        data_length = self.buffer[2]
                        expected_frame_length = 5 + data_length  # 地址(1) + 功能码(1) + 长度(1) + 数据(N) + CRC(2)
                    else:
                        # 未知功能码，跳过一个字节继续查找
                        self.get_logger().debug("Unknown function code: 0x%02x, skipping byte" % func_code)
                        del self.buffer[0]
                        continue
                    
                    # 检查数据长度是否足够
                    if len(self.buffer) < expected_frame_length:
                        # 等待更多数据
                        self.get_logger().debug("Insufficient data for frame: %d/%d bytes" % (len(self.buffer), expected_frame_length))
                        break
                    
                    # 提取并处理帧
                    frame = self.buffer[:expected_frame_length]
                    del self.buffer[:expected_frame_length]
                    
                    self.get_logger().debug("Processing frame: %s" % ' '.join(format(x, '02x') for x in frame))
                    
                    # 根据功能码选择解析函数
                    if func_code == 0x02:  # IO响应
                        parsed = self.parse_io_response(frame)
                        if parsed:
                            self.publish_io_data(parsed)
                            # 更新数据时间戳
                            self.last_data_time = self.get_clock().now()
                    
                    processed_frames += 1
            
            # 检查串口连接状态
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

    @staticmethod
    def calculate_crc(data):
        """Modbus CRC16校验"""
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
        """计算Modbus CRC校验"""
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
    node = IMUParser()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()