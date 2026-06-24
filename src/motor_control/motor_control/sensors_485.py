#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import UInt8, Float32MultiArray

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
        self.declare_parameter('io_baudrate', 9600)  # IO波特率
        self.declare_parameter('battery_baudrate', 4800)  # 电池波特率
        
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.io_baudrate = self.get_parameter('io_baudrate').get_parameter_value().integer_value
        self.battery_baudrate = self.get_parameter('battery_baudrate').get_parameter_value().integer_value
        
        self.battery_ID = 0x0B  # 电池设备地址
        self.io_cmd = bytes.fromhex("01 02 00 00 00 08 79 CC")  # IO采集命令
        # 电池基础参数查询命令
        self.battery_base_cmd = bytes.fromhex(f"{self.battery_ID:02x} 04 00 00 00 03 B0 A1")
        # 电池温度查询命令（地址0x0B，功能码0x03，寄存器0x0050，长度1）
        self.battery_temp_cmd = bytes.fromhex(f"{self.battery_ID:02x} 03 00 50 00 01 84 B1")
        
        self.ser = None
        self.reconnect_interval = 1.0  # 重连间隔
        self.last_reconnect_time = 0
        self.buffer = bytearray()  # 串口接收缓冲区
        self.current_baudrate = self.io_baudrate  # 当前波特率（默认IO波特率）
        self.latest_temp = 0.0  # 存储最新温度值

        # 数据更新监控参数
        self.last_data_time = self.get_clock().now()  # 上次数据更新时间
        self.data_timeout = 3.0  # 数据超时时间(秒)

        # 轮询状态管理
        self.io_polling_interval = 0.25  # IO查询间隔（2Hz）- 降低频率以避免干扰电池通信
        self.last_io_poll_time = 0
        self.battery_base_polling_interval = 10.0  # 电池基础参数查询间隔
        self.last_base_poll_time = 0
        self.battery_temp_polling_interval = 20.0    # 电池温度查询间隔（比基础参数更频繁）
        self.last_temp_poll_time = 0
        self.baudrate_switch_delay = 0.01  # 波特率切换延迟（确保稳定）

        # 初始化串口（默认IO波特率）
        try:
            if not self.init_serial(baudrate=self.io_baudrate):
                self.get_logger().warn("Failed to initialize serial connection. Will retry in main loop.")
        except Exception as e:
            self.get_logger().error("Exception occurred while initializing serial connection: %s" % str(e))
            self.get_logger().warn("Will retry in main loop.")

        # 发布器配置
        self.io_pub = self.create_publisher(UInt8, '/io_data', 1)
        self.battery_pub = self.create_publisher(Float32MultiArray, '/battery_data', 1)
        
        # 统一的轮询定时器
        self.polling_timer = self.create_timer(0.005, self.polling_callback)
        self.main_loop_timer = self.create_timer(0.005, self.main_loop)

    def init_serial(self, baudrate=None):
        """初始化/重新初始化串口连接（支持指定波特率）"""
        target_baudrate = baudrate if baudrate else self.io_baudrate
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                port=self.port,
                baudrate=target_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.current_baudrate = target_baudrate
            self.get_logger().info(f"Successfully connected to {self.port} with baudrate {target_baudrate}")
            return True
        except Exception as e:
            self.get_logger().error(f"Serial connection failed (baudrate {target_baudrate}): %s" % str(e))
            return False

    def switch_baudrate(self, target_baudrate):
        """切换串口波特率"""
        if self.current_baudrate == target_baudrate:
            return True  # 已为目标波特率，无需切换
        
        self.get_logger().debug(f"Switching baudrate from {self.current_baudrate} to {target_baudrate}")
        try:
            # 关闭当前串口并重新初始化
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                port=self.port,
                baudrate=target_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.current_baudrate = target_baudrate
            # 波特率切换后短暂延迟，确保稳定
            import time
            time.sleep(self.baudrate_switch_delay)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to switch baudrate to {target_baudrate}: {str(e)}")
            return False

    def safe_serial_write(self, data, target_baudrate=None):
        """安全的串口数据写入（支持指定波特率）"""
        try:
            # 如果指定了目标波特率，先切换
            if target_baudrate and not self.switch_baudrate(target_baudrate):
                return False
            
            if self.ser and self.ser.is_open:
                self.ser.write(data)
                return True
            return False
        except Exception as e:
            self.get_logger().warn(f"Serial write failed (baudrate {self.current_baudrate}): %s" % str(e))
            self.init_serial(target_baudrate if target_baudrate else self.current_baudrate)
            return False

    def polling_callback(self):
        """统一的轮询回调函数，分时轮询IO、电池基础参数、电池温度"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 轮询IO（9600波特率）- 降低频率以避免干扰电池通信
        if current_time - self.last_io_poll_time >= self.io_polling_interval:
            self.safe_serial_write(self.io_cmd, target_baudrate=self.io_baudrate)
            self.get_logger().debug("Sent IO query command: %s" % ' '.join(format(x, '02x') for x in self.io_cmd))
            self.last_io_poll_time = current_time
        
        # 轮询电池温度（4800波特率）
        if current_time - self.last_temp_poll_time >= self.battery_temp_polling_interval:
            if self.safe_serial_write(self.battery_temp_cmd, target_baudrate=self.battery_baudrate):
                # self.get_logger().info("Sent battery temp query command: %s" % ' '.join(format(x, '02x') for x in self.battery_temp_cmd))
                self.last_temp_poll_time = current_time
                # 电池命令发送后，在IO轮询间隙读取响应
                self.read_battery_response()
        
        # 轮询电池基础参数（4800波特率）
        if current_time - self.last_base_poll_time >= self.battery_base_polling_interval:
            if self.safe_serial_write(self.battery_base_cmd, target_baudrate=self.battery_baudrate):
                # self.get_logger().info("Sent battery base query command: %s" % ' '.join(format(x, '02x') for x in self.battery_base_cmd))
                self.last_base_poll_time = current_time
                # 电池命令发送后，在IO轮询间隙读取响应
                self.read_battery_response()
    
    def read_battery_response(self):
        """专门读取电池响应（在电池命令发送后调用）"""
        try:
            if not self.ser or not self.ser.is_open:
                return
            
            # 切换到电池波特率
            if self.current_baudrate != self.battery_baudrate:
                self.switch_baudrate(self.battery_baudrate)
            
            import time
            max_wait_time = 0.5  # 最多等待500ms确保完整接收
            start_wait = time.time()
            
            while time.time() - start_wait < max_wait_time:
                bytes_available = self.ser.in_waiting
                if bytes_available > 0:
                    data = self.ser.read(min(bytes_available, 1024))
                    if data:
                        self.buffer += data
                        # self.get_logger().info(f"Battery raw data: {' '.join(format(x, '02x') for x in data)}")
                        
                        # 检查是否已接收到完整帧（至少11字节）
                        if len(self.buffer) >= 11:
                            break
                else:
                    time.sleep(0.01)  # 短暂休眠避免CPU占用
            
            if len(self.buffer) >= 11:
                # self.get_logger().info(f"Full battery response received: {len(self.buffer)} bytes")
                self.process_battery_buffer()
        except Exception as e:
            self.get_logger().error(f"Error reading battery response: {str(e)}")
    
    def process_battery_buffer(self):
        """处理缓冲区中的电池数据"""
        while len(self.buffer) >= 3:
            if self.buffer[0] != self.battery_ID:
                del self.buffer[0]
                continue
            
            func_code = self.buffer[1]
            
            if func_code == 0x04:  # 电池基础参数响应
                expected_frame_length = 11
                if len(self.buffer) >= expected_frame_length:
                    frame = self.buffer[:expected_frame_length]
                    del self.buffer[:expected_frame_length]
                    parsed_data = self.parse_battery_base_response(frame)
                    if parsed_data:
                        self.publish_battery_data(parsed_data)
                        self.last_data_time = self.get_clock().now()
            
            elif func_code == 0x03:  # 电池温度响应
                expected_frame_length = 7
                if len(self.buffer) >= expected_frame_length:
                    frame = self.buffer[:expected_frame_length]
                    del self.buffer[:expected_frame_length]
                    parsed_data = self.parse_battery_temp_response(frame)
                    if parsed_data:
                        self.last_data_time = self.get_clock().now()
            
            else:
                del self.buffer[0]

    def parse_io_response(self, data):
        """解析IO返回数据"""
        if len(data) < 4 or data[1] != 0x02:
            self.get_logger().debug("Invalid IO frame: length=%d, func_code=0x%02x" % (len(data), data[1] if len(data) > 1 else 0))
            return None

        expected_length = 5 + data[2]
        if len(data) != expected_length:
            self.get_logger().debug("IO frame length mismatch: expected=%d, actual=%d" % (expected_length, len(data)))
            return None

        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        if recv_crc != calc_crc:
            self.get_logger().warn("IO: CRC check failed")
            return None

        io_data_byte = data[3]
        self.get_logger().debug("Parsed IO state: 0x%02x (%d)" % (io_data_byte, io_data_byte))
        return {'io_state': io_data_byte}

    def parse_battery_base_response(self, data):
        """解析电池基础参数返回数据"""
        if len(data) < 9 or data[0] != self.battery_ID or data[1] != 0x04:
            self.get_logger().debug("Invalid battery base frame: length=%d, addr=0x%02x, func_code=0x%02x" % 
                                  (len(data), data[0] if len(data) > 0 else 0, data[1] if len(data) > 1 else 0))
            return None
        
        if data[2] != 0x06:
            self.get_logger().debug("Battery base data length mismatch: expected=6, actual=%d" % data[2])
            return None
        
        expected_frame_length = 11
        if len(data) != expected_frame_length:
            self.get_logger().debug("Battery base frame length mismatch: expected=%d, actual=%d" % (expected_frame_length, len(data)))
            return None
        
        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        if recv_crc != calc_crc:
            self.get_logger().warn("Battery base: CRC check failed")
            return None
        
        # 解析参数
        battery_data = {}
        capacity_raw = (data[3] << 8) | data[4]
        battery_data['capacity_percent'] = capacity_raw * 0.01
        
        current_raw = (data[5] << 8) | data[6]
        if current_raw > 0x7FFF:
            current_raw -= 0x10000
        battery_data['total_current'] = current_raw * 0.01
        
        voltage_raw = (data[7] << 8) | data[8]
        battery_data['total_voltage'] = voltage_raw * 0.01
        
        # 加入最新温度值
        battery_data['temperature'] = self.latest_temp
        
        # 电压合理性校验（扩展范围以适应不同电池）
        if battery_data['total_voltage'] < 5.0 or battery_data['total_voltage'] > 60.0:
            self.get_logger().warn(f"Battery voltage out of range: {battery_data['total_voltage']:.2f}V")
            return None
        
        # self.get_logger().info(f"Parsed battery base: {battery_data['capacity_percent']:.2f}% | "
                            #   f"{battery_data['total_current']:.2f}A | {battery_data['total_voltage']:.2f}V | "
                            #   f"Temp: {battery_data['temperature']:.1f}℃")
        # self.get_logger().warn("Battery data parsed successfully - will publish to /battery_data topic")
        return battery_data

    def parse_battery_temp_response(self, data):
        """解析电池温度返回数据"""
        self.get_logger().debug("Received battery temp data: %s" % ' '.join(format(x, '02x') for x in data))
        
        # 基础格式校验
        if len(data) < 7 or data[0] != self.battery_ID or data[1] != 0x03:
            self.get_logger().debug("Invalid battery temp frame: length=%d, addr=0x%02x, func_code=0x%02x" % 
                                  (len(data), data[0] if len(data) > 0 else 0, data[1] if len(data) > 1 else 0))
            return None
        
        # 数据长度校验
        if data[2] != 0x02:
            self.get_logger().debug("Battery temp data length mismatch: expected=2, actual=%d" % data[2])
            return None
        
        expected_frame_length = 7
        if len(data) != expected_frame_length:
            self.get_logger().debug("Battery temp frame length mismatch: expected=%d, actual=%d" % (expected_frame_length, len(data)))
            return None
        
        # CRC校验
        recv_crc = data[-2:]
        calc_crc = self.calculate_modbus_crc(data[:-2])
        if recv_crc != calc_crc:
            self.get_logger().warn("Battery temp: CRC check failed")
            return None
        
        # 提取温度数据
        temp_raw = (data[3] << 8) | data[4]
        
        # 处理负温度补码
        if temp_raw & 0x8000:
            temp_raw = temp_raw - 0x10000
        
        # 温度转换（0.1℃单位）
        temperature = temp_raw * 0.1
        self.latest_temp = temperature  # 更新最新温度值
        
        # self.get_logger().info(f"Parsed battery temp: {temperature:.1f}℃")
        return {'temperature': temperature}

    def main_loop(self):
        """主循环：处理串口数据，解析IO、电池基础参数、电池温度帧"""
        try:
            # 检查数据超时
            elapsed_time = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
            if elapsed_time > self.data_timeout:
                self.get_logger().warn("Data timeout (%.2f seconds since last data), resetting connection" % elapsed_time)
                self.last_data_time = self.get_clock().now()
                self.init_serial()  # 超时后重置为IO波特率
            
            # 读取串口数据
            if self.ser and self.ser.is_open:
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
                    # 查找帧头（IO:0x01，电池:self.battery_ID）
                    header_pos = -1
                    for i in range(len(self.buffer)):
                        if self.buffer[i] in [0x01, self.battery_ID]:
                            header_pos = i
                            break
                    
                    if header_pos == -1:
                        self.get_logger().debug("No frame header found, clearing buffer")
                        self.buffer.clear()
                        break
                    
                    if header_pos > 0:
                        self.get_logger().debug("Discarding %d bytes before frame header" % header_pos)
                        del self.buffer[:header_pos]
                    
                    if len(self.buffer) < 3:
                        self.get_logger().debug("Insufficient data to determine frame length: %d bytes" % len(self.buffer))
                        break
                    
                    # 根据功能码处理数据
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
                    
                    elif func_code == 0x04:  # 电池基础参数响应
                        expected_frame_length = 11
                        if len(self.buffer) >= expected_frame_length:
                            frame = self.buffer[:expected_frame_length]
                            del self.buffer[:expected_frame_length]
                            parsed_data = self.parse_battery_base_response(frame)
                            if parsed_data:
                                self.publish_battery_data(parsed_data)
                                self.last_data_time = self.get_clock().now()
                    
                    elif func_code == 0x03:  # 电池温度响应
                        expected_frame_length = 7
                        if len(self.buffer) >= expected_frame_length:
                            frame = self.buffer[:expected_frame_length]
                            del self.buffer[:expected_frame_length]
                            parsed_data = self.parse_battery_temp_response(frame)
                            if parsed_data:
                                self.last_data_time = self.get_clock().now()
                    
                    else:
                        self.get_logger().debug("Unknown function code: 0x%02x, skipping byte" % func_code)
                        del self.buffer[0]
                        continue
                    
                    processed_frames += 1
            
            # 自动重连
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
        """发布电池数据（包含温度）"""
        msg = Float32MultiArray()
        # 数据顺序：[剩余容量百分比, 总电流, 总电压, 温度]
        msg.data = [
            battery_data['capacity_percent'],
            battery_data['total_current'],
            battery_data['total_voltage'],
            battery_data['temperature']
        ]
        self.battery_pub.publish(msg)

    @staticmethod
    def calculate_crc(data):
        """Modbus CRC16校验（兼容旧逻辑）"""
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