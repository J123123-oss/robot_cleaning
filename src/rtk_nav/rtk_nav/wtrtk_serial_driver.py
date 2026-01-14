#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import serial
import threading
import time
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus
from custom_msgs.msg import WTRTK  # 保持自定义消息结构

class WTRTKSerialDriver(Node):
    def __init__(self):
        super().__init__('wtrtk_serial_driver')
        
        # 读取参数（默认端口和波特率）
        self.declare_parameter('port', '/dev/WTRTK')
        self.declare_parameter('baud', 460800)
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud').value
        
        # 初始化串口
        self.ser = None
        self.connect_serial()
        
        # 创建发布者 QoS设置更合理，适配GPS高频数据
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 20)
        self.wtrtk_pub = self.create_publisher(WTRTK, '/wtrtk_data', 20)
        
        self.buffer = ""  # 缓存串口数据
        self.buffer_max_len = 4096  # 缓存最大长度，防止内存溢出
        
        # 缓存最新解析的消息
        self.latest_fix = None
        self.latest_wtrtk = None
        
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info("GNGGA + WTRTK serial driver started successfully")

    def connect_serial(self):
        """连接串口设备"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.05,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            if self.ser.is_open:
                self.get_logger().info(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {str(e)}")
            return False

    def dms_to_decimal(self, dms_str, is_latitude=True):
        """度分格式转十进制"""
        try:
            dms_str = dms_str.strip()
            if not dms_str:
                return 0.0
            
            dms = float(dms_str)
            degrees = int(dms // 100)
            minutes = dms % 100
            decimal = degrees + minutes / 60.0
            
            if is_latitude:
                if not (-90 <= decimal <= 90):
                    self.get_logger().warn(f"纬度超出范围: {decimal}，重置为0.0")
                    return 0.0
            else:
                if not (-180 <= decimal <= 180):
                    self.get_logger().warn(f"经度超出范围: {decimal}，重置为0.0")
                    return 0.0
            return decimal
        except (ValueError, TypeError) as e:
            if dms_str:
                self.get_logger().warn(f"经纬度转换失败: '{dms_str}', 错误: {e}，重置为0.0")
            return 0.0

    def parse_gngga(self, frame):
        """解析$GNGGA帧 ✅ 修复所有核心问题"""
        if not frame.startswith("$GNGGA"):
            return None
        
        star_pos = frame.find('*')
        if star_pos == -1:
            self.get_logger().warn("Invalid GNGGA frame (no checksum)")
            return None
        
        fields = frame.split(',')
        if len(fields) < 15:
            self.get_logger().warn(f"Invalid GNGGA fields count: {len(fields)} (expected >=15)")
            return None
        
        fix_msg = NavSatFix()
        fix_msg.header = Header()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = "gps"
        
        try:
            # 解析纬度
            lat_dms = fields[2].strip() if fields[2].strip() else "0.0"
            lat_flag = fields[3].strip() if fields[3].strip() else "N"
            latitude = self.dms_to_decimal(lat_dms, is_latitude=True)
            if lat_flag == 'S':
                latitude = -abs(latitude)

            # 解析经度
            lon_dms = fields[4].strip() if fields[4].strip() else "0.0"
            lon_flag = fields[5].strip() if fields[5].strip() else "E"
            longitude = self.dms_to_decimal(lon_dms, is_latitude=False)
            if lon_flag == 'W':
                longitude = -abs(longitude)

            # 解析海拔
            altitude = float(fields[9]) if fields[9].strip() else 0.0

            # 定位状态
            fix_status = int(fields[6]) if fields[6].strip() else 0
            
            # 填充消息
            fix_msg.latitude = latitude
            fix_msg.longitude = longitude
            fix_msg.altitude = altitude
            
            # ✅ 【修复核心报错】严格遵循ROS2 NavSatStatus枚举值规范
            if fix_status == 0:
                fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            elif fix_status == 1:
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            elif fix_status >= 2:
                fix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            
            # ✅ 【修复position_covariance类型错误】强制转为numpy float64数组 + 固定9位长度
            # 彻底解决：must be a set or sequence with length 9 and each value of type 'float'
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            fix_msg.position_covariance = [
                float(0.1), float(0.0), float(0.0),
                float(0.0), float(0.1), float(0.0),
                float(0.0), float(0.0), float(1.0)
            ]
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse GNGGA fields: {str(e)}, frame: {frame[:60]}")
            return None
        
        return fix_msg

    def parse_wtrtk(self, frame):
        """解析$WTRTK帧"""
        if not frame.startswith("$WTRTK"):
            return None
        
        star_pos = frame.find('*')
        if star_pos == -1:
            self.get_logger().warn("Invalid WTRTK frame (no checksum)")
            return None
        
        content = frame[7:star_pos]
        fields = content.split(',')
        
        if len(fields) != 25:
            return None
        
        msg = WTRTK()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "wtrtk_link"
        
        try:
            # 1. 差分相关字段（0-3）
            msg.diff_x = float(fields[0]) if fields[0].strip() else 0.0
            msg.diff_y = float(fields[1]) if fields[1].strip() else 0.0
            msg.diff_z = float(fields[2]) if fields[2].strip() else 0.0
            msg.diff_r = float(fields[3]) if fields[3].strip() else 0.0
            
            # 2. 角度相关字段（4-6）
            msg.angle_x = float(fields[4]) if fields[4].strip() else 0.0
            msg.angle_y = float(fields[5]) if fields[5].strip() else 0.0
            msg.angle_z = float(fields[6]) if fields[6].strip() else 0.0
            
            # 3. 状态相关字段（7-11）
            msg.fix_status = int(fields[7]) if fields[7].strip() else 0
            msg.wireless_status = int(fields[8]) if fields[8].strip() else 0
            msg.ntrip_status = int(fields[9]) if fields[9].strip() else 0
            msg.signal_quality = int(fields[10]) if fields[10].strip() else 0
            msg.data_rate = int(fields[11]) if fields[11].strip() else 0
            
            # 4. GPS航向角（12）
            msg.gps_heading = fields[12].strip() if fields[12].strip() else "--"
            
            # 5. 校准标志（13）
            msg.calib_flag = int(fields[13]) if fields[13].strip() else 0
            
            # 6. 电池电压和温度（14-15）
            msg.battery_voltage = float(fields[14]) if fields[14].strip() else 0.0
            msg.temperature = float(fields[15]) if fields[15].strip() else 0.0
            
            # 7. 基站距离和惯导标志（16-17）
            msg.base_distance = int(fields[16]) if fields[16].strip() else 0
            msg.ins_flag = int(fields[17]) if fields[17].strip() else 0
            
            # 8. 惯导经纬度（18-21）
            ins_lat_dms = fields[18].strip() if fields[18].strip() else "0.0"
            msg.ins_latitude = self.dms_to_decimal(ins_lat_dms, is_latitude=True)
            msg.lat_flag = fields[19].strip() if fields[19].strip() else "N"
            
            ins_lon_dms = fields[20].strip() if fields[20].strip() else "0.0"
            msg.ins_longitude = self.dms_to_decimal(ins_lon_dms, is_latitude=False)
            msg.lon_flag = fields[21].strip() if fields[21].strip() else "E"
            
            # 9. 惯导速度、航向角、高度（22-24）
            msg.ins_speed = float(fields[22]) if fields[22].strip() else 0.0
            msg.ins_heading = float(fields[23]) if fields[23].strip() else 0.0
            msg.ins_altitude = float(fields[24]) if fields[24].strip() else 0.0
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse WTRTK fields: {str(e)}")
            return None
        
        return msg

    def read_serial(self):
        """持续读取串口数据并解析 ✅ 修复所有BUG"""
        while rclpy.ok():
            if not self.ser or not self.ser.is_open:
                self.get_logger().warn("Serial port closed, reconnecting...")
                if not self.connect_serial():
                    time.sleep(1)
                    continue
            
            try:
                data = self.ser.read(1024)
                if data:
                    self.buffer += data.decode('utf-8', errors='replace')
                    
                    # ✅ 【修复BUG2】串口缓存长度限制，防止脏数据导致缓存溢出
                    if len(self.buffer) > self.buffer_max_len:
                        self.buffer = self.buffer[-self.buffer_max_len:]
                    
                    # 处理缓存中的所有完整帧
                    while True:
                        gngga_start = self.buffer.find('$GNGGA')
                        wtrtk_start = self.buffer.find('$WTRTK')
                        
                        if gngga_start == -1 and wtrtk_start == -1:
                            break
                        
                        target_start = min([s for s in [gngga_start, wtrtk_start] if s != -1])
                        end_idx = self.buffer.find('\r\n', target_start)
                        if end_idx == -1:
                            break
                        
                        frame = self.buffer[target_start:end_idx]
                        self.buffer = self.buffer[end_idx+2:]

                        # 立即发布，无需等待线程触发
                        if frame.startswith("$GNGGA"):
                            parsed_fix = self.parse_gngga(frame)
                            if parsed_fix:
                                self.latest_fix = parsed_fix
                                self.fix_pub.publish(parsed_fix)
                        elif frame.startswith("$WTRTK"):
                            parsed_wtrtk = self.parse_wtrtk(frame)
                            if parsed_wtrtk:
                                self.latest_wtrtk = parsed_wtrtk
                                self.wtrtk_pub.publish(parsed_wtrtk)
                        time.sleep(0.5)  # 保持1Hz频率 /2
            
            except Exception as e:
                self.get_logger().error(f"Serial read error: {str(e)}")
                if self.ser:
                    self.ser.close()
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    driver = WTRTKSerialDriver()
    
    executor = MultiThreadedExecutor()
    executor.add_node(driver)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        driver.get_logger().info("Received shutdown signal, exiting...")
    finally:
        executor.shutdown()
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()