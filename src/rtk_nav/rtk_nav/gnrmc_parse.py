#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import serial
import threading
import time

class RMCSerialTester(Node):
    def __init__(self):
        super().__init__('rmc_serial_tester')
        
        # 读取串口参数（默认端口和波特率，与你的硬件一致）
        self.declare_parameter('port', '/dev/WTRTK')
        self.declare_parameter('baud', 460800)
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud').value
        
        # 初始化串口
        self.ser = None
        self.connect_serial()
        
        # 串口缓冲区配置
        self.buffer = ""
        self.buffer_max_len = 4096  # 防止内存溢出
        
        # 启动串口读取线程
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info(f"RMC Serial Tester started: {self.port} @ {self.baud_rate} baud")

    def connect_serial(self):
        """连接串口设备，失败返回False"""
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
                self.get_logger().info(f"Successfully connected to {self.port}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            return False

    def dms_to_decimal(self, dms_str, is_latitude=True):
        """度分格式转十进制度，复用原有稳定逻辑"""
        try:
            dms_str = dms_str.strip()
            if not dms_str:
                return 0.0
            
            dms = float(dms_str)
            degrees = int(dms // 100)
            minutes = dms % 100
            decimal = degrees + minutes / 60.0
            
            # 范围校验
            if is_latitude:
                if not (-90 <= decimal <= 90):
                    self.get_logger().warn(f"Invalid latitude: {decimal}, reset to 0.0")
                    return 0.0
            else:
                if not (-180 <= decimal <= 180):
                    self.get_logger().warn(f"Invalid longitude: {decimal}, reset to 0.0")
                    return 0.0
            return decimal
        except (ValueError, TypeError) as e:
            if dms_str:
                self.get_logger().warn(f"Failed to convert DMS: '{dms_str}', error: {e}, reset to 0.0")
            return 0.0

    def parse_rmc(self, frame):
        """专门解析RMC帧（$GPRMC/$GLRMC/$GNRMC），输出详细日志"""
        # 验证帧合法性
        valid_prefixes = ("$GPRMC", "$GLRMC", "$GNRMC")
        if not frame.startswith(valid_prefixes):
            return None
        
        star_pos = frame.find('*')
        if star_pos == -1:
            self.get_logger().warn("Invalid RMC frame: no checksum")
            return None
        
        fields = frame.split(',')
        if len(fields) < 13:
            self.get_logger().warn(f"Invalid RMC fields: {len(fields)} (expected >=13)")
            return None
        
        try:
            # 1. 识别卫星系统
            prefix = frame[:3]
            satellite_system = {
                "$GP": "GPS",
                "$GL": "GLONASS",
                "$GN": "ALL (GPS+GLONASS+others)"
            }.get(prefix, "UNKNOWN")
            
            # 2. 核心字段解析
            utc_time = fields[1].strip() if fields[1].strip() else "UNKNOWN"
            fix_status = fields[2].strip() if fields[2].strip() else "V"
            utc_date = fields[9].strip() if fields[9].strip() else "UNKNOWN"
            mode_indicator = fields[12].strip() if len(fields)>=13 and fields[12].strip() else "N"
            
            # 3. 经纬度解析
            lat_dms = fields[3].strip() if fields[3].strip() else "0.0"
            lat_flag = fields[4].strip() if fields[4].strip() else "N"
            latitude = self.dms_to_decimal(lat_dms, is_latitude=True)
            if lat_flag == 'S':
                latitude = -abs(latitude)
            
            lon_dms = fields[5].strip() if fields[5].strip() else "0.0"
            lon_flag = fields[6].strip() if fields[6].strip() else "E"
            longitude = self.dms_to_decimal(lon_dms, is_latitude=False)
            if lon_flag == 'W':
                longitude = -abs(longitude)
            
            # 4. 速率和航向解析
            ground_speed = round(float(fields[7].strip()) if fields[7].strip() else 0.0, 3)
            ground_course = round(float(fields[8].strip()) if fields[8].strip() else 0.0, 1)
            
            # 5. 打印详细解析结果（核心测试输出）
            self.get_logger().info("="*60)
            self.get_logger().info(f"RMC Frame Parsed Successfully (Satellite: {satellite_system})")
            self.get_logger().info(f"UTC Time: {utc_time} | UTC Date: {utc_date}")
            self.get_logger().info(f"Fix Status: {'VALID (A)' if fix_status == 'A' else 'INVALID (V)'} | Mode: {mode_indicator}")
            self.get_logger().info(f"Latitude: {latitude:.8f}° ({lat_flag}) | Longitude: {longitude:.8f}° ({lon_flag})")
            self.get_logger().info(f"Ground Speed: {ground_speed} knots | Ground Course: {ground_course}° (True North)")
            self.get_logger().info("="*60)
            
            return {
                'satellite_system': satellite_system,
                'utc_time': utc_time,
                'fix_status': fix_status,
                'latitude': latitude,
                'longitude': longitude,
                'ground_speed': ground_speed,
                'ground_course': ground_course
            }
        
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Failed to parse RMC fields: {str(e)} | Frame: {frame[:60]}")
            return None

    def read_serial(self):
        """持续读取串口，仅处理最新的RMC帧"""
        while rclpy.ok():
            # 串口重连逻辑
            if not self.ser or not self.ser.is_open:
                self.get_logger().warn("Serial port closed, reconnecting...")
                if not self.connect_serial():
                    time.sleep(0.5)
                    continue
            
            try:
                # 读取串口数据并更新缓冲区
                data = self.ser.read(1024)
                if data:
                    self.buffer += data.decode('utf-8', errors='replace')
                    if len(self.buffer) > self.buffer_max_len:
                        self.buffer = self.buffer[-self.buffer_max_len:]  # 只保留最新数据
                    
                    # 仅查找最新的RMC帧（兼容所有前缀）
                    rmc_indexes = [
                        self.buffer.rfind(prefix) for prefix in ("$GPRMC", "$GLRMC", "$GNRMC")
                    ]
                    latest_rmc_idx = max(rmc_indexes)
                    
                    if latest_rmc_idx != -1:
                        # 提取完整帧（以\r\n结尾）
                        end_idx = self.buffer.find('\r\n', latest_rmc_idx)
                        if end_idx != -1:
                            rmc_frame = self.buffer[latest_rmc_idx:end_idx]
                            self.buffer = self.buffer[end_idx+2:]  # 清空已解析数据，防止堆积
                            
                            # 解析最新RMC帧
                            self.parse_rmc(rmc_frame)
            
            except Exception as e:
                self.get_logger().error(f"Serial read error: {str(e)}")
                if self.ser:
                    self.ser.close()
                time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    rmc_tester = RMCSerialTester()
    
    try:
        rclpy.spin(rmc_tester)
    except KeyboardInterrupt:
        rmc_tester.get_logger().info("Received shutdown signal, exiting...")
    finally:
        rmc_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()