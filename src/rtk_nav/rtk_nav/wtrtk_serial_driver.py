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
from custom_msgs.msg import WTRTK

class WTRTKSerialDriver(Node):
    def __init__(self):
        super().__init__('wtrtk_serial_driver')
        
        # 读取参数（默认端口和波特率）
        self.declare_parameter('port', '/dev/WTRTK')
        self.declare_parameter('baud', 460800)
        self.declare_parameter('gga_timeout', 2.0)
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud').value
        self.gga_timeout = max(0.1, float(self.get_parameter('gga_timeout').value))
        
        # 初始化串口
        self.ser = None
        self.connect_serial()
        
        # 创建发布者 QoS设置更合理，适配GPS高频数据
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 20)
        self.wtrtk_pub = self.create_publisher(WTRTK, '/wtrtk_data', 20)

        self.buffer = ""  # 缓存串口数据
        self.buffer_max_len = 4096  # 缓存最大长度，防止内存溢出

        # GPGGA/GNGGA 缓存值，用于补充 WTRTK 的定位数据
        self._gga_lat = 0.0
        self._gga_lon = 0.0
        self._gga_position_status = 0
        self._last_gga_time = None

        # 缓存最新解析的消息
        self.latest_fix = None
        self.latest_wtrtk = None
        self._fix_updated = False
        self._wtrtk_updated = False
        self.data_lock = threading.Lock()
        self.timer = self.create_timer(0.1, self.publish_latest_data)
        
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info("GPGGA/GNGGA + WTRTK serial driver started successfully")

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
            # self.get_logger().error(f"Failed to open serial port {self.port}: {str(e)}")
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

    def parse_gga(self, frame):
        """解析$GPGGA或$GNGGA帧。"""
        if not frame.startswith(("$GPGGA", "$GNGGA")):
            return None

        star_pos = frame.find('*')
        if star_pos == -1:
            self.get_logger().warn("Invalid GGA frame (no checksum)")
            return None

        fields = frame.split(',')
        # 仅字段0-9是当前解析所必需的；部分设备会省略末尾差分时间等空字段。
        if len(fields) < 10:
            self.get_logger().warn(f"Invalid GGA fields count: {len(fields)} (expected >=10)")
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

            fix_msg.status.status = fix_status
            fix_msg.status.service = NavSatStatus.SERVICE_GPS

            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            fix_msg.position_covariance = [
                float(0.1), float(0.0), float(0.0),
                float(0.0), float(0.1), float(0.0),
                float(0.0), float(0.0), float(1.0)
            ]

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse GGA fields: {str(e)}, frame: {frame[:60]}")
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
        # self.get_logger().info("fields: %s" % fields)
        if len(fields) != 25:
            return None
        
        msg = WTRTK()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "wtrtk_link"
        msg.position_status = self._gga_position_status
        msg.position_data_valid = (
            self._last_gga_time is not None
            and time.monotonic() - self._last_gga_time <= self.gga_timeout
        )
        
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
        """持续读取串口数据并解析 ✅ 修复延迟和旧数据问题"""
        while rclpy.ok():
            # 检查串口是否打开，未打开则重连
            if not self.ser or not self.ser.is_open:
                self.get_logger().warn("Serial port closed, reconnecting...")
                if not self.connect_serial():
                    time.sleep(0.5)  # 缩短重连等待，减少阻塞
                    continue
            
            try:
                # 读取串口最新数据
                data = self.ser.read(1024)
                if data:
                    # 将新数据追加到缓冲区，并限制缓冲区长度
                    self.buffer += data.decode('utf-8', errors='replace')
                    if len(self.buffer) > self.buffer_max_len:
                        self.buffer = self.buffer[-self.buffer_max_len:]  # 只保留末尾的最新数据
                    
                    # 循环处理缓冲区中所有完整帧（从早到晚），避免 rfind 丢弃中间帧
                    while True:
                        frame_candidates = []
                        for frame_header, frame_type in (
                            ('$GPGGA', 'GGA'),
                            ('$GNGGA', 'GGA'),
                            ('$WTRTK', 'WTRTK'),
                        ):
                            frame_idx = self.buffer.find(frame_header)
                            if frame_idx != -1:
                                frame_candidates.append((frame_idx, frame_type))

                        if not frame_candidates:
                            # 没有任何帧头：跳过前导垃圾，保留可能的不完整帧头
                            dollar_idx = self.buffer.find('$')
                            if dollar_idx == -1:
                                self.buffer = ""
                            else:
                                self.buffer = self.buffer[dollar_idx:]
                            break

                        # 取最早出现的受支持帧头
                        target_start, frame_type = min(frame_candidates, key=lambda item: item[0])

                        # 查找帧尾 \r\n
                        end_idx = self.buffer.find('\r\n', target_start)
                        if end_idx == -1:
                            # 帧不完整，保留残片等待后续数据
                            self.buffer = self.buffer[target_start:]
                            break

                        # 提取完整帧
                        frame = self.buffer[target_start:end_idx]

                        if frame_type == "GGA":
                            parsed_fix = self.parse_gga(frame)
                            if parsed_fix is not None:
                                with self.data_lock:
                                    self.latest_fix = parsed_fix
                                    self._gga_lat = parsed_fix.latitude
                                    self._gga_lon = parsed_fix.longitude
                                    self._gga_position_status = parsed_fix.status.status
                                    self._last_gga_time = time.monotonic()
                                    self._fix_updated = True
                        elif frame_type == "WTRTK":
                            parsed_wtrtk = self.parse_wtrtk(frame)
                            if parsed_wtrtk is not None:
                                # 定向状态保留在 fix_status，定位状态来自最新GGA帧。
                                if self.latest_fix is not None:
                                    parsed_wtrtk.ins_latitude = self._gga_lat
                                    parsed_wtrtk.ins_longitude = self._gga_lon
                                with self.data_lock:
                                    self.latest_wtrtk = parsed_wtrtk
                                    self._wtrtk_updated = True

                        # 跳过已处理的帧，继续处理下一帧
                        self.buffer = self.buffer[end_idx+2:]
            
            except Exception as e:
                self.get_logger().error(f"Serial read error: {str(e)}")
                if self.ser:
                    self.ser.close()
                time.sleep(0.1)  # 缩短异常等待，减少阻塞
    def publish_latest_data(self):
        """仅发布新解析的数据，避免旧帧掩盖串口断流。"""
        with self.data_lock:
            fix_msg = self.latest_fix if self._fix_updated else None
            wtrtk_msg = self.latest_wtrtk if self._wtrtk_updated else None
            self._fix_updated = False
            self._wtrtk_updated = False

        if fix_msg is not None:
            self.fix_pub.publish(fix_msg)
        if wtrtk_msg is not None:
            self.wtrtk_pub.publish(wtrtk_msg)

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
