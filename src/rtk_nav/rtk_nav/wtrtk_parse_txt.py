#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from custom_msgs.msg import WTRTK  # 替换为实际的消息包名

class WTRTKFileParser(Node):
    def __init__(self):
        super().__init__('wtrtk_file_parser')
        
        # 声明并获取参数
        self.declare_parameter('file_path', '/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/rtkmsgs/返回.txt')
        self.declare_parameter('play_rate', 1.0)
        self.file_path = self.get_parameter('file_path').value
        self.play_rate = self.get_parameter('play_rate').value
        
        # 创建发布者（使用不同的回调组以支持多线程）
        # self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.wtrtk_pub = self.create_publisher(WTRTK, '/wtrtk_data', 10)
        
        self.buffer = ""  # 缓存文件读取的数据
        self.latest_fix = None  # 最新GPGGA/GNGGA解析结果
        self.latest_wtrtk = None  # 最新WTRTK解析结果
        self._fix_updated = False
        self._wtrtk_updated = False
        self.data_lock = threading.Lock()
        
        # 线程与事件
        self.publish_event = threading.Event()
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        self.read_thread = threading.Thread(target=self.read_file, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info(
            f"GPGGA/GNGGA + WTRTK file parser started. Reading from: {self.file_path}, "
            f"play rate: {self.play_rate}s/line"
        )

    def dms_to_decimal(self, dms_str, is_latitude=True):
        """度分格式转十进制（强化空字符串和异常处理）"""
        try:
            dms_str = dms_str.strip()
            if not dms_str:
                return 0.0
            
            dms = float(dms_str)
            degrees = int(dms // 100)
            minutes = dms % 100
            decimal = degrees + minutes / 60.0
            
            # 校验范围
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
        
        content = frame[7:star_pos]
        fields = content.split(',')
        
        # content已移除帧头，字段0-8是当前解析所必需的。
        if len(fields) < 9:
            self.get_logger().warn(f"Invalid GGA fields count: {len(fields)} (expected >=9)")
            return None
        
        fix_msg = NavSatFix()
        fix_msg.header = Header()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = "gps"
        
        try:
            # 纬度解析
            lat_dms = fields[1] if fields[1].strip() else "0.0"
            lat_flag = fields[2].strip() if len(fields) > 2 and fields[2].strip() else "N"
            latitude = self.dms_to_decimal(lat_dms, is_latitude=True)
            if lat_flag == 'S':
                latitude = -latitude
            
            # 经度解析
            lon_dms = fields[3] if fields[3].strip() else "0.0"
            lon_flag = fields[4].strip() if len(fields) > 4 and fields[4].strip() else "E"
            longitude = self.dms_to_decimal(lon_dms, is_latitude=False)
            if lon_flag == 'W':
                longitude = -longitude
            
            # 海拔解析
            altitude = float(fields[8]) if len(fields) > 8 and fields[8].strip() else 0.0
            
            # 定位状态
            fix_status = int(fields[5]) if len(fields) > 5 and fields[5].strip() else 0
            
            # 填充消息
            fix_msg.latitude = latitude
            fix_msg.longitude = longitude
            fix_msg.altitude = altitude
            fix_msg.status.status = fix_status
            fix_msg.status.service = 1  # GPS服务
            
            # 协方差设置
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            fix_msg.position_covariance = [
                float(0.1), float(0.0), float(0.0),
                float(0.0), float(0.1), float(0.0),
                float(0.0), float(0.0), float(1.0)
            ]
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse GGA fields: {str(e)}")
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
            self.get_logger().warn(f"Invalid WTRTK fields count: {len(fields)} (expected 25)")
            return None
        
        msg = WTRTK()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "wtrtk_link"
        msg.position_status = self.latest_fix.status.status if self.latest_fix is not None else 0
        msg.position_data_valid = self.latest_fix is not None
        
        try:
            # 差分相关字段（0-3）
            msg.diff_x = float(fields[0]) if fields[0].strip() else 0.0
            msg.diff_y = float(fields[1]) if fields[1].strip() else 0.0
            msg.diff_z = float(fields[2]) if fields[2].strip() else 0.0
            msg.diff_r = float(fields[3]) if fields[3].strip() else 0.0
            
            # 角度相关字段（4-6）
            msg.angle_x = float(fields[4]) if fields[4].strip() else 0.0
            msg.angle_y = float(fields[5]) if fields[5].strip() else 0.0
            msg.angle_z = float(fields[6]) if fields[6].strip() else 0.0
            
            # 状态相关字段（7-11）
            msg.fix_status = int(fields[7]) if fields[7].strip() else 0
            msg.wireless_status = int(fields[8]) if fields[8].strip() else 0
            msg.ntrip_status = int(fields[9]) if fields[9].strip() else 0
            msg.signal_quality = int(fields[10]) if fields[10].strip() else 0
            msg.data_rate = int(fields[11]) if fields[11].strip() else 0
            
            # GPS航向角（12）
            msg.gps_heading = fields[12].strip() if fields[12].strip() else "--"
            
            # 校准标志（13）
            msg.calib_flag = int(fields[13]) if fields[13].strip() else 0
            
            # 电池电压和温度（14-15）
            msg.battery_voltage = float(fields[14]) if fields[14].strip() else 0.0
            msg.temperature = float(fields[15]) if fields[15].strip() else 0.0
            
            # 基站距离和惯导标志（16-17）
            msg.base_distance = int(fields[16]) if fields[16].strip() else 0
            msg.ins_flag = int(fields[17]) if fields[17].strip() else 0
            
            # 经纬度解析（18-21）
            ins_lat_dms = fields[18].strip() if fields[18].strip() else "0.0"
            msg.ins_latitude = self.dms_to_decimal(ins_lat_dms, is_latitude=True)
            msg.lat_flag = fields[19].strip() if fields[19].strip() else "N"
            
            ins_lon_dms = fields[20].strip() if fields[20].strip() else "0.0"
            msg.ins_longitude = self.dms_to_decimal(ins_lon_dms, is_latitude=False)
            msg.lon_flag = fields[21].strip() if fields[21].strip() else "E"
            
            # 惯导速度、航向角、高度（22-24）
            msg.ins_speed = float(fields[22]) if fields[22].strip() else 0.0
            msg.ins_heading = float(fields[23]) if fields[23].strip() else 0.0
            msg.ins_altitude = float(fields[24]) if fields[24].strip() else 0.0
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse WTRTK fields: {str(e)}")
            return None
        
        return msg

    def publish_loop(self):
        """1Hz频率发布数据"""
        while rclpy.ok():
            if not self.publish_event.wait(timeout=1.0):
                continue

            with self.data_lock:
                fix_msg = self.latest_fix if self._fix_updated else None
                wtrtk_msg = self.latest_wtrtk if self._wtrtk_updated else None
                self._fix_updated = False
                self._wtrtk_updated = False
                self.publish_event.clear()
            
            # 发布GGA解析结果
            if fix_msg:
                fix_msg.header.stamp = self.get_clock().now().to_msg()
                # self.fix_pub.publish(fix_msg)
                self.get_logger().debug(
                    f"Published GGA: lat={fix_msg.latitude:.6f}, "
                    f"lon={fix_msg.longitude:.6f}, alt={fix_msg.altitude:.2f}"
                )
            
            # 发布WTRTK解析结果
            if wtrtk_msg:
                wtrtk_msg.header.stamp = self.get_clock().now().to_msg()
                self.wtrtk_pub.publish(wtrtk_msg)
                self.get_logger().debug(
                    f"Published WTRTK: ins_lat={wtrtk_msg.ins_latitude:.6f}, "
                    f"ins_lon={wtrtk_msg.ins_longitude:.6f}"
                )
            
            time.sleep(1.0)  # 1Hz

    def read_file(self):
        """从文件读取串口消息并处理"""
        while rclpy.ok():
            try:
                with open(self.file_path, 'r', encoding='utf-8') as f:
                    self.get_logger().info(f"Successfully opened file: {self.file_path}")
                    
                    for line in f:
                        if not rclpy.ok():
                            break
                            
                        line = line.strip()
                        if not line:
                            continue
                        
                        self.buffer += line + "\r\n"
                        self.parse_buffer()
                        time.sleep(self.play_rate)
                
                self.get_logger().info(f"File read completed. Re-reading after 1 second...")
                time.sleep(1)
                
            except FileNotFoundError:
                self.get_logger().error(f"File not found: {self.file_path}")
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f"File read error: {str(e)}")
                time.sleep(1)

    def parse_buffer(self):
        """解析缓存中的完整帧"""
        while True:
            gpgga_start = self.buffer.find('$GPGGA')
            gngga_start = self.buffer.find('$GNGGA')
            wtrtk_start = self.buffer.find('$WTRTK')

            gga_starts = [idx for idx in (gpgga_start, gngga_start) if idx != -1]
            gga_start = min(gga_starts) if gga_starts else -1

            if gga_start == -1 and wtrtk_start == -1:
                break

            if gga_start != -1 and (wtrtk_start == -1 or gga_start < wtrtk_start):
                # 处理GPGGA/GNGGA帧
                start_idx = gga_start
                end_idx = self.buffer.find('\r\n', start_idx)
                if end_idx == -1:
                    break
                frame = self.buffer[start_idx:end_idx]
                self.buffer = self.buffer[end_idx+2:]
                parsed_fix = self.parse_gga(frame)
                if parsed_fix:
                    with self.data_lock:
                        self.latest_fix = parsed_fix
                        self._fix_updated = True
                    self.publish_event.set()
            else:
                # 处理WTRTK帧
                start_idx = wtrtk_start
                end_idx = self.buffer.find('\r\n', start_idx)
                if end_idx == -1:
                    break
                frame = self.buffer[start_idx:end_idx]
                self.buffer = self.buffer[end_idx+2:]
                parsed_wtrtk = self.parse_wtrtk(frame)
                if parsed_wtrtk:
                    with self.data_lock:
                        self.latest_wtrtk = parsed_wtrtk
                        self._wtrtk_updated = True
                    self.publish_event.set()

def main(args=None):
    rclpy.init(args=args)
    parser = WTRTKFileParser()
    
    # 使用多线程执行器以支持多线程回调
    executor = MultiThreadedExecutor()
    executor.add_node(parser)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        parser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
