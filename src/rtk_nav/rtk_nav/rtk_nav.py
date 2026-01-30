#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
from typing import Optional, List, Dict, Tuple, Generator

import rclpy
import re
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3  # 用于发布左右轮速度
from std_msgs.msg import String, UInt8, Float32       # 用于发布控制模式和导航状态
from custom_msgs.msg import WTRTK
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType

# -------------------------- 全局配置与枚举 --------------------------
# RTK导航配置
RTK_WAYPOINT_TOLERANCE = 0.1
RTK_HEADING_TOLERANCE = 0.1  # degree
LINEAR_SPEED_BASE = 5.0    # origin 0.0124
TURN_SPEED = 1.0      # origin 0.1
INITIAL_MOVE_TOLERANCE = 0.1
RTK_CALIBRATION_TIMEOUT = 5.0
IMU_CALIBRATION_TIMEOUT = 3.0
HEADING_CALIBRATION_TIMEOUT = 40.0

TURN_SPEED_FAST = 0.8  # 大误差快速转向基准速度
TURN_SPEED_MID = 0.6   # 中误差中等转向基准速度
TURN_SPEED_SLOW = 0.2  # 小误差慢速转向基准速度（防超调）
MAX_CORRECTION = 0.8   # 最大修正量

# straight line speed correction factor
STRAIGHT_PID_SCALE = 0.8
SPEED_LIMIT = 1.5 * LINEAR_SPEED_BASE

# 控制模式（与电机节点保持一致）
class ControlMode:
    REMOTE = "REMOTE"
    NORMAL = "NORMAL"
    RTK_NAV = "RTK_NAV"

# 导航状态枚举
class NavState:
    IDLE = "IDLE"
    INITIAL_MOVE = "INITIAL_MOVE"
    WAYPOINT_MOVE = "WAYPOINT_MOVE"
    WAYPOINT_CALIB = "WAYPOINT_CALIB"
    COMPLETED = "COMPLETED"
# -------------------------- 合并后的RTK控制+导航节点 --------------------------
class RTKNavControlNode(Node):
    def __init__(self):
        super().__init__('rtk_nav_control_node')

        self.process_percent = 0.0  # 路径文件处理进度百分比
        # ================== 原有RTKNavigator属性 ==================
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_waypoint_idx = 0
        self.current_gps: Optional[Tuple[float, float]] = None
        self.current_lon = 0.0
        self.current_lat = 0.0
        self.imu_yaw = 0.0
        self.imu_initialized = False
        self.imu_calibration_offset = 0.0
        self.last_yaw_error = 0.0
        self.current_control_mode = ControlMode.NORMAL


        self.current_segment_heading = None


        # Sensor 
        self.front_left = False # test, None origin
        self.front_right = False
        self.mid_left = None
        self.mid_right = None
        self.back_left = None
        self.back_right = None

        self.correct_speed_scale = 0.4
        # self.is_boundary_triggered = False # test, False origin
        # 定义参数描述：bool类型, 名称：is_boundary_triggered, 默认值：False
        boundary_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='手动强制开启/关闭边界触发, True=触发矫正, False=强制关闭边界矫正(屏蔽传感器)'
        )
        # 声明参数 + 绑定到类成员变量
        self.declare_parameter('is_boundary_triggered', False, boundary_param_desc)
        # 读取初始值（程序启动时的默认值）
        self.is_boundary_triggered = self.get_parameter('is_boundary_triggered').value

        # ========== ✅ 必须添加：参数回调函数, 监听参数修改事件 ==========
        self.add_on_set_parameters_callback(self.update_boundary_parameter)


        # 声明RTK路径参数
        self.declare_parameter("rtk_path_file", "/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/three_path_20260129_144149.txt")
        self.rtk_path_file = self.get_parameter("rtk_path_file").value
        self.path_dir = os.path.dirname(self.rtk_path_file)
        # self.rtk_path_file = self.declare_parameter(
        #     'rtk_path_file',
        #     # "/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"
        #     "/home/ubuntu/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"
        # )
        
        # self.path_dir = os.path.dirname(self.rtk_path_file)  # 获取路径文件所在目录


        # 导航上下文
        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None,
            "last_distance": 0.0,
            "last_target_heading": 0.0
        }

        # ================== 原有RTKControlNode属性 ==================
        self.rate = self.create_rate(4)
        self.nav_generator: Optional[Generator] = None
        self.nav_running = False
        self.multi_waypoint_generator = None  # 多点导航生成器

        # ROS2发布器/订阅器
        self.motor_speed_pub = self.create_publisher(Vector3, "/rtk/motor_speed", 10)
        self.nav_state_pub = self.create_publisher(String, "/rtk/nav_state", 10)
        self.imu_heading_pub = self.create_publisher(Float32, "/imu_heading", 10)
        self.control_mode_sub = self.create_subscription(String, "/control/mode", self.mode_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.heading_sub = self.create_subscription(WTRTK, '/wtrtk_data', self.heading_callback, 10)
        self.io_data_rtk_sub = self.create_subscription(String, '/io/data', self.io_data_rtk_callback, 10)
        # 定时器（10Hz驱动导航逻辑）
        self.rtk_nav_timer = self.create_timer(0.1, self.rtk_timer_callback)

        # 加载航点
        self.load_rtk_path()
        # 加载初始路径文件
        self.load_waypoints_from_file(self.rtk_path_file)
    
    def load_waypoints_from_file(self, file_path: str) -> bool:
        """从文件加载航点数据"""
        try:
            self.waypoints = []
            self.current_waypoint_idx = 0
              
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()[1:]  # 跳过表头
                for line in lines:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    seq, lon, lat, heading_deg = line.split(',')
                    self.waypoints.append((float(lon), float(lat), float(heading_deg)))
            
            self.get_logger().info(f"[RTKNav] 成功加载路径文件: {file_path}, 共 {len(self.waypoints)} 个航点")
            self.rtk_path_file = file_path  # 更新当前路径文件
            return True
            
        except Exception as e:
            self.get_logger().error(f"[RTKNav] 加载路径文件失败: {e}")
            return False
    def update_boundary_parameter(self, params):
        """
        ROS2动态参数回调函数：监听参数修改, 实时更新 self.is_boundary_triggered
        运行中修改参数后, 立刻生效, 无需重启节点
        """
        for param in params:
            if param.name == 'is_boundary_triggered':
                # 强制更新类成员变量
                self.is_boundary_triggered = param.value
                # 打印日志, 方便调试查看修改结果
                if self.is_boundary_triggered:
                    self.get_logger().warn(f"✅ [手动设置] 开启边界触发：self.is_boundary_triggered = {self.is_boundary_triggered}")
                else:
                    self.get_logger().info(f"✅ [手动设置] 关闭边界触发：self.is_boundary_triggered = {self.is_boundary_triggered} (强制屏蔽传感器矫正)")
        # 必须返回成功状态
        return SetParametersResult(successful=True)
    def get_boundary_correct_speed(self):
        """
        边界触发时的实时矫正速度计算
        返回：(left_speed_correct, right_speed_correct) 矫正速度(/s)
        核心逻辑：哪边传感器触发 → 向反方向小幅度移动矫正, 避开边界
        """
        base_correct_speed = self.correct_speed_scale * LINEAR_SPEED_BASE
        left_speed = 0.0
        right_speed = 0.0

        # 前侧传感器触发 → 小幅后退矫正
        if self.front_left or self.front_right:
            left_speed = base_correct_speed
            right_speed = -base_correct_speed
        
        # 后侧传感器触发 → 小幅前进矫正
        elif self.back_left or self.back_right:
            left_speed = -base_correct_speed
            right_speed = base_correct_speed

        # 左侧传感器触发(中左/前左/后左) → 小幅向右矫正,turn_right,+,+
        if self.mid_left or self.front_left or self.back_left:
            left_speed = base_correct_speed * 0.8
            right_speed = base_correct_speed * 0.8

        # 右侧传感器触发(中右/前右/后右) → 小幅向左矫正,turn_left,-,-
        if self.mid_right or self.front_right or self.back_right:
            left_speed = -base_correct_speed * 0.8
            right_speed = -base_correct_speed * 0.8

        self.get_logger().info(f"[RTKNav] 执行边界矫正, 矫正速度：左轮={left_speed:.2f},右轮={right_speed:.2f}")
        return (left_speed, right_speed)
    
    def io_data_rtk_callback(self, msg: UInt8):
        self.get_logger().info(f"[RTKNav] 收到IO数据: {msg.data}")
        # 位0 (1<<0 = 0x01)：前左
        self.front_left = (msg.data & 0x01) == 0x01
        
        # 位1 (1<<1 = 0x02)：前右
        self.front_right = (msg.data & 0x02) == 0x02
        
        # 位2 (1<<2 = 0x04)：中左
        self.mid_left = (msg.data & 0x04) == 0x04
        
        # 位3 (1<<3 = 0x08)：中右  8
        self.mid_right = (msg.data & 0x08) == 0x08
        
        # 位4 (1<<4 = 0x10)：后左 16
        self.back_left = (msg.data & 0x10) == 0x10
        
        # 位5 (1<<5 = 0x20)：后右  32
        self.back_right = (msg.data & 0x20) == 0x20

        # 逻辑：如果是【手动通过rqt设置为False】, 则不再更新这个变量, 永久保持False；否则正常读取传感器
        if not self.is_boundary_triggered:
            return
        
        self.is_boundary_triggered = msg.data & 0xFF
        # if msg.data:
        #     for m in self.motors:
        #         self.motor_set_speed(m["id"], 0) 
        #         self.get_logger().info(f"--------------Test Proximity Speed--------------")
        # if self.front_left or self.front_right:
        #     self.motor_set_speed(1, -0.3 * self.BASE_SPEED)
        #     self.motor_set_speed(2, 0.3 * self.BASE_SPEED)
        #     if self.mid_left or self.mid_right:
        #         self.motor_set_speed(1, 0)
        #         self.motor_set_speed(2, 0)
        #         while self.front_left or self.front_right and rclpy.ok():
        #             self.motor_set_speed(1, 0.3 * self.BASE_SPEED)
        #             self.motor_set_speed(2, -0.3 * self.BASE_SPEED)
        # if self.back_left or self.back_right:
        #     self.motor_set_speed(1, 0)
        #     self.motor_set_speed(2, 0)
        #     while self.front_left or self.front_right and rclpy.ok():
        #         self.motor_set_speed(1, 0.3 * self.BASE_SPEED)
        #         self.motor_set_speed(2, -0.3 * self.BASE_SPEED)

        # self.get_logger().info(f"IO状态: {msg.data}, front_left={self.front_left}, front_right={self.front_right}, "
        # f"mid_left={self.mid_left}, mid_right={self.mid_right}, "
        # f"back_left={self.back_left}, back_right={self.back_right}"
        # )

    def get_next_path_file(self) -> Optional[str]:
        """获取下一个路径文件（按文件名时间戳排序）"""
        try:
            # 1. 获取目录下所有符合命名规则的路径文件
            file_pattern = re.compile(r'.*_\d{8}_\d{6}\.txt')
            all_files = [f for f in os.listdir(self.path_dir) if file_pattern.match(f)]
            
            if not all_files:
                self.get_logger().warn("[RTKNav] 路径目录下未找到符合规则的路径文件")
                return None
            
            # 2. 按文件名中的时间戳排序（提取YYYYMMDD_HHMMSS部分）
            def extract_timestamp(filename: str) -> str:
                match = re.search(r'(\d{8}_\d{6})', filename)
                return match.group(1) if match else ''
            
            all_files.sort(key=extract_timestamp)
            total_files = len(all_files)  # 总文件数
            current_file = os.path.basename(self.rtk_path_file)
            
            # 3. 找到当前文件的索引
            try:
                current_idx = all_files.index(current_file)
            except ValueError:
                self.get_logger().warn(f"[RTKNav] 当前文件 {current_file} 不在路径目录中, 使用第一个文件")
                # 首次使用第一个文件, 进度 1/总数量
                progress_num = 1
                progress_percent = round((progress_num / total_files) * 100, 1)
                self.get_logger().info(f"[RTKNav] 路径文件进度：{progress_num}/{total_files}, {progress_percent}%")
                return os.path.join(self.path_dir, all_files[0])
            
            # 4. 计算并输出进度（当前文件索引+1 为已执行/待执行的序号）
            current_progress = current_idx + 1
            progress_percent = round((current_progress / total_files) * 100, 1)
            # 更新进度百分比
            self.process_percent = progress_percent
            self.get_logger().info(f"[RTKNav] 路径文件进度：{current_progress}/{total_files}, {progress_percent}%")
            
            # 5. 最后一个文件时结束循环（不再返回新文件）
            if current_idx >= total_files - 1:
                self.get_logger().info("[RTKNav] 已执行到最后一个路径文件（{current_file}）, 执行返回")
                return None
            
            # 6. 获取下一个文件（非最后一个时）
            next_idx = current_idx + 1
            next_file = all_files[next_idx]
            self.get_logger().info(f"[RTKNav] 准备切换到下一个路径文件：{next_file}")
            
            return os.path.join(self.path_dir, next_file)
            
        except Exception as e:
            self.get_logger().error(f"[RTKNav] 获取下一个路径文件失败: {e}")
            return None

    # ================== 原有RTKNavigator方法 ==================
    def load_rtk_path(self) -> bool:
        if not os.path.exists(self.rtk_path_file):
            self.get_logger().error(f"RTK路径文件不存在：{self.rtk_path_file}")
            return False

        try:
            with open(self.rtk_path_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()[1:]  # 跳过表头
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    seq, lon, lat, heading_deg = line.split(',')
                    self.waypoints.append((float(lon), float(lat), float(heading_deg)))
            self.get_logger().info(f"成功加载RTK航点{len(self.waypoints)}个")
            return True
        except Exception as e:
            self.get_logger().error(f"解析RTK文件失败：{str(e)}")
            return False

    def gps_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            self.get_logger().warn("GPS信号无效")
            return
        last_status = msg.status.status
        status_map = {0: "未定位", 1: "单点", 2: "差分", 5: "RTK Float", 4: "RTK Fixed"}
        # when status change, print info once
        if msg.status.status in status_map and msg.status.status != last_status:

            self.get_logger().info(f"GPS状态：{status_map[msg.status.status]}")
        last_status = msg.status.status
        self.current_gps = (msg.longitude, msg.latitude)
        self.current_lon = msg.longitude
        self.current_lat = msg.latitude
        # self.get_logger().info(f"current_lon: {self.current_lon}, current_lat: {self.current_lat}")


    def heading_callback(self, msg: WTRTK) -> None:
        ins_heading_deg = msg.ins_heading
        self.imu_yaw = ins_heading_deg  #install offset + x degree
        self.imu_yaw = math.fmod(self.imu_yaw + 180.0, 360.0) - 180.0
        self.imu_initialized = True
        imu_msg= Float32()
        imu_msg.data = self.imu_yaw
        self.imu_heading_pub.publish(imu_msg)

    # def heading_callback(self, msg: WTRTK) -> None:
    #     # 1. 提取原始航向角，加上硬件安装偏移90°
    #     ins_heading_deg = msg.ins_heading
    #     raw_heading_deg = ins_heading_deg + 90.0  # 硬件安装偏移：+90°

    #     # 2. 处理IMU校准（首次校准 + 重新校准）
    #     if not self.imu_initialized or self.need_imu_recalibrate:
    #         # 计算校准偏移量：反向补偿原始航向，使校准后初始航向为合理基准
    #         self.imu_calibration_offset = -raw_heading_deg
    #         # 重置重新校准标志位
    #         self.need_imu_recalibrate = False
    #         # 标记校准完成
    #         self.imu_initialized = True
    #         # 打印校准日志（修复原代码括号不匹配问题）
    #         self.get_logger().info(f"IMU校准完成！初始偏移：{abs(self.imu_calibration_offset):.2f}°")

    #     # 3. 计算校准后的最终航向角（原始航向 + 校准偏移）
    #     self.imu_yaw = raw_heading_deg + self.imu_calibration_offset

    #     # 4. 航向角归一化（避免角度溢出，统一到 [-180°, 180°]）
    #     self.imu_yaw = self.normalize_heading_deg(self.imu_yaw)

    #     # 5. 发布校准后的航向角消息
    #     imu_msg = Float32()
    #     imu_msg.data = self.imu_yaw
    #     self.imu_heading_pub.publish(imu_msg)


    def get_target_waypoint(self, current_waypoint_idx: int = None) -> Optional[Tuple[float, float, float]]:
        """获取当前目标航点（含航向角）, 到达最后一个航点时自动切换路径文件"""
        idx = current_waypoint_idx if current_waypoint_idx is not None else self.current_waypoint_idx
        
        # 检查是否到达最后一个航点
        if idx >= len(self.waypoints):
            self.get_logger().info("[RTKNav] 已到达当前路径文件的最后一个航点, 准备切换路径文件")
            
            # 获取下一个路径文件
            next_file = self.get_next_path_file()
            if next_file and self.load_waypoints_from_file(next_file):
                # 切换文件成功, 返回新文件的第一个航点
                self.current_waypoint_idx = 0
                return self.waypoints[0]
            else:
                # 没有下一个文件, 返回None表示结束
                self.get_logger().info("[RTKNav] 没有更多路径文件, 导航结束")
                self.current_control_mode = ControlMode.NORMAL
                return None
        
        # 返回当前目标航点
        return self.waypoints[idx]

    def calc_distance_to_waypoint(self, waypoint: Tuple[float, float, float]) -> float:
        if not self.current_gps:
            return float('inf')

        R = 6371000.0  # 地球半径（米）
        lon1, lat1 = self.current_gps
        lon2, lat2, _ = waypoint

        # 转换为弧度
        lon1_rad = math.radians(lon1)
        lat1_rad = math.radians(lat1)
        lon2_rad = math.radians(lon2)
        lat2_rad = math.radians(lat2)

        # Haversine公式计算距离
        delta_lon = lon2_rad - lon1_rad
        delta_lat = lat2_rad - lat1_rad

        a = math.sin(delta_lat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2)** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def get_path_heading(self, waypoint: Tuple[float, float, float]) -> float:
        """获取目标航点的路径航向角（转换为rad并归一化, 与IMU基准一致）"""
        # 修正：航点航向角是绝对角度, 需叠加IMU校准偏移（让路径航向与IMU基准对齐）
        heading_deg = waypoint[2] + self.imu_calibration_offset
        heading_rad = math.radians(heading_deg)
        # return math.fmod(heading_rad + math.pi, 2 * math.pi) - math.pi
        return math.fmod(heading_deg + 180.0, 360.0) - 180.0

    def get_heading_error(self, target_heading: float) -> float:
        """新增：计算当前航向与目标航向的误差（归一化到[-π, π], 单位：rad）"""
        heading_error = target_heading - self.imu_yaw
        # return math.fmod(heading_error + math.pi, 2 * math.pi) - math.pi
        return math.fmod(heading_error + 180.0, 360.0) - 180.0


    # def get_speed_correction(self, target_heading: float) -> float:
    #     """计算对称纠正量（保证左右转一致）"""
    #     yaw_error = self.get_heading_error(target_heading)
    #     yaw_error_abs = abs(yaw_error)

    #     # 误差死区：避免微小震荡
    #     if yaw_error_abs < RTK_WAYPOINT_TOLERANCE:
    #         self.last_yaw_error = 0.0
    #         return 0.0

    #     # 统一KP参数（左右转纠正量一致，不差异化）
    #     if yaw_error_abs > 30:
    #         kp = 0.40
    #     elif yaw_error_abs > 10:
    #         kp = 0.20
    #     else:
    #         kp = 0.08

    #     # 统一KD参数（抑制超调，左右转一致）
    #     kd = 0.1
    #     yaw_error_diff = yaw_error - self.last_yaw_error
    #     d_term = kd * yaw_error_diff

    #     # 计算修正量并限制上限（统一最大纠正量0.5）
    #     correction = (kp * yaw_error) - d_term
    #     self.last_yaw_error = yaw_error
    #     max_correction = 0.8
    #     return max(min(correction, max_correction), -max_correction)
    def get_speed_correction(self, target_heading: float) -> float:
        """计算对称纠正量（保证左右转一致，明确区分yaw_error正负）"""
        yaw_error = self.get_heading_error(target_heading)
        yaw_error_abs = abs(yaw_error)

        # 2. 误差死区：避免微小震荡
        if yaw_error_abs < RTK_WAYPOINT_TOLERANCE:
            self.last_yaw_error = 0.0
            return 0.0

        # 3. 统一KP参数（左右转纠正量一致，不差异化）
        if yaw_error_abs > 30:
            kp = 0.05
        elif yaw_error_abs > 10:
            kp = 0.03
        else:
            kp = 0.005

        # 4. 统一KD参数（抑制超调，左右转一致）
        kd = 0.03
        yaw_error_diff = yaw_error - self.last_yaw_error
        d_term = kd * yaw_error_diff

        # 5. 计算修正量
        correction = (kp * yaw_error) - d_term

        # 6. 统一最大修正量限制
        correction_clamped = max(min(-correction, MAX_CORRECTION), -MAX_CORRECTION)

        # if abs(yaw_error - self.last_yaw_error) > 0.1:
            # self.get_logger().info(f"yaw_error={yaw_error:.2f}，修正量={correction_clamped:.2f}")
        self.last_yaw_error = yaw_error

        return correction_clamped 
    
    def get_adaptive_turn_speed(self, yaw_error_abs: float) -> float:
        """
        分级自适应转向基准速度（核心：大误差快，小误差慢）
        无需减小PID参数，通过基准速度分级实现快慢切换
        """
        if yaw_error_abs > 30:
            return TURN_SPEED_FAST  # type: ignore # 大误差（>30°）：快速转向
        elif yaw_error_abs > 5:
            return TURN_SPEED_MID   # 中误差（5°~30°）：中等速度
        else:
            return TURN_SPEED_SLOW  # 小误差（<5°）：慢速转向，防止超调
        

    def calibrate_heading_at_waypoint(self, target_heading: float) -> Generator[Tuple[float, float], None, bool]:
        self.get_logger().info(
            f"开始航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
        start_time = self.get_clock().now()

        while rclpy.ok():
            heading_error = self.get_heading_error(target_heading)
            # self.get_logger().info(f"heading_error:{heading_error}")

            heading_error_deg = abs(heading_error)
            # 校准达标
            if heading_error_deg <= RTK_HEADING_TOLERANCE:
                self.get_logger().info(f"航向校准完成！误差：{heading_error_deg:.2f}°")
                return True

            # 超时处理
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > HEADING_CALIBRATION_TIMEOUT:
                self.get_logger().warn(f"航向校准超时！误差：{heading_error_deg:.2f}°")
                return True

            # 计算转向速度
            turn_speed = self.get_adaptive_turn_speed(heading_error_deg)

            # 额外的方向修正项（基于航向误差的速度修正）
            correction = self.get_speed_correction(target_heading)

            # 根据修正后的误差计算转向方向
            min_positive_speed = 0.1  # 最小正向速度，防止停滞
            if heading_error > 0:
                # turn_right旋转（根据你的电机控制逻辑调整, 若反向则互换左右速度）
                left_speed = -max(turn_speed - correction, min_positive_speed)
                right_speed = -max(turn_speed - correction, min_positive_speed)
            else:
                # turn_left旋转
                left_speed = max(turn_speed + correction, min_positive_speed)
                right_speed = max(turn_speed + correction, min_positive_speed)
            yield (left_speed, right_speed)

        return False

    def calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        计算两点间的绝对朝向角（方位角），用于初始点→第一个航点的转向目标
        :param lat1: 起点纬度
        :param lon1: 起点经度
        :param lat2: 终点纬度
        :param lon2: 终点经度
        :return: 绝对朝向角（°，归一化到[-180°, 180°]，与IMU航向角格式一致）
        """
        # 转换为弧度
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # 方位角公式计算（0°=正北，90°=正东，180°=正南，270°=正西）
        delta_lon = lon2_rad - lon1_rad
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        bearing_rad = math.atan2(y, x)
        
        # 转换为角度并归一化到[0°, 360°]
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = math.fmod(bearing_deg + 360.0, 360.0)
        # 转换到[-180°, 180°]，与IMU航向角格式统一
        bearing_deg = bearing_deg - 360.0 if bearing_deg > 180.0 else bearing_deg
        return bearing_deg

    def move_to_first_waypoint(self) -> Generator[Tuple[float, float], None, bool]:
        if not self.waypoints:
            self.get_logger().error("无航点数据, 无法执行初始移动")
            return False
        
        first_waypoint = self.waypoints[0]
        self.nav_context["target_waypoint"] = first_waypoint
        self.get_logger().info(f"开始准备移动到第一个航点：{first_waypoint[:2]}")
        
        # ========== 步骤1：等待GPS和IMU初始化（确保获取初始位置和航向） ==========
        # 等待GPS信号（超时5秒）
        start_gps_time = self.get_clock().now()
        while not self.current_gps and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            gps_elapsed = (self.get_clock().now() - start_gps_time).nanoseconds / 1e9
            if gps_elapsed > RTK_CALIBRATION_TIMEOUT:
                self.get_logger().error("获取GPS初始位置超时，无法计算行驶朝向")
                return False
        self.get_logger().info(f"GPS初始位置获取成功：({self.current_gps[0]:.6f}, {self.current_gps[1]:.6f})")
        
        # 等待IMU初始化（超时3秒）
        start_imu_time = self.get_clock().now()
        while not self.imu_initialized and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            imu_elapsed = (self.get_clock().now() - start_imu_time).nanoseconds / 1e9
            if imu_elapsed > IMU_CALIBRATION_TIMEOUT:
                self.get_logger().warn("IMU校准超时, 使用当前偏航角")
                break
        self.get_logger().info(f"IMU初始化完成，当前航向：{self.imu_yaw:.2f}°")
        
        # ========== 步骤2：计算初始点→第一个航点的真实朝向角（核心修复） ==========
        init_lon, init_lat = self.current_gps  # 初始点GPS位置
        first_lon, first_lat, _ = first_waypoint  # 第一个航点经纬度
        target_bearing = self.calculate_bearing(init_lat, init_lon, first_lat, first_lon)
        self.get_logger().info(f"初始点→第一个航点 真实朝向角：{target_bearing:.2f}°")
        
        # ========== 步骤3：航向对准（以真实朝向角为目标） ==========
        target_heading = target_bearing
        self.get_logger().info(f"开始航向对准：目标航向{target_heading:.2f}°")
        calib_generator = self.calibrate_heading_at_waypoint(target_heading)
        heading_aligned = False
        first_straight_heading = 0.0  # 存储对准后的航向角，用于直行纠偏
        while rclpy.ok() and not heading_aligned:
            try:
                left_speed, right_speed = next(calib_generator)
                yield (left_speed, right_speed)
            except StopIteration:
                self.get_logger().info("航向对准完成, 开始直线行驶到第一个航点")
                first_straight_heading = self.imu_yaw  # 记录对准后的实际航向
                heading_aligned = True
                break
            except Exception as e:
                self.get_logger().error(f"航向对准失败: {e}")
                return False
        
        # ========== 步骤4：直线行驶到第一个航点（带航向纠偏） ==========
        # 直行纠偏配置
        last_distance = 0.0
        consecutive_threshold = 5  # 连续达标次数，避免误判
        consecutive_count = 0
        
        while rclpy.ok():
            # 计算到第一个航点的距离
            distance = self.calc_distance_to_waypoint(first_waypoint)
            
            # 打印距离变化（减少日志冗余）
            if abs(last_distance - distance) > 0.1:
                self.get_logger().info(f"到第一个航点距离：{distance:.2f} m")
                last_distance = distance
            
            # 距离达标判断（连续N次达标视为到达）
            if distance < INITIAL_MOVE_TOLERANCE:
                consecutive_count += 1
                self.get_logger().info(f"距离达标, 连续计数：{consecutive_count}/{consecutive_threshold}")
                if consecutive_count >= consecutive_threshold:
                    self.get_logger().info(f"已到达第一个航点距离阈值：{distance:.2f} m")
                    # 到达后精准校准（使用第一个航点的预设航向角）
                    target_waypoint_heading = self.get_path_heading(first_waypoint)
                    self.get_logger().info(f"开始最终航向校准：目标{target_waypoint_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
                    self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_waypoint_heading)
                    self.nav_context["nav_state"] = NavState.WAYPOINT_CALIB
                    while rclpy.ok():
                        try:
                            left_speed, right_speed = next(self.nav_context["calib_generator"])
                            yield (left_speed, right_speed)
                        except StopIteration:
                            self.nav_context["calib_generator"] = None
                            self.nav_context["target_waypoint"] = None
                            return True
                    return True
            else:
                consecutive_count = 0  # 未达标，重置计数
            
            # ========== 直行航向纠偏逻辑（核心） ==========
            # 目标航向：对准后的真实朝向角（first_straight_heading）
            target_heading = first_straight_heading
            # 计算纠偏量（基于真实朝向角的误差）
            correction = self.get_speed_correction(target_heading) * STRAIGHT_PID_SCALE
            # 基础速度（保持原有互为相反数逻辑，仅通过correction微调差速）
            base_speed = LINEAR_SPEED_BASE
            left_speed = -base_speed + correction
            right_speed = base_speed + correction
            
            # 速度限制（双重保障，避免异常值）
            left_speed = max(min(left_speed, SPEED_LIMIT), -SPEED_LIMIT)
            right_speed = max(min(right_speed, SPEED_LIMIT), -SPEED_LIMIT)
            
            # 边界触发时优先执行边界矫正
            if self.is_boundary_triggered:
                left_speed, right_speed = self.get_boundary_correct_speed()
            
            yield (left_speed, right_speed)
        
        return False

    def reset_imu_calibration(self):
        """重置IMU校准状态, 用于在切换模式后重新校准"""
        self.imu_initialized = False
        self.imu_calibration_offset = 0.0
        self.get_logger().info("IMU校准状态已重置")

    def reset_nav_context(self):
        """重置导航状态"""
        self.current_waypoint_idx = 0
        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None,
            "last_distance": 0.0,
            "last_target_heading": 0.0
        }
        # self.get_logger().info("RTK导航状态已重置")

    # ================== 原有控制模式检查 + 多点导航生成器 ==================
    def check_control_mode(self) -> bool:
        """
        检查当前控制模式, 若切换为遥控器模式, 暂停导航
        返回：True=保持RTK模式, False=已切换为遥控器模式
        """
        if self.current_control_mode == ControlMode.REMOTE:
            self.get_logger().info("[ROSNode] 切换到遥控器控制模式, 暂停RTK导航（保存上下文）")
            # 发布停止速度
            stop_speed = Vector3()
            stop_speed.x = 0.0
            stop_speed.y = 0.0
            self.motor_speed_pub.publish(stop_speed)
            self.nav_running = False
            return False
        return True

    def multi_waypoint_nav_generator(self, resume: bool = False):
        """
        多点RTK导航生成器（适配ROS2定时器回调）
        每次yield返回左右轮速度, 支持中断恢复
        """
        # 1. 初始化/恢复导航状态
        if resume:
            current_nav_state = self.nav_context["nav_state"]
            self.get_logger().info(f"从状态{current_nav_state}恢复导航")
            # 新增：强制更新一次到目标航点的距离
            if self.nav_context["target_waypoint"]:
                distance = self.calc_distance_to_waypoint(self.nav_context["target_waypoint"])
                self.nav_context["last_distance"] = distance
        else:
            # 只有导航状态为IDLE时, 才重新初始化初始移动（解决重复进入第一个航点）
            if self.nav_context["nav_state"] == NavState.IDLE:
                current_nav_state = NavState.INITIAL_MOVE
                self.nav_context["nav_state"] = current_nav_state
                self.current_waypoint_idx = 0  # 新导航从0开始
                self.nav_context["target_waypoint"] = None
                self.nav_context["calib_generator"] = None
            else:
                current_nav_state = self.nav_context["nav_state"]
        if self.is_boundary_triggered:
            self.get_logger().info("boundary_triggered!!!")
            self.get_logger().warn("boundary_triggered!!!")

        # 2. 阶段1：初始移动（初始点→第一个航点）- 仅首次启动且非恢复时执行
        if current_nav_state == NavState.INITIAL_MOVE and not resume:
            self.get_logger().info("[ROSNode] 进入初始移动阶段：初始点→第一个航点")
            initial_move_generator = self.move_to_first_waypoint()
            while True:
                # 检查控制模式, 若切换则退出
                if not self.check_control_mode():
                    yield (0.0, 0.0)  # 返回停止速度
                    return
                # 获取初始移动速度
                try:
                    left_speed, right_speed = next(initial_move_generator)
                    if self.is_boundary_triggered:
                        # 触发边界 → 暂停校准, 执行矫正速度
                        left_speed, right_speed = self.get_boundary_correct_speed()
                    yield (left_speed, right_speed)  # 向定时器回调返回速度
                except StopIteration:
                    # 初始移动完成, 切换到第一个航点
                    self.current_waypoint_idx = 1
                    self.current_segment_heading = self.waypoints[0][2]

                    current_nav_state = NavState.WAYPOINT_MOVE
                    self.nav_context["nav_state"] = current_nav_state
                    self.get_logger().info("[ROSNode] 初始移动完成, 进入航点导航阶段")
                    break
                except Exception as e:
                    self.get_logger().error(f"[ROSNav] 初始移动失败：{str(e)}")
                    self.nav_running = False
                    self.publish_stop_speed()
                    yield (0.0, 0.0)
                    return

        # 3. 阶段2：多点航点循环（依次导航到所有航点）
        last_waypoint_idx = self.current_waypoint_idx
        while True:
            # 检查退出条件：控制模式切换/航点全部完成
            if not self.check_control_mode():
                yield (0.0, 0.0)
                return
            if self.current_waypoint_idx >= len(self.waypoints):
                break  # 所有航点导航完成

            # 关键修复：若航点索引切换（新航点）, 强制重置为WAYPOINT_MOVE状态
            if self.current_waypoint_idx != last_waypoint_idx:
                current_nav_state = NavState.WAYPOINT_MOVE
                self.nav_context["nav_state"] = current_nav_state
                self.current_segment_heading = self.waypoints[self.current_waypoint_idx - 1][2]  # 上一个航点=当前路段heading
                self.get_logger().info(f"[直行航向设置] 路段{self.current_waypoint_idx - 1}→{self.current_waypoint_idx}，目标航向（起点航向）：{self.current_segment_heading:.2f}°")

                last_waypoint_idx = self.current_waypoint_idx  # 更新上一个航点索引
                # self.get_logger().info(f"[ROSNode] 检测到航点切换, 强制进入移动阶段（当前航点{self.current_waypoint_idx}）")

            # 3.1 获取目标航点（原有逻辑保留）
            if self.nav_context["target_waypoint"]:
                target_waypoint = self.nav_context["target_waypoint"]
            else:
                target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                print("target_waypoint:",target_waypoint)
                if not target_waypoint:
                    self.get_logger().warn("[ROSNode] 未获取到目标航点, 退出导航")
                    yield (0.0, 0.0)
                    return
                self.nav_context["target_waypoint"] = target_waypoint

            # 3.2 发布当前导航状态
            self.publish_nav_state(current_nav_state)

            # 3.3 子阶段A：航向校准
            if current_nav_state == NavState.WAYPOINT_CALIB:
                calib_generator = self.nav_context["calib_generator"]
                # 若校准生成器不存在, 重新创建
                if not calib_generator:
                    self.get_logger().warn("[ROSNode] 校准生成器不存在, 重新初始化航向校准")
                    target_heading = self.get_path_heading(target_waypoint)
                    calib_generator = self.calibrate_heading_at_waypoint(target_heading)
                    self.nav_context["calib_generator"] = calib_generator

                # 执行航向校准
                try:
                    left_speed, right_speed = next(calib_generator)
                    if self.is_boundary_triggered:
                        # 触发边界 → 暂停校准, 执行矫正速度
                        left_speed, right_speed = self.get_boundary_correct_speed()

                    yield (left_speed, right_speed)
                except StopIteration as e:
                    # 校准完成, 切换到下一个航点
                    calib_result = e.value if hasattr(e, 'value') else False
                    self.get_logger().info(
                        f"[ROSNode] 航点{self.current_waypoint_idx}航向校准完成, 结果：{calib_result}"
                    )
                    # 更新索引和状态
                    self.current_waypoint_idx += 1
                    self.nav_context["calib_generator"] = None
                    self.nav_context["target_waypoint"] = None
                    self.nav_context["last_distance"] = 0.0  # 重置距离缓存
                    self.nav_context["last_target_heading"] = 0.0  # 重置航向缓存

                    # 关键：主动触发一次“获取新航点”的逻辑（提前校验航点2是否存在）
                    new_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                    if new_waypoint:
                        self.get_logger().info(
                            f"[ROSNode] 切换到新航点{self.current_waypoint_idx}, 准备进入移动阶段"
                        )
                    else:
                        self.get_logger().warn(f"[ROSNode] 未获取到航点{self.current_waypoint_idx}")
                    yield (0.0, 0.0)  # 停顿期间停止电机
                    
                except Exception as e:
                    self.get_logger().error(f"[ROSNav] 航向校准失败：{str(e)}")
                    yield (0.0, 0.0)
                    return

            # 3.4 子阶段B：移动到当前航点
            if current_nav_state == NavState.WAYPOINT_MOVE:
                # 关键修复：强制校验并获取目标航点（确保航点2被正确加载）
                if not self.nav_context["target_waypoint"]:
                    target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                    if not target_waypoint:
                        self.get_logger().warn(f"[ROSNode] 未获取到航点{self.current_waypoint_idx}, 退出导航")
                        yield (0.0, 0.0)
                        return
                    self.nav_context["target_waypoint"] = target_waypoint
                # 计算到目标航点的距离和航向
                distance = self.calc_distance_to_waypoint(target_waypoint)
                target_heading = self.get_path_heading(target_waypoint)

                # 打印距离和航向（仅当变化较大时, 减少日志冗余）
                if (abs(self.nav_context["last_distance"] - distance) > 0.1 or
                    abs(self.nav_context["last_target_heading"] - target_heading) > 0.1):
                    self.nav_context["last_distance"] = distance
                    self.nav_context["last_target_heading"] = target_heading
                    self.get_logger().info(
                        f"[ROSNode] 目标航点{self.current_waypoint_idx}：({self.waypoints[self.current_waypoint_idx][0]}, {self.waypoints[self.current_waypoint_idx][1]}), 航向角{self.waypoints[self.current_waypoint_idx][2]}°, 距离{distance:.2f}m, 目标航向{target_heading:.2f}°"
                        # f"[ROSNode] 目标航点{self.current_waypoint_idx}：{self.current_waypoint_idx[0][:2]}距离{distance:.2f}m, 目标航向{target_heading):.2f}°"
                    )

                # 距离未达标：直线行驶+实时纠偏
                if distance >= RTK_WAYPOINT_TOLERANCE:
                    target_heading = self.current_segment_heading

                    correction = self.get_speed_correction(target_heading) * STRAIGHT_PID_SCALE
                    base_speed = LINEAR_SPEED_BASE
                    left_speed = -base_speed + correction
                    right_speed = base_speed + correction
                    left_speed = max(min(left_speed, SPEED_LIMIT), -SPEED_LIMIT)
                    right_speed = max(min(right_speed, SPEED_LIMIT), -SPEED_LIMIT)
                    if self.is_boundary_triggered:
                        # 触发边界 → 暂停校准, 执行矫正速度
                        left_speed, right_speed = self.get_boundary_correct_speed()
                    yield (left_speed, right_speed)
                # 距离达标：切换到航向校准阶段
                else:
                    self.get_logger().info(
                        f"[ROSNode] 已到达航点{self.current_waypoint_idx}距离阈值（{distance:.2f}m ≤ {RTK_WAYPOINT_TOLERANCE}m）"
                    )
                    target_heading = self.get_path_heading(target_waypoint)
                    calib_generator = self.calibrate_heading_at_waypoint(target_heading)
                    self.nav_context["calib_generator"] = calib_generator
                    current_nav_state = NavState.WAYPOINT_CALIB
                    self.nav_context["nav_state"] = current_nav_state
                    yield (0.0, 0.0)  # 到达航点后先停止

        # 4. 所有航点导航完成
        self.get_logger().info("[ROSNode] RTK多点导航全部完成")
        self.nav_context["nav_state"] = NavState.COMPLETED
        self.nav_running = False
        self.publish_stop_speed()  # 停止电机
        self.reset_nav_context()
        yield (0.0, 0.0)  # 最终返回停止速度

    # ================== 原有RTKControlNode核心方法 ==================
    def publish_stop_speed(self):
        """发布停止电机速度指令"""
        stop_speed = Vector3()
        stop_speed.x = 0.0
        stop_speed.y = 0.0
        stop_speed.z = 0.0
        self.motor_speed_pub.publish(stop_speed)

    def publish_nav_state(self, state: NavState):
        """发布当前导航状态"""
        state_msg = String()
        state_msg.data = state if isinstance(state, str) else state.value
        self.nav_state_pub.publish(state_msg)

    def mode_callback(self, msg: String):
        """接收电机节点的控制模式, 更新自身状态"""
        previous_mode = self.current_control_mode
        self.current_control_mode = msg.data
        # 切换到RTK模式时, 重置IMU校准
        if self.current_control_mode == ControlMode.RTK_NAV and previous_mode != ControlMode.RTK_NAV:
            # self.reset_imu_calibration()
            # 新增：强制重置导航生成器和运行状态
            self.multi_waypoint_generator = None
            self.nav_running = False
        # 切换非RTK模式时, 保存导航状态, 停止导航
        if self.current_control_mode != ControlMode.RTK_NAV:
            if self.multi_waypoint_generator:
                self.multi_waypoint_generator = None
            self.nav_running = False
            self.publish_stop_speed()
            # 不重置导航上下文, 保存状态以便后续恢复

    def rtk_timer_callback(self):
        """10Hz定时器回调, 驱动多点导航逻辑"""
        # 仅在RTK导航模式下执行导航逻辑
        if self.current_control_mode == ControlMode.RTK_NAV:
            # 初始化多点导航生成器（首次进入/导航完成后重新初始化, 解决重复进入初始点）
            if not self.multi_waypoint_generator and not self.nav_running:
                # 判断是否需要恢复导航
                resume = (self.nav_context["nav_state"] != NavState.IDLE)
                self.multi_waypoint_generator = self.multi_waypoint_nav_generator(resume=resume)
                self.nav_running = True
                self.publish_nav_state(self.nav_context["nav_state"])
                self.get_logger().info("[ROSNode] 启动/恢复RTK多点导航")

            # 获取多点导航速度并发布
            try:
                if self.multi_waypoint_generator and self.nav_running:
                    left_speed, right_speed = next(self.multi_waypoint_generator)
                    # 构造速度消息并发布
                    speed_msg = Vector3()
                    speed_msg.x = float(left_speed)
                    speed_msg.y = float(right_speed)
                    speed_msg.z = 0.0
                    self.motor_speed_pub.publish(speed_msg)
            except StopIteration:
                # 导航生成器执行完毕（全部航点完成/主动退出）
                self.get_logger().info("[ROSNode] 多点导航生成器执行完毕")
                self.publish_nav_state(NavState.COMPLETED)
                self.multi_waypoint_generator = None
                self.nav_running = False
            except Exception as e:
                self.get_logger().error(f"[ROSNode] RTK多点导航错误：{str(e)}")
                # 发布停止指令
                self.publish_stop_speed()
                # 重置导航状态
                self.multi_waypoint_generator = None
                self.nav_running = False
                self.nav_context["nav_state"] = NavState.IDLE
                self.publish_nav_state(NavState.IDLE)
        else:
            # 非RTK模式：重置生成器
            if self.multi_waypoint_generator:
                self.multi_waypoint_generator = None
                self.nav_running = False
                self.publish_stop_speed()
            pass

# -------------------------- 主函数入口 --------------------------
def main(args=None):
    rclpy.init(args=args)
    rtk_node = RTKNavControlNode()

    try:
        rclpy.spin(rtk_node)
    except KeyboardInterrupt:
        rtk_node.get_logger().info("RTK控制节点收到中断信号, 即将退出")
    except Exception as e:
        rtk_node.get_logger().fatal(f"RTK控制节点异常：{str(e)}")
    finally:
        # 发布停止速度
        rtk_node.publish_stop_speed()
        rtk_node.destroy_node()
        rclpy.shutdown()
        print("RTK控制节点退出完成")

if __name__ == "__main__":
    main()