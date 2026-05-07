#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
from typing import Optional, List, Dict, Tuple, Generator
import time
import rclpy
import re
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3  # 用于发布左右轮速度
from std_msgs.msg import String, UInt8, Float32       # 用于发布控制模式和导航状态
from custom_msgs.msg import WTRTK
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType

# -------------------------- 全局配置与枚举 --------------------------
# 新增：信号漂移过滤配置
GPS_SMOOTH_WINDOW = 5  # GPS经纬度滑动平均窗口大小（帧）

# 滚刷速度
RTK_BRUSH_SPEED = 18.0 #18.0  # 负数表示正常运行速度，与前进反方向

# RTK导航配置
RTK_WAYPOINT_TOLERANCE = 0.1 # 多点导航距离阈值
RTK_HEADING_TOLERANCE = 1.0  # 多点导航角度阈值 0.2
LINEAR_SPEED_BASE = 10.0   # origin 8.0
TURN_SPEED = 1.0 *2     # origin 0.1
INITIAL_MOVE_TOLERANCE = 0.1 #起始点距离阈值
RTK_CALIBRATION_TIMEOUT = 5.0
IMU_CALIBRATION_TIMEOUT = 3.0
HEADING_CALIBRATION_TIMEOUT = 40.0

TURN_SPEED_FAST = 0.8 *2 # 大误差快速转向基准速度
TURN_SPEED_MID = 0.6 *2  # 中误差中等转向基准速度
TURN_SPEED_SLOW = 0.1 *4 # 小误差慢速转向基准速度（防超调）
MAX_CORRECTION = 0.8 *2   # 最大修正量
STRAIGHT_MAX_CORRECTION = 3.0
# straight line speed correction factor
STRAIGHT_PID_SCALE = 2.0  # 2.0
SPEED_LIMIT = 1.5 * LINEAR_SPEED_BASE

#近距离减速/REVERSE阈值
LOW_DISTANCE = 1.3
BACKUP_DURATION = 2.0  # 后退纠正持续时间（秒）
BACKUP_SPEED_SCALE = 0.3  # 后退速度缩放系数（相对于基础速度）
DISTANCE_INCREASE_THRESHOLD = 0.05  # 距离增大触发阈值（米）
DISTANCE_INCREASE_COUNT = 3  # 连续增大次数阈值
ANGLE_ABNORMAL_COUNT = 5  # 连续角度异常次数阈值（触发重新进入角度校准）

# 控制模式（与电机节点保持一致）
class ControlMode:
    REMOTE = "REMOTE"
    NORMAL = "NORMAL"
    AUTO_CLEANING = "AUTO_CLEANING"

# 导航状态枚举
class NavState:
    IDLE = "IDLE"
    INITIAL_MOVE = "INITIAL_MOVE"
    WAYPOINT_MOVE = "WAYPOINT_MOVE"
    WAYPOINT_CALIB = "WAYPOINT_CALIB"
    COMPLETED = "COMPLETED"
    PAUSE = "PAUSE"  # RTK非固定解时暂停导航

class BoundaryCorrectState:
    IDLE = "IDLE"
    TURNING = "TURNING"
    BACKING = "BACKING"
    RETURNING = "RETURNING"
# -------------------------- 合并后的RTK控制+导航节点 --------------------------
class RTKNavControlNode(Node):
    def __init__(self):
        super().__init__('rtk_nav_control_node')

        self.process_percent = 0.0  # 路径文件处理进度百分比
        # ================== 原有RTKNavigator属性 ==================
        self.waypoints: List[Tuple[float, float, float]] = []
        self.current_waypoint_idx = 0
        self.current_gps: Optional[Tuple[float, float]] = [0.0, 0.0]
        self.current_lon = 0.0
        self.current_lat = 0.0
        self.imu_yaw = 0.0
        self.rtk_install_offset = -90.0    #-90.0(old)  # RTK安装偏移角度

         # 例如：天线在车体中心前方0.31米，左侧0.2米（根据实际安装位置调整）
        self.antenna_offset_front = 0.2517   # 前向偏移（+：天线在车体前，-：在后）
        self.antenna_offset_left = -0.19625   # 左向偏移（-：天线在车体左，+：在右）

        # 新增：出仓点基准缓存与偏移量
        self.base_loading_waypoint = None  # 基准出仓点（首次接收的出仓点），格式：(lon, lat, heading)
        self.waypoint_offset = {
            "lon_offset": 0.0,    # 经纬度偏移量（°）
            "lat_offset": 0.0,
            "heading_offset": 0.0 # 航向角偏移量（°）
        }
        self.laste_state = None  # 保存上一个  "status": "START"状态
        self.offset_calculated = False  # 偏移量是否已计算（避免重复计算）

        self.imu_initialized = False
        self.imu_calibration_offset = 0.0
        self.last_yaw_error = 0.0
        self.integral_yaw = 0.0
        self.current_control_mode = ControlMode.NORMAL
        self.last_state = None  # 电机状态（用于监听HOLD切换）

        # Sensor 
        self.front_left = None # test, None origin
        self.front_right = None
        self.mid_left = None
        self.mid_right = None
        self.back_left = None
        self.back_right = None

        self.correct_speed_scale = 0.4 # boundary correct speed scale
        self.last_waypoint_cache = None
        self.has_printed_coincide_log = False
        # 新增：跨文件缓存（保存上一个文件的最后一个航点，用于计算跨文件偏角）
        self.cross_file_last_waypoint = None  # 格式：(lon, lat, heading)

        # 出仓点（从 /unloading_gps 话题获取）
        self.loading_waypoint = None  # 格式：(lon, lat, heading)
        self.return_to_loading_added = False  # 出仓点是否已追加标志

        # 边界矫正状态机
        self.boundary_correct_state = BoundaryCorrectState.IDLE
        self.boundary_correct_start_time = None
        self.boundary_correct_direction = None  # 'left' 或 'right'
        self.boundary_correct_locked = False    # 锁定标志，锁定后不受传感器条件影响
        self.BOUNDARY_TURN_DURATION = 1.0    # 偏转持续时间（秒）
        self.BOUNDARY_BACK_DURATION = 4.0    # 后退持续时间（秒）
        self.BOUNDARY_RETURN_DURATION = 1.0  # 反向偏转退回持续时间（秒）

        self.gps_cache = []
        # 距离历史缓存（用于异常检测）
        self.distance_cache = []
        # 目标航向角滑动缓存
        self.heading_cache = []
        # IMU偏航角滑动缓存
        self.imu_yaw_cache = []

        self.last_valid_heading = None  # 存储距离≥0.5m时最后一次计算的航向
        self.bad_error_counter = 0   #yaw_error绝对值大于15度的次数
        
        self.brush_start_idx = None  # 滚刷开启的航点索引
        self.brush_stop_idx = None    # 滚刷关闭的航点索引
        self.brush_active = False     # 滚刷是否激活
        # self.is_boundary_triggered = False # test, False origin
        # 定义参数描述：bool类型, 名称：is_boundary_triggered, 默认值：False
        # boundary_param_desc = ParameterDescriptor(
        #     type=ParameterType.PARAMETER_BOOL,
        #     description='手动强制开启/关闭边界触发, True=触发矫正, False=强制关闭边界矫正(屏蔽传感器)'
        # )
        # 声明参数 + 绑定到类成员变量
        # self.declare_parameter('is_boundary_triggered', True, boundary_param_desc)
        # 读取初始值（程序启动时的默认值）
        self.is_boundary_triggered = None
        # self.get_parameter('is_boundary_triggered').value

        # ========== ✅ 必须添加：参数回调函数, 监听参数修改事件 ==========
        # self.add_on_set_parameters_callback(self.update_boundary_parameter)


        # 声明RTK路径参数
        self.declare_parameter("rtk_path_file", "/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/three_path_20260129_144149.txt")
        self.rtk_path_file = self.get_parameter("rtk_path_file").value
        self.path_dir = os.path.dirname(self.rtk_path_file)
        # self.rtk_path_file = self.declare_parameter(
        #     'rtk_path_file',
        #     # "/home/forlinx/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"
        #     "/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/cleaning_path_20251121_173149.txt"
        # )
        
        # self.path_dir = os.path.dirname(self.rtk_path_file)  # 获取路径文件所在目录


        # 导航上下文
        self.nav_context = {
            "nav_state": NavState.IDLE,
            "target_waypoint": None,
            "calib_generator": None,
            "last_distance": 0.0,
            "last_target_heading": 0.0,
            "pre_pause_state": None,
            "angle_abnormal_count": 0,  # 连续角度异常计数器
            "is_angle_recalib": False,  # 是否是角度异常后的重新校准
        }

        # ================== 原有RTKControlNode属性 ==================
        self.rate = self.create_rate(10)  # 10Hz, origin 4
        self.nav_generator: Optional[Generator] = None
        self.nav_running = False
        self.multi_waypoint_generator = None  # 多点导航生成器

        # ROS2发布器/订阅器
        self.motor_speed_pub = self.create_publisher(Vector3, "/rtk/motor_speed", 10)
        self.nav_state_pub = self.create_publisher(String, "/rtk/nav_state", 10)
        self.imu_heading_pub = self.create_publisher(Float32, "/imu_heading", 10)
        self.car_center_gps_pub = self.create_publisher(NavSatFix, "car_center_gps", 10)

        self.control_mode_sub = self.create_subscription(String, "/control/mode", self.mode_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.heading_sub = self.create_subscription(WTRTK, '/wtrtk_data', self.heading_callback, 10)
        self.io_data_rtk_sub = self.create_subscription(UInt8, '/io_data', self.io_data_rtk_callback, 10)
        self.unloading_gps_sub = self.create_subscription(Vector3, '/unloading_gps', self.unloading_gps_callback, 10)
        self.state_sub = self.create_subscription(String, "/motor/state", self.state_callback, 10)
        self.route_change_sub = self.create_subscription(String, "/rtk/route_change", self.route_change_callback, 10)


        # 定时器（10Hz驱动导航逻辑）
        self.rtk_nav_timer = self.create_timer(0.1, self.rtk_timer_callback)

        # 加载航点
        self.load_rtk_path()
        # 加载初始路径文件
        self.load_waypoints_from_file(self.rtk_path_file)
    

    # def load_waypoints_from_file(self, file_path: str) -> bool:
    #     """从文件加载航点数据（新增跨文件缓存逻辑）"""
    #     try:
    #         # 核心：加载新文件前，保存当前文件的最后一个航点到跨文件缓存
    #         if self.waypoints:  # 若当前有已加载的航点（即切换文件场景）
    #             self.cross_file_last_waypoint = self.waypoints[-1]  # 保存最后一个航点
    #             self.get_logger().info(f"[跨文件缓存] 保存上一个文件最后一个航点：{self.cross_file_last_waypoint}")
            
    #         # 重置当前文件航点数据（原有逻辑保留）
    #         self.waypoints = []
    #         self.current_waypoint_idx = 0
            
    #         with open(file_path, 'r', encoding='utf-8') as f:
    #             lines = f.readlines()[1:]  # 跳过表头
    #             for line in lines:
    #                 line = line.strip()
    #                 if not line or line.startswith('#'):
    #                     continue
    #                 seq, lon, lat, heading_deg = line.split(',')
    #                 self.waypoints.append((float(lon), float(lat), float(heading_deg)))
            
    #         # 初始化当前文件的上一个航点缓存（原有逻辑优化）
    #         if self.waypoints:
    #             self.last_waypoint_cache = self.waypoints[0]
    #             self.get_logger().info(f"[当前文件缓存] 首次加载初始化：上一个航点为{self.waypoints[0]}")
    #         else:
    #             self.last_waypoint_cache = None
    #             self.cross_file_last_waypoint = None
    #             self.get_logger().warn("[RTKNav] 未加载到有效航点，缓存重置")
            
    #         # 原有日志和返回逻辑保留
    #         self.get_logger().info(f"[RTKNav] 成功加载路径文件: {file_path}, 共 {len(self.waypoints)} 个航点")
    #         self.rtk_path_file = file_path  # 更新当前路径文件
    #         return True
            
    #     except Exception as e:
    #         self.get_logger().error(f"[RTKNav] 加载路径文件失败: {e}")
    #         return False
    def load_waypoints_from_file(self, file_path: str) -> bool:
        """从文件加载航点数据（正确顺序：先区分场景，再执行缓存逻辑）"""
        try:
            # ================== 第一步：先区分「首次加载」和「文件切换」==================
            # 定义首次加载的判断条件：程序启动后第一次加载文件（用一个标记位控制）
            if not hasattr(self, 'is_first_file_load'):
                self.is_first_file_load = True  # 首次进入时初始化标记位
            
            # ================== 第二步：仅文件切换时，执行缓存逻辑==================
            if not self.is_first_file_load and self.waypoints:
                # 只有「文件切换」（非首次加载），才保存上一文件最后一个航点
                self.cross_file_last_waypoint = self.waypoints[-1]
                self.get_logger().info(f"[跨文件缓存] 保存上一个文件最后一个航点：{self.cross_file_last_waypoint}")
            else:
                # 首次加载：强制重置 cross_file_last_waypoint 为 None
                self.cross_file_last_waypoint = None
                self.get_logger().info(f"[首次加载] 不执行跨文件缓存，cross_file_last_waypoint 置空")
            
            # ================== 第三步：重置当前文件航点数据（原有逻辑）==================
            self.waypoints = []
            self.current_waypoint_idx = 0
            
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()[1:]  # 跳过表头
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    
                    # 检查是否为注释行（包含#）
                    if line.startswith('#'):
                        comment = line[1:].strip().lower()
                        if 'start' in comment:
                            self.brush_start_idx = len(self.waypoints)
                            self.get_logger().info(f"[RTKNav] 检测到#start标记，滚刷将在航点{self.brush_start_idx}开启")
                            if self.brush_start_idx == 0:
                                self.brush_active = True
                        elif 'stop' in comment:
                            self.brush_stop_idx = len(self.waypoints)
                            self.get_logger().info(f"[RTKNav] 检测到#stop标记，滚刷将在航点{self.brush_stop_idx}关闭")
                        continue
                    
                    parts = line.split(',')
                    if len(parts) < 4:
                        self.get_logger().warn(f"[RTKNav] 航点行格式错误，跳过: {line}")
                        continue
                    
                    seq, raw_lon, raw_lat, raw_heading = parts[0], parts[1], parts[2], parts[3]
                    self.waypoints.append((float(raw_lon), float(raw_lat), float(raw_heading)))
                    # # 核心修改：对当前航点应用出仓点偏移修正
                    # # 核心修复：强制转换为浮点数（之前未转换，导致字符串类型）
                    # raw_lon = float(raw_lon.strip())
                    # raw_lat = float(raw_lat.strip())
                    # raw_heading = float(raw_heading.strip())
                    # corrected_lon, corrected_lat, corrected_heading = self.correct_waypoint_by_offset(
                    #     raw_lon, raw_lat, raw_heading
                    # )
                    
                    # # 替换原有逻辑：添加修正后的航点
                    # self.waypoints.append((corrected_lon, corrected_lat, corrected_heading))
            
            # 验证航点有效性
            if not self.waypoints:
                self.last_waypoint_cache = None
                self.cross_file_last_waypoint = None
                self.get_logger().warn("[RTKNav] 未加载到有效航点，缓存重置")
                return False
            
            first_waypoint = self.waypoints[0]
            self.get_logger().info(f"[当前文件航点验证] 第一个航点已加载：{first_waypoint}（总航点数：{len(self.waypoints)}）")
            
            # ================== 第四步：初始化 last_waypoint_cache（基于场景）==================
            if self.cross_file_last_waypoint is not None:
                # 场景1：文件切换 - 参考上一文件最后一个航点
                self.last_waypoint_cache = self.cross_file_last_waypoint
                self.get_logger().info(f"[当前文件缓存] 跨文件衔接：当前第一个航点[{first_waypoint}] 的前置参考航点为[{self.cross_file_last_waypoint}]")
                self.cross_file_last_waypoint = None  # 清空缓存，防止重复使用
            else:
                # 场景2：首次加载 - 参考当前文件第一个航点自身
                self.last_waypoint_cache = first_waypoint
                self.get_logger().info(f"[当前文件缓存] 首次加载初始化：当前第一个航点[{first_waypoint}] 作为初始参考航点")
            
            # ================== 第五步：更新标记位（首次加载后改为 False）==================
            self.is_first_file_load = False
            
            # 原有日志和返回逻辑
            self.get_logger().info(f"[RTKNav] 成功加载路径文件: {file_path}, 共 {len(self.waypoints)} 个航点（第一个航点已就绪）")
            self.rtk_path_file = file_path
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
        边界触发时的实时矫正速度计算（状态机版本）
        逻辑：向左偏转(1s) → 后退2s → 反向偏转退回(1s)
        """
        
        base_correct_speed = self.correct_speed_scale * LINEAR_SPEED_BASE
        left_speed = 0.0
        right_speed = 0.0

        current_time = time.time()

        # 检查是否需要启动新的边界矫正序列（只在非锁定状态下检测）
        if self.boundary_correct_state == BoundaryCorrectState.IDLE and not self.boundary_correct_locked:
            # if self.front_left and not self.front_right:
            if self.mid_left and not self.mid_right:
                self.boundary_correct_state = BoundaryCorrectState.TURNING
                self.boundary_correct_start_time = current_time
                self.boundary_correct_direction = 'left'
                self.boundary_correct_locked = True  # 锁定，不受后续传感器变化影响
                self.get_logger().info("[RTKNav] 边界矫正启动：向左偏转 [锁定]")
            # elif self.front_right and not self.front_left:
            elif self.mid_right and not self.mid_left:
                self.boundary_correct_state = BoundaryCorrectState.TURNING
                self.boundary_correct_start_time = current_time
                self.boundary_correct_direction = 'right'
                self.boundary_correct_locked = True  # 锁定，不受后续传感器变化影响
                self.get_logger().info("[RTKNav] 边界矫正启动：向右偏转 [锁定]")
            elif self.mid_left and self.mid_right:
                self.boundary_correct_state = BoundaryCorrectState.BACKING
                self.boundary_correct_start_time = current_time
                self.boundary_correct_locked = True  # 锁定，不受后续传感器变化影响
                self.get_logger().info("[RTKNav] 边界矫正启动：正前方均触发,执行后退 [锁定]")

        # 状态机处理
        if self.boundary_correct_state == BoundaryCorrectState.TURNING:
            elapsed = current_time - self.boundary_correct_start_time
            if self.boundary_correct_direction == 'left':
                left_speed = base_correct_speed
                right_speed = base_correct_speed
            else:
                left_speed = -base_correct_speed
                right_speed = -base_correct_speed

            if elapsed >= self.BOUNDARY_TURN_DURATION:
                self.boundary_correct_state = BoundaryCorrectState.BACKING
                self.boundary_correct_start_time = current_time
                self.get_logger().info(f"[RTKNav] 边界矫正阶段：后退{self.BOUNDARY_BACK_DURATION}s")

        elif self.boundary_correct_state == BoundaryCorrectState.BACKING:
            elapsed = current_time - self.boundary_correct_start_time
            left_speed = base_correct_speed
            right_speed = -base_correct_speed

            if elapsed >= self.BOUNDARY_BACK_DURATION:
                self.boundary_correct_state = BoundaryCorrectState.RETURNING
                self.boundary_correct_start_time = current_time
                self.get_logger().info(f"[RTKNav] 边界矫正阶段：反向偏转退回{self.BOUNDARY_RETURN_DURATION}s")

        elif self.boundary_correct_state == BoundaryCorrectState.RETURNING:
            elapsed = current_time - self.boundary_correct_start_time
            if self.boundary_correct_direction == 'left':
                left_speed = -base_correct_speed
                right_speed = -base_correct_speed
            elif self.boundary_correct_direction == 'right':
                left_speed = base_correct_speed
                right_speed = base_correct_speed
            else:
                left_speed = 0.0
                right_speed = 0.0

            if elapsed >= self.BOUNDARY_RETURN_DURATION:
                self.boundary_correct_state = BoundaryCorrectState.IDLE
                self.boundary_correct_start_time = None
                self.boundary_correct_direction = None
                self.boundary_correct_locked = False  # 解除锁定
                self.get_logger().info("[RTKNav] 边界矫正完成，恢复正常 [解锁]")

        self.get_logger().info(f"[RTKNav] 边界矫正状态: {self.boundary_correct_state}, 速度：左轮={left_speed:.2f},右轮={right_speed:.2f}")
        return (left_speed, right_speed)
    
    def io_data_rtk_callback(self, msg: UInt8):

        # 按位或结果存储传感器状态
        # self.sensors_status = self.front_left | self.front_right<<1 | self.mid_left<<2 | self.mid_right<<3 | self.back_left<<4 | self.back_right<<5 
        # self.sensors_status = ~self.sensors_status & 0x3F  # 取反并保留6位
        # self.get_logger().info(f"[RTKNav] 收到IO数据: {msg.data}")
        # 位0 (1<<0 = 0x01)：前左
        self.front_left = (msg.data & 0x01) == 0x00
        self.front_right = (msg.data & 0x02) == 0x00
        self.mid_left = (msg.data & 0x04) == 0x00
        self.mid_right = (msg.data & 0x08) == 0x00
        self.back_left = (msg.data & 0x10) == 0x00
        self.back_right = (msg.data & 0x20) == 0x00
        self.sensors_status = self.front_left | self.front_right<<1 | self.mid_left<<2 | self.mid_right<<3 | self.back_left<<4 | self.back_right<<5 
        self.sensors_status = ~self.sensors_status & 0x3F  # 取反并保留6位
        #开启后传感器误触发，注释
        # if self.mid_left or self.mid_right:
        #     self.is_boundary_triggered = True
        # else:
        #     self.is_boundary_triggered = False

        # self.get_logger().info(f"IO状态: {msg.data}, front_left={self.front_left}, front_right={self.front_right}, "
        # f"mid_left={self.mid_left}, mid_right={self.mid_right}, "
        # f"back_left={self.back_left}, back_right={self.back_right}"
        # )
    # def unloading_gps_callback(self, msg: Vector3):
    #     loading_lon = msg.x
    #     loading_lat = msg.y
    #     heading = msg.z
    #     self.loading_waypoint = (loading_lon, loading_lat, heading)
    #     self.get_logger().info(f"[RTKNav] 收到出仓GPS坐标: 经度={loading_lon}, 纬度={loading_lat}")
    def unloading_gps_callback(self, msg: Vector3):
        loading_lon = msg.x
        loading_lat = msg.y
        loading_heading = msg.z
        current_loading = (loading_lon, loading_lat, loading_heading)
        self.loading_waypoint = current_loading
        self.get_logger().info(f"[RTKNav] 收到出仓GPS坐标: 经度={loading_lon:.6f}, 纬度={loading_lat:.6f}, 航向={loading_heading:.2f}°")
        
        # # 步骤1：首次接收出仓点，缓存为基准点（不计算偏移）
        # if self.base_loading_waypoint is None:
        #     self.base_loading_waypoint = current_loading
        #     self.get_logger().info(f"[RTKNav] 缓存基准出仓点：{self.base_loading_waypoint}")
        #     self.offset_calculated = False
        #     return
        
        # # 步骤2：非首次接收，计算当前出仓点与基准点的偏移量
        # if not self.offset_calculated:
        #     base_lon, base_lat, base_heading = self.base_loading_waypoint
            
        #     # 2.1 计算经纬度偏移（直接差值，单位：°）
        #     lon_offset = loading_lon - base_lon
        #     lat_offset = loading_lat - base_lat
            
        #     # 2.2 计算航向角偏移（归一化到[-180°, 180°]）
        #     heading_offset = loading_heading - base_heading
        #     heading_offset = math.fmod(heading_offset + 180.0, 360.0) - 180.0
            
        #     # 2.3 保存偏移量
        #     self.waypoint_offset = {
        #         "lon_offset": lon_offset,
        #         "lat_offset": lat_offset,
        #         "heading_offset": heading_offset
        #     }
        #     self.offset_calculated = True
        #     self.get_logger().info(
        #         f"[RTKNav] 计算出仓点偏移量：经度{lon_offset:.6f}°, 纬度{lat_offset:.6f}°, 航向{heading_offset:.2f}°"
        #     )
        #     # 在偏移量计算后检查是否过大
        #     max_offset_deg = 0.00001  # 最大允许偏移（约1米）
        #     if abs(lon_offset) > max_offset_deg or abs(lat_offset) > max_offset_deg or abs(heading_offset) > 1.0:
        #         self.get_logger().warn(f"[RTKNav] 出仓点偏移过大（超过{max_offset_deg}°），请检查出仓点准确性")
        #         self.waypoint_offset = {
        #             "lon_offset": 0.0,
        #             "lat_offset": 0.0,
        #             "heading_offset": 0.0
        #         }
        #         self.offset_calculated = False
        #         self.get_logger().info("[RTKNav] 已重置偏移量，后续航点将不进行修正")
    
    def correct_waypoint_by_offset(self, raw_lon: float, raw_lat: float, raw_heading: float) -> Tuple[float, float, float]:
        """
        根据出仓点偏移量，修正航点的经纬度和航向角
        :param raw_lon: 原始航点经度（°）
        :param raw_lat: 原始航点纬度（°）
        :param raw_heading: 原始航点航向角（°）
        :return: 修正后的（lon, lat, heading）
        """
        # 未计算偏移量时，直接返回原始航点
        if not self.offset_calculated:
            return (raw_lon, raw_lat, raw_heading)
        
        lon_offset = self.waypoint_offset["lon_offset"]
        lat_offset = self.waypoint_offset["lat_offset"]
        heading_offset = self.waypoint_offset["heading_offset"]
        
        # 步骤1：修正航向角（直接叠加偏移，归一化）
        corrected_heading = raw_heading + heading_offset
        corrected_heading = math.fmod(corrected_heading + 180.0, 360.0) - 180.0
        
        # 步骤2：修正经纬度（考虑航向角对偏移的影响，参考天线→车体中心的计算逻辑）
        EARTH_RADIUS = 6378137.0  # 地球半径（米）
        # 将航点航向角转换为弧度（用于计算偏移方向）
        heading_rad = math.radians(raw_heading)
        
        # 2.1 将经纬度偏移（°）转换为米级偏移（基于地球半径）
        # 纬度1°≈111319.9米，经度1°≈111319.9×cos(lat) 米
        lat_rad = math.radians(raw_lat)
        lon_offset_m = lon_offset * 111319.9 * math.cos(lat_rad)  # 经度偏移→米
        lat_offset_m = lat_offset * 111319.9  # 纬度偏移→米
        
        # 2.2 基于航点航向角，计算米级偏移在北向（N）和东向（E）的分量
        # 逻辑：出仓点的偏移是相对于基准点的，需叠加到航点的北向和东向
        delta_n = lat_offset_m * math.cos(heading_rad) - lon_offset_m * math.sin(heading_rad)
        delta_e = lat_offset_m * math.sin(heading_rad) + lon_offset_m * math.cos(heading_rad)
        
        # 2.3 将米级偏移转换为经纬度偏移（弧度→角度）
        delta_lat_rad = delta_n / EARTH_RADIUS
        delta_lon_rad = delta_e / (EARTH_RADIUS * math.cos(lat_rad))
        delta_lat_deg = math.degrees(delta_lat_rad)
        delta_lon_deg = math.degrees(delta_lon_rad)
        
        # 2.4 计算修正后的经纬度
        corrected_lon = raw_lon + delta_lon_deg
        corrected_lat = raw_lat + delta_lat_deg
        
        self.get_logger().debug(
            f"[航点修正] 原始({raw_lon:.6f}, {raw_lat:.6f}, {raw_heading:.2f}°) → "
            f"修正后({corrected_lon:.6f}, {corrected_lat:.6f}, {corrected_heading:.2f}°)"
        )
        return (corrected_lon, corrected_lat, corrected_heading)
    def get_next_path_file(self) -> Optional[str]:
        """获取下一个路径文件（按文件名序号从小到大排序）"""
        try:
            # 1. 获取目录下所有符合命名规则的路径文件
            file_pattern = re.compile(r'^\d{3}_.*\.txt$')
            # file_pattern = re.compile(r'.*_\d{8}_\d{6}\.txt')
            all_files = [f for f in os.listdir(self.path_dir) if file_pattern.match(f)]
            
            if not all_files:
                self.get_logger().warn("[RTKNav] 路径目录下未找到符合规则的路径文件")
                return None
            
            # 2. 按文件名最前面的3位数字序号排序（提取如002, 004等）
            def extract_sequence_number(filename: str) -> int:
                match = re.search(r'^(\d{3})_', filename)
                return int(match.group(1)) if match else 0
            
            all_files.sort(key=extract_sequence_number)
            total_files = len(all_files)
            current_file = os.path.basename(self.rtk_path_file)
            
            # 3. 找到当前文件的索引
            try:
                current_idx = all_files.index(current_file)
            except ValueError:
                self.get_logger().warn(f"[RTKNav] 当前文件 {current_file} 不在路径目录中, 使用第一个文件")
                progress_num = 1
                progress_percent = round((progress_num / total_files) * 100, 1)
                self.get_logger().info(f"[RTKNav] 路径文件进度：{progress_num}/{total_files}, {progress_percent}%")
                return os.path.join(self.path_dir, all_files[0])
            
            # 4. 计算并输出进度（当前文件索引+1 为已执行/待执行的序号）
            current_progress = current_idx + 1
            progress_percent = round((current_progress / total_files) * 100, 1)
            self.process_percent = progress_percent
            self.get_logger().info(f"[RTKNav] 路径文件进度：{current_progress}/{total_files}, {progress_percent}%")
            
            # 5. 最后一个文件时结束循环（不再返回新文件）
            if current_idx >= total_files - 1:
                self.get_logger().info(f"[RTKNav] 已执行到最后一个路径文件（{current_file}）, 执行返回")
                return None
            
            # 6. 获取下一个文件（非最后一个时）
            next_idx = current_idx + 1
            next_file = all_files[next_idx]
            next_seq = extract_sequence_number(next_file)
            self.get_logger().info(f"[RTKNav] 准备切换到下一个路径文件：{next_file} (序号: {next_seq:03d})")
            
            return os.path.join(self.path_dir, next_file)
            
        except Exception as e:
            self.get_logger().error(f"[RTKNav] 获取下一个路径文件失败: {e}")
            return None
    # def get_next_path_file(self) -> Optional[str]:
        # """获取下一个路径文件（按文件名时间戳排序）"""
        # try:
        #     # 1. 获取目录下所有符合命名规则的路径文件
        #     file_pattern = re.compile(r'.*_\d{8}_\d{6}\.txt')
        #     all_files = [f for f in os.listdir(self.path_dir) if file_pattern.match(f)]
            
        #     if not all_files:
        #         self.get_logger().warn("[RTKNav] 路径目录下未找到符合规则的路径文件")
        #         return None
            
        #     # 2. 按文件名中的时间戳排序（提取YYYYMMDD_HHMMSS部分）
        #     def extract_timestamp(filename: str) -> str:
        #         match = re.search(r'(\d{8}_\d{6})', filename)
        #         return match.group(1) if match else ''
            
        #     all_files.sort(key=extract_timestamp)
        #     total_files = len(all_files)  # 总文件数
        #     current_file = os.path.basename(self.rtk_path_file)
            
        #     # 3. 找到当前文件的索引
        #     try:
        #         current_idx = all_files.index(current_file)
        #     except ValueError:
        #         self.get_logger().warn(f"[RTKNav] 当前文件 {current_file} 不在路径目录中, 使用第一个文件")
        #         # 首次使用第一个文件, 进度 1/总数量
        #         progress_num = 1
        #         progress_percent = round((progress_num / total_files) * 100, 1)
        #         self.get_logger().info(f"[RTKNav] 路径文件进度：{progress_num}/{total_files}, {progress_percent}%")
        #         return os.path.join(self.path_dir, all_files[0])
            
        #     # 4. 计算并输出进度（当前文件索引+1 为已执行/待执行的序号）
        #     current_progress = current_idx + 1
        #     progress_percent = round((current_progress / total_files) * 100, 1)
        #     # 更新进度百分比
        #     self.process_percent = progress_percent
        #     self.get_logger().info(f"[RTKNav] 路径文件进度：{current_progress}/{total_files}, {progress_percent}%")
            
        #     # 5. 最后一个文件时结束循环（不再返回新文件）
        #     if current_idx >= total_files - 1:
        #         self.get_logger().info(f"[RTKNav] 已执行到最后一个路径文件（{current_file}）, 执行返回")
        #         return None
            
        #     # 6. 获取下一个文件（非最后一个时）
        #     next_idx = current_idx + 1
        #     next_file = all_files[next_idx]
        #     self.get_logger().info(f"[RTKNav] 准备切换到下一个路径文件：{next_file}")
            
        #     return os.path.join(self.path_dir, next_file)
            
        # except Exception as e:
        #     self.get_logger().error(f"[RTKNav] 获取下一个路径文件失败: {e}")
        #     return None
    

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
                    
                    # 检查是否为注释行（包含#）
                    if line.startswith('#'):
                        comment = line[1:].strip().lower()
                        if 'start' in comment:
                            # 记录滚刷开启的下一个航点索引
                            self.brush_start_idx = len(self.waypoints)
                            self.get_logger().info(f"[RTKNav] 检测到#start标记，滚刷将在航点{self.brush_start_idx}开启")
                            if self.brush_start_idx == 0:
                                self.brush_active = True
                        elif 'stop' in comment:
                            # 记录滚刷关闭的下一个航点索引
                            self.brush_stop_idx = len(self.waypoints)
                            self.get_logger().info(f"[RTKNav] 检测到#stop标记，滚刷将在航点{self.brush_stop_idx}关闭")
                        continue
                    
                    seq, lon, lat, heading_deg = line.split(',')
                    self.waypoints.append((float(lon), float(lat), float(heading_deg)))
                    # self.get_logger().info(f"成功加载RTK航点{len(self.waypoints)}个")
                    # return True
                    # seq, raw_lon, raw_lat, raw_heading = line.split(',')
                    # # 核心修复：转换为浮点数
                    # raw_lon = float(raw_lon.strip())
                    # raw_lat = float(raw_lat.strip())
                    # raw_heading = float(raw_heading.strip())
                    # # 添加航点偏移修正
                    # corrected_lon, corrected_lat, corrected_heading = self.correct_waypoint_by_offset(
                    #     raw_lon, raw_lat, raw_heading
                    # )
                    # self.waypoints.append((corrected_lon, corrected_lat, corrected_heading))
            # self.get_logger().info(f"成功加载RTK航点{len(self.waypoints)}个")
            if self.brush_start_idx is not None:
                self.get_logger().info(f"[RTKNav] 滚刷开启航点索引: {self.brush_start_idx}")
            if self.brush_stop_idx is not None:
                self.get_logger().info(f"[RTKNav] 滚刷关闭航点索引: {self.brush_stop_idx}")
            return True
        except Exception as e:
            self.get_logger().error(f"解析RTK文件失败：{str(e)}")
            return False
    def reset_base_loading_waypoint(self):
        """重置基准出仓点（用于更换任务场景）"""
        self.base_loading_waypoint = None
        self.offset_calculated = False
        self.get_logger().info("[RTKNav] 已重置基准出仓点，下次接收将重新缓存")

    def gps_callback(self, msg: NavSatFix) -> None:
        # 初始化上一状态变量（首次调用时创建）
        if not hasattr(self, 'last_gps_status'):
            self.last_gps_status = -1
            
        if msg.status.status < 0:
            self.get_logger().warn("GPS信号无效")
            # 更新上一状态为无效
            self.last_gps_status = -1
            return
        
        status_map = {0: "未定位", 1: "单点", 2: "差分", 5: "RTK Float", 4: "RTK Fixed"}
        # 仅当状态改变时才打印日志
        if msg.status.status in status_map and msg.status.status != self.last_gps_status:
            self.get_logger().info(f"GPS状态：{status_map[msg.status.status]}")
            # 更新上一状态为当前状态
            self.last_gps_status = msg.status.status
        
        # ========== RTK非固定解时暂停导航 ==========
        current_gps_status = msg.status.status
        is_fixed = (current_gps_status == 4)  # RTK Fixed
        
        if not is_fixed:
            if hasattr(self, 'nav_context') and self.nav_context["nav_state"] not in [NavState.IDLE, NavState.PAUSE, NavState.COMPLETED]:
                # 保存暂停前的状态
                self.nav_context["pre_pause_state"] = self.nav_context["nav_state"]
                self.nav_context["brush_active"] = self.brush_active  # 保存滚刷状态
                self.get_logger().warn(f"[RTK状态] 当前状态：{status_map.get(current_gps_status, '未知')}，非固定解，暂停导航（保存状态：{self.nav_context['pre_pause_state']}）")
                self.nav_context["nav_state"] = NavState.PAUSE
                self.nav_running = False
                stop_speed = Vector3()
                self.publish_stop_speed()
        else:
            if hasattr(self, 'nav_context') and self.nav_context["nav_state"] == NavState.PAUSE:
                self.get_logger().info("[RTK状态] 恢复RTK固定解，自动恢复导航")
                self.nav_context["nav_state"] = self.nav_context["pre_pause_state"]
                self.brush_active = self.nav_context.get("brush_active", False)  # 恢复滚刷状态
                if self.brush_active:
                    self.publish_brush_speed(RTK_BRUSH_SPEED)
                else:
                    self.publish_brush_speed(0.0)
                self.nav_running = True
        
        # self.current_gps = (msg.longitude, msg.latitude)
        # self.current_lon = msg.longitude
        # self.current_lat = msg.latitude
            # ========== 新增：经纬度过滤逻辑 ==========
        raw_lon = msg.longitude
        raw_lat = msg.latitude
        
        # 1. 突变检测：与上一个有效坐标的差值超过阈值则视为异常
        if self.current_gps and self.current_gps != [0.0, 0.0]:
            lon_diff = abs(raw_lon - self.current_gps[0])
            lat_diff = abs(raw_lat - self.current_gps[1])
            # if lon_diff > GPS_CHANGE_THRESHOLD or lat_diff > GPS_CHANGE_THRESHOLD:
            #     self.get_logger().warn(f"[GPS过滤] 坐标突变（lon_diff={lon_diff:.6f}, lat_diff={lat_diff:.6f}），使用历史平滑值")
            #     # 异常时不更新坐标，直接返回
            #     return
        
        # 2. 滑动平均平滑：保留最近N帧坐标取平均
        self.gps_cache.append((raw_lon, raw_lat))
        if len(self.gps_cache) > GPS_SMOOTH_WINDOW:
            self.gps_cache.pop(0)
        # 计算平滑后的经纬度
        smooth_lon = sum([x[0] for x in self.gps_cache]) / len(self.gps_cache)
        smooth_lat = sum([x[1] for x in self.gps_cache]) / len(self.gps_cache)
        
        # 更新当前GPS坐标（使用过滤后的值）
        self.current_gps = (smooth_lon, smooth_lat)
        self.current_lon = smooth_lon
        self.current_lat = smooth_lat
        self.get_logger().debug(f"当前天线坐标：经度={smooth_lon:.6f}, 纬度={smooth_lat:.6f}")

        EARTH_RADIUS = 6378137.0       # 地球半径（米），WGS84椭球长半轴
        # 1. 将航向角转换为弧度（注意：imu_yaw是车体朝向，需确认角度定义：0°为北，顺时针增加）
        heading_rad = math.radians(self.imu_yaw)
        
        # 2. 计算天线相对于车体中心的北向（N）、东向（E）偏移量（米）
        # 车体朝向为heading_rad，天线在前则北向偏移为正，在左则东向偏移为负（根据坐标系调整）
        # 核心公式：
        # 北向偏移 = 前向偏移 * cos(航向角) - 左向偏移 * sin(航向角)
        # 东向偏移 = 前向偏移 * sin(航向角) + 左向偏移 * cos(航向角)
        delta_n = self.antenna_offset_front * math.cos(heading_rad) - self.antenna_offset_left * math.sin(heading_rad)
        delta_e = self.antenna_offset_front * math.sin(heading_rad) + self.antenna_offset_left * math.cos(heading_rad)
        
        # 3. 将米级偏移转换为经纬度偏移（弧度）
        # 纬度偏移：delta_lat = delta_n / 地球半径
        # 经度偏移：delta_lon = delta_e / (地球半径 * cos(纬度弧度))
        lat_rad = math.radians(self.current_lat)
        delta_lat_rad = delta_n / EARTH_RADIUS
        delta_lon_rad = delta_e / (EARTH_RADIUS * math.cos(lat_rad))
        
        # 4. 转换为角度并计算车体中心坐标（天线坐标 - 偏移量 = 车体中心坐标）
        car_lat = self.current_lat - math.degrees(delta_lat_rad)
        car_lon = self.current_lon - math.degrees(delta_lon_rad)

        # 发布车体中心GPS
        car_gps_msg = NavSatFix()
        car_gps_msg.header = msg.header
        car_gps_msg.status = msg.status
        car_gps_msg.latitude = car_lat
        car_gps_msg.longitude = car_lon
        car_gps_msg.altitude = msg.altitude  # 高度可根据实际偏移调整
        self.car_center_gps_pub.publish(car_gps_msg)

        self.current_lat = car_lat
        self.current_lon = car_lon
        self.current_gps = (car_lon, car_lat)
        self.get_logger().debug(f"当前车体中心坐标：经度={car_lon:.6f}, 纬度={car_lat:.6f}")


    def heading_callback(self, msg: WTRTK) -> None:
        ins_heading_deg = msg.ins_heading
        self.imu_yaw = ins_heading_deg  + self.rtk_install_offset # + x degree
        self.imu_yaw = math.fmod(self.imu_yaw + 180.0, 360.0) - 180.0
        self.imu_initialized = True
        imu_msg= Float32()
        imu_msg.data = self.imu_yaw
        self.imu_heading_pub.publish(imu_msg)

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
                if hasattr(self, 'loading_waypoint') and self.loading_waypoint is not None and not self.return_to_loading_added:
                    self.get_logger().info(f"[RTKNav] 开始执行返回出仓点：{self.loading_waypoint}")
                    # 将出仓点追加到航点列表末尾
                    self.waypoints.append(self.loading_waypoint)
                    # 手动更新索引，指向新增的出仓点
                    self.current_waypoint_idx = idx
                    # 标记出仓点已追加
                    self.return_to_loading_added = True
                    self.get_logger().info(f"[RTKNav] 出仓点追加成功，当前航点索引：{self.current_waypoint_idx}，总航点数：{len(self.waypoints)}")
                    # 返回出仓点作为新的目标航点
                    return self.loading_waypoint
                # self.current_control_mode = ControlMode.NORMAL
                return None
        
        # 返回当前目标航点
        return self.waypoints[idx]

    # def calc_distance_to_waypoint(self, waypoint: Tuple[float, float, float]) -> float:
    #     if not self.current_gps:
    #         return float('inf')

    #     R = 6371000.0  # 地球半径（米）
    #     lon1, lat1 = self.current_gps
    #     lon2, lat2, _ = waypoint

    #     # 转换为弧度
    #     lon1_rad = math.radians(lon1)
    #     lat1_rad = math.radians(lat1)
    #     lon2_rad = math.radians(lon2)
    #     lat2_rad = math.radians(lat2)

    #     # Haversine公式计算距离
    #     delta_lon = lon2_rad - lon1_rad
    #     delta_lat = lat2_rad - lat1_rad

    #     a = math.sin(delta_lat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2)** 2
    #     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    #     return R * c
    def latlon_to_utm(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        经纬度转UTM平面坐标（米）
        :param lat: 纬度
        :param lon: 经度
        :return: (utm_x, utm_y) 平面坐标，单位米
        """
        # WGS84椭球参数
        a = 6378137.0  # 长半轴
        f = 1 / 298.257223563  # 扁率
        e_sq = 2 * f - f ** 2  # 第一偏心率平方

        # 计算UTM投影带号（6度带）
        zone = int((lon + 180) / 6) + 1
        # 中央子午线经度
        lon0 = (zone - 1) * 6 - 180 + 3

        # 转换为弧度
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lon0_rad = math.radians(lon0)

        # 子午线收敛角计算
        n = a / math.sqrt(1 - e_sq * math.sin(lat_rad) ** 2)
        t = math.tan(lat_rad) ** 2
        c = e_sq * math.cos(lat_rad) ** 2 / (1 - e_sq)
        a1 = math.cos(lat_rad) * (lon_rad - lon0_rad)

        # 计算UTM x坐标（东向）
        x = n * (a1 + (1 - t + c) * a1 ** 3 / 6 + (5 - 18 * t + t ** 2 + 72 * c - 58 * e_sq) * a1 ** 5 / 120)
        # 计算UTM y坐标（北向）
        m = a * ((1 - e_sq / 4 - 3 * e_sq ** 2 / 64 - 5 * e_sq ** 3 / 256) * lat_rad -
                (3 * e_sq / 8 + 3 * e_sq ** 2 / 32 + 45 * e_sq ** 3 / 1024) * math.sin(2 * lat_rad) +
                (15 * e_sq ** 2 / 256 + 45 * e_sq ** 3 / 1024) * math.sin(4 * lat_rad) -
                (35 * e_sq ** 3 / 3072) * math.sin(6 * lat_rad))
        y = m + n * math.tan(lat_rad) * (a1 ** 2 / 2 + (5 - t + 9 * c + 4 * c ** 2) * a1 ** 4 / 24 +
                                        (61 - 58 * t + t ** 2 + 600 * c - 330 * e_sq) * a1 ** 6 / 720)

        # 东向偏移500km，避免负数
        x += 500000.0
        # 南半球偏移10000km（本方案默认北半球，若需支持南半球可添加判断）
        if lat < 0:
            y += 10000000.0

        return x, y
    def calc_distance_to_waypoint(self, waypoint: Tuple[float, float, float]) -> float:
        if not self.current_gps:
            return float('inf')
        
        # 获取当前位置和目标航点的经纬度
        lon1, lat1 = self.current_gps
        lon2, lat2, _ = waypoint

        # 转换为UTM平面坐标
        x1, y1 = self.latlon_to_utm(lat1, lon1)
        x2, y2 = self.latlon_to_utm(lat2, lon2)
        raw_distance = math.hypot(x2 - x1, y2 - y1)

        # 计算平面直线距离（米）
        distance = math.hypot(x2 - x1, y2 - y1)
        return distance

        # ========== 新增：距离过滤逻辑 ==========
        # 1. 突变检测：与历史距离差值超过阈值则视为异常
        # smooth_distance = raw_distance
        # if self.distance_cache:
        #     last_distance = self.distance_cache[-1]
        #     distance_diff = abs(raw_distance - last_distance)
        #     if distance_diff > DISTANCE_CHANGE_THRESHOLD:
        #         self.get_logger().warn(f"[距离过滤] 距离突变（当前{raw_distance:.2f}m，上一帧{last_distance:.2f}m，差值{distance_diff:.2f}m），使用历史平滑值")
        #         # 异常时使用历史平滑值
        #         smooth_distance = last_distance
        #     else:
        #         # 正常时加入缓存做滑动平均
        #         self.distance_cache.append(raw_distance)
        #         if len(self.distance_cache) > GPS_SMOOTH_WINDOW:
        #             self.distance_cache.pop(0)
        #         # 滑动平均平滑
        #         smooth_distance = sum(self.distance_cache) / len(self.distance_cache)
        # else:
        #     # 首次缓存初始化
        #     self.distance_cache.append(raw_distance)
        
        # return smooth_distance
    def smooth_angle(self, raw_angle: float, cache: list, window_size: int, change_threshold: float) -> float:
        """
        角度平滑与突变过滤通用函数（支持航向角）
        :param raw_angle: 原始角度（°），已归一化到[-180°, 180°]
        :param cache: 角度缓存列表
        :param window_size: 滑动窗口大小
        :param change_threshold: 突变阈值（°）
        :return: 过滤后的平滑角度
        """
        # 1. 突变检测：与上一个平滑角度差值超过阈值则视为异常
        smooth_angle = raw_angle
        if cache:
            last_smooth = cache[-1]
            # 计算角度差值（处理-180°和180°的连续问题）
            angle_diff = abs(raw_angle - last_smooth)
            angle_diff = min(angle_diff, 360 - angle_diff)  # 取最小角度差
            if angle_diff > change_threshold:
                self.get_logger().warn(f"[角度过滤] 角度突变（原始{raw_angle:.2f}°，上一帧{last_smooth:.2f}°，差值{angle_diff:.2f}°），使用历史值")
                smooth_angle = last_smooth
            else:
                # 正常时加入缓存做滑动平均
                cache.append(raw_angle)
                if len(cache) > window_size:
                    cache.pop(0)
                # 滑动平均平滑
                smooth_angle = sum(cache) / len(cache)
        else:
            # 首次缓存初始化
            cache.append(raw_angle)
        
        # 确保平滑后角度仍归一化到[-180°, 180°]
        smooth_angle = math.fmod(smooth_angle + 180.0, 360.0) - 180.0
        return smooth_angle
    def get_path_heading(self, waypoint: Tuple[float, float, float]) -> float:
        """获取目标航点的路径航向角（转换为rad并归一化, 与IMU基准一致）"""
        # 修正：航点航向角是绝对角度, 需叠加IMU校准偏移（让路径航向与IMU基准对齐）
        heading_deg = waypoint[2] + self.imu_calibration_offset
        heading_rad = math.radians(heading_deg)
        # return math.fmod(heading_rad + math.pi, 2 * math.pi) - math.pi
        return math.fmod(heading_deg + 180.0, 360.0) - 180.0

    # def get_heading_error(self, target_heading: float) -> float:
    #     """计算当前航向与目标航向的误差（归一化到[-180°, 180°]，单位：度）"""
    #     imu_yaw_normalized = ((self.imu_yaw + 180.0) % 360.0) - 180.0
    #     target_heading_normalized = ((target_heading + 180.0) % 360.0) - 180.0
    #     heading_error = target_heading_normalized - imu_yaw_normalized
    #     heading_error = ((heading_error + 180.0) % 360.0) - 180.0
    #     return heading_error
    def get_heading_error(self, target_heading: float) -> float:
        """计算当前航向与目标航向的误差（归一化到[-180°, 180°]，单位：度）"""
        # 将两个角度都归一化到 [-180°, 180°) 范围处理环形最短路径
        imu_normalized = self.imu_yaw % 360.0
        if imu_normalized >= 180.0:
            imu_normalized -= 360.0
        
        target_normalized = target_heading % 360.0
        if target_normalized >= 180.0:
            target_normalized -= 360.0
        
        # 计算误差
        heading_error = target_normalized - imu_normalized
        
        # 归一化到 [-180°, 180°]，确保不会超出范围
        while heading_error > 180.0:
            heading_error -= 360.0
        while heading_error <= -180.0:
            heading_error += 360.0
        
        return heading_error
    def get_ring_angle_diff(self, angle1: float, angle2: float) -> float:
        """
        计算两个归一化到[-180°,180°]角度的物理最短路径差值（绝对值）
        解决-180°和180°连续问题，如-170°和170°的差值为20°而非340°
        :param angle1: 角度1（°），已归一化[-180,180]
        :param angle2: 角度2（°），已归一化[-180,180]
        :return: 物理最短路径角度差（°），范围[0,180]
        """
        diff = abs(angle1 - angle2)
        # 取环形最短路径：差值>180°则取360°-差值
        return min(diff, 360.0 - diff)

    def straight_get_speed_correction(self, target_heading: float) -> float:
        """计算对称纠正量（优化PID，减少长距离累积偏移）"""
        yaw_error = target_heading
        # 强制归一化误差到[-180°, 180°]，防止边界情况
        yaw_error = ((yaw_error + 180.0) % 360.0) - 180.0
        yaw_error_abs = abs(yaw_error)

        # 异常过滤：yaw_error绝对值大于30度视为异常，不处理（增大阈值让更多误差能被修正）
        if yaw_error_abs > 30.0:
            self.get_logger().warn(f"直线：yaw_error={yaw_error:.2f}°（>{30.0}°），跳过处理")
            self.last_yaw_error = 0.0
            self.integral_yaw = 0.0
            return 0.0

        # # 1. 误差死区：缩小死区，及时响应小误差
        if yaw_error_abs < 0.1:
            self.last_yaw_error = 0.0
            self.integral_yaw = 0.0
            return 0.0

        # # 2. KP参数优化：大幅增强
        if yaw_error_abs > 10:
            kp = 0.15  # 大误差：大幅增强
        elif yaw_error_abs > 5:
            kp = 0.15  # 中误差：增强
        else:
            kp = 0.12  # 小误差：增强

        # 3. KD参数：减小阻尼，避免过度抑制
        kd = 0.04

        # 4. 积分项：增强积分作用
        ki = 0.008

        # 5. 误差差分计算
        yaw_error_diff = yaw_error - self.last_yaw_error
        d_term = kd * yaw_error_diff

        # 6. 积分累积（小误差时积分）
        if yaw_error_abs < 5.0:
            self.integral_yaw += yaw_error
        else:
            self.integral_yaw = 0.0
        self.integral_yaw = max(min(self.integral_yaw, 30.0), -30.0)
        i_term = ki * self.integral_yaw

        # 7. PID修正量计算
        correction = (kp * yaw_error) - d_term + i_term

        # 8. 最大修正量限制
        correction_clamped = -max(min(correction, STRAIGHT_MAX_CORRECTION), -STRAIGHT_MAX_CORRECTION)

        # 日志输出
        if abs(yaw_error - self.last_yaw_error) > 0.1:
            self.get_logger().info(f"直线：yaw_error={yaw_error:.2f}，修正量={correction_clamped:.2f}")
        
        self.last_yaw_error = yaw_error
        return correction_clamped
    
    def get_speed_correction(self, target_heading: float) -> float:
        """计算对称纠正量（优化PID，减少长距离累积偏移）"""
        yaw_error = self.get_heading_error(target_heading)
        yaw_error_abs = abs(yaw_error)

        # 1. 误差死区：缩小死区，及时响应
        if yaw_error_abs < 0.1:
            self.last_yaw_error = 0.0
            return 0.0

        # 2. KP参数优化：增强大误差时的修正能力
        if yaw_error_abs > 60:
            kp = 0.15  # 大误差：大幅增强
        elif yaw_error_abs > 20:
            kp = 0.10  # 中误差：增强
        else:
            kp = 0.05  # 小误差：精准修正

        # 3. KD参数：增强阻尼
        kd = 0.08

        # 4. 误差差分计算
        yaw_error_diff = yaw_error - self.last_yaw_error
        d_term = kd * yaw_error_diff

        # 5. 修正量计算
        correction = (kp * yaw_error) + d_term

        # 6. 最大修正量限制
        correction_clamped = -max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        # 日志输出
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
        # self.get_logger().info(
            # f"开始航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")

        while rclpy.ok():
            start_time = self.get_clock().now()
            heading_error = self.get_heading_error(target_heading)
            # ========== 核心修复：归一化航向误差到[-180°, 180°]，取最短路径 ==========
            heading_error = math.fmod(heading_error + 180.0, 360.0) - 180.0
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
            min_positive_speed = 0.1  # 最小正向速度，防止停滞 origin 0.1
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
        raw_bearing = bearing_deg - 360.0 if bearing_deg > 180.0 else bearing_deg
        # 强制归一化到[-180°, 180°]，防止边界情况
        raw_bearing = ((raw_bearing + 180.0) % 360.0) - 180.0
        return raw_bearing

    def move_to_first_waypoint(self) -> Generator[Tuple[float, float], None, bool]:
        if not self.waypoints:
            self.get_logger().error("无航点数据, 无法执行初始移动")
            yield (0.0, 0.0)  # 占位，使函数成为合法生成器
            return
        
        first_waypoint = self.waypoints[0]
        self.nav_context["target_waypoint"] = first_waypoint
        self.get_logger().info(f"开始准备移动到第一个航点：{first_waypoint[:2]}")
        
        # ========== 步骤1：等待GPS和IMU初始化 ==========
        start_gps_time = self.get_clock().now()
        while not self.current_gps and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            gps_elapsed = (self.get_clock().now() - start_gps_time).nanoseconds / 1e9
            if gps_elapsed > RTK_CALIBRATION_TIMEOUT:
                self.get_logger().error("获取GPS初始位置超时，无法计算行驶朝向")
                return False
        self.get_logger().info(f"GPS初始位置获取成功：({self.current_gps[0]:.6f}, {self.current_gps[1]:.6f})")
        
        start_imu_time = self.get_clock().now()
        while not self.imu_initialized and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            imu_elapsed = (self.get_clock().now() - start_imu_time).nanoseconds / 1e9
            if imu_elapsed > IMU_CALIBRATION_TIMEOUT:
                self.get_logger().warn("IMU校准超时, 使用当前偏航角")
                break
        self.get_logger().info(f"IMU初始化完成，当前航向：{self.imu_yaw:.2f}°")
        
        # ========== 步骤2：初始航向对准（仅首次转向，后续用实时朝向角） ==========
        init_lon, init_lat = self.current_gps
        first_lon, first_lat, _ = first_waypoint
        initial_bearing = self.calculate_bearing(init_lat, init_lon, first_lat, first_lon)
        self.get_logger().info(f"初始点→第一个航点 初始朝向角：{initial_bearing:.2f}°")
        
        target_heading = initial_bearing
        self.get_logger().info(f"开始初始航向对准：目标航向{target_heading:.2f}°")
        calib_generator = self.calibrate_heading_at_waypoint(target_heading)
        heading_aligned = False
        last_heading = None
        while rclpy.ok() and not heading_aligned:
            try:
                left_speed, right_speed = next(calib_generator)
                yield (left_speed, right_speed)
            except StopIteration:
                self.get_logger().info("初始航向对准完成, 开始实时纠偏直线行驶")
                heading_aligned = True
                last_heading = (target_heading + 180) % 360 - 180
                self.get_logger().info(f"记录初始旋转完成航向角：{last_heading:.2f}°")
                break
            except Exception as e:
                self.get_logger().error(f"初始航向对准失败: {e}")
                return False
        
        # ========== 步骤3：实时RTK角度矫正 + 直线行驶 ==========
        last_distance = 0.0
        consecutive_threshold = 5
        consecutive_count = 0
        
        while rclpy.ok():
            # 实时获取当前RTK位置和目标航点经纬度
            current_lon, current_lat = self.current_gps
            target_lon, target_lat, _ = first_waypoint
            
            # 核心：每帧实时计算当前位置→目标航点的朝向角（替代固定航向）
            real_time_heading = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
            # 新增：角度平滑（取当前与上一帧的平均值，减少波动）
            # real_time_heading = (real_time_heading * 0.7 + self.nav_context["last_target_heading"] * 0.3)
            self.nav_context["last_target_heading"] = real_time_heading
            # 计算实时距离
            distance = self.calc_distance_to_waypoint(first_waypoint)
            
            # 打印距离和实时朝向角（减少冗余）
            if abs(last_distance - distance) > 0.1:
                self.get_logger().info(f"到第一个航点距离：{distance:.2f} m, 实时朝向角：{real_time_heading:.2f}°")
                last_distance = distance
            
            # 距离达标判断
            if distance < INITIAL_MOVE_TOLERANCE:
                consecutive_count += 1
                self.get_logger().info(f"距离达标, 连续计数：{consecutive_count}/{consecutive_threshold}")
                if consecutive_count >= consecutive_threshold:
                    self.get_logger().info(f"已到达第一个航点距离阈值：{distance:.2f} m")
                    # 到达后用航点预设航向校准（为下一段导航准备）
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
                consecutive_count = 0
            # ========== 新增：距离<LOW_DISTANCE米时线性减速 ==========
            current_base_speed = LINEAR_SPEED_BASE  # 距离≥1米，正常速度
            speed_scale = 1.0
            if distance < LOW_DISTANCE:
                # 线性减速：距离LOW_DISTANCE米时速度=BASE的50%，距离0.1米时速度=BASE的10%
                speed_scale = max(0.2, distance/LOW_DISTANCE * 0.7)  # 0.1~0.5之间动态缩放
                current_base_speed = LINEAR_SPEED_BASE * speed_scale
            #  # 1. 先判断距离，决定是否更新实时航向
            # if distance >= 0.3:
            #     # 距离足够远时，正常计算实时航向，并更新缓存
            #     real_time_heading = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
            #     self.last_valid_heading = real_time_heading  # 缓存最新的有效航向
            # else:
            #     # 距离<0.5m时，使用缓存的最后一次有效航向（不再实时计算）
            #     if self.last_valid_heading is not None:
            #         real_time_heading = self.last_valid_heading  # 复用之前的航向
            #         self.get_logger().info(
            #             f"[近距离固定航向] 距离{distance:.2f}m < 0.5m，固定目标航向为{real_time_heading:.2f}°"
            #         )
            #     else:
            #         # 极端情况：首次进入近距离就无缓存，降级使用实时计算（避免程序报错）
            #         real_time_heading = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
            #         self.get_logger().warning(
            #             f"[航向缓存异常] 距离{distance:.2f}m < 0.5m但无有效航向缓存，临时使用实时计算航向{real_time_heading:.2f}°"
            #         )
            # ========== 实时角度纠偏逻辑 ==========
            target_heading = real_time_heading # 使用实时朝向角
            # target_heading = real_time_heading - last_heading  # 使用实时朝向角与初始旋转完成的朝向角夹角
            # target_heading = (target_heading + 180 ) % 360 - 180 + self.imu_yaw
            yaw_error = self.get_heading_error(target_heading)
            # 连续角度异常检测
            yaw_error_abs = abs(yaw_error)
            if yaw_error_abs > 15.0:
                self.nav_context["angle_abnormal_count"] = self.nav_context.get("angle_abnormal_count", 0) + 1
                abnormal_count = self.nav_context["angle_abnormal_count"]
                self.get_logger().warn(f"[初始移动角度异常] yaw_error={yaw_error:.2f}°（>15.0°），连续异常{abnormal_count}次")
                
                if abnormal_count >= ANGLE_ABNORMAL_COUNT:
                    self.get_logger().warn(f"[初始移动角度异常] 连续{abnormal_count}次角度异常，重置状态为INITIAL_MOVE重新校准")
                    self.nav_context["angle_abnormal_count"] = 0
                    self.nav_context["nav_state"] = NavState.INITIAL_MOVE
                    current_nav_state = NavState.INITIAL_MOVE
                    self.nav_context["nav_state"] = current_nav_state
                    self.nav_context["is_angle_recalib"] = True  # 标记为角度异常重置
                    self.nav_context["target_waypoint"] = None
                    self.nav_context["calib_generator"] = None
                    yield (0.0, 0.0)
                    return
                else:
                    yield (0.0, 0.0)
                    continue
            else:
                self.nav_context["angle_abnormal_count"] = 0
            
            # 计算修正量（使用真实偏差角）
            correction = self.straight_get_speed_correction(yaw_error) * STRAIGHT_PID_SCALE
            self.get_logger().info(f"yaw_error={yaw_error:.2f}°, correction={correction:.2f}")
            # base_speed = LINEAR_SPEED_BASE
            last_left_speed = None
            last_right_speed = None
            left_speed = -current_base_speed + correction
            right_speed = current_base_speed + correction
            if left_speed != last_left_speed or right_speed != last_right_speed:
                self.get_logger().debug(f"初始移动：base_speed={current_base_speed:.2f}, correction={correction:.2f}, left_speed={left_speed:.2f}, right_speed={right_speed:.2f}")
            last_left_speed = left_speed
            last_right_speed = right_speed
            # 速度限制
            left_speed = max(min(left_speed, SPEED_LIMIT), -SPEED_LIMIT)
            right_speed = max(min(right_speed, SPEED_LIMIT), -SPEED_LIMIT)
            
            # 边界触发优先
            if self.is_boundary_triggered or self.boundary_correct_locked:
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
            "last_target_heading": 0.0,
            "pre_pause_state": None,
            "angle_abnormal_count": 0,
            "is_angle_recalib": False,
            "brush_active": False,  # 滚刷是否激活
        }
        self.return_to_loading_added = False  # 重置出仓点追加标志
        self.brush_active = False  # 重置滚刷状态
            # 重置后退纠正相关计数器
        self.last_distance_for_backup = 0.0
        self.distance_increase_count = 0
            # ========== 新增：重置所有信号过滤缓存 ==========
        if hasattr(self, 'gps_cache'):
            self.gps_cache.clear()
        if hasattr(self, 'distance_cache'):
            self.distance_cache.clear()
        if hasattr(self, 'heading_cache'):
            self.heading_cache.clear()
        # 后退纠正计数器重置
        if hasattr(self, 'last_distance_for_backup'):
            self.last_distance_for_backup = 0.0
        if hasattr(self, 'distance_increase_count'):
            self.distance_increase_count = 0
        # self.get_logger().info("RTK导航状态已重置")

    # ================== 原有控制模式检查 + 多点导航生成器 ==================
    def check_control_mode(self) -> bool:
        """
        检查当前控制模式, 若切换为遥控器模式, 暂停导航
        返回：True=保持RTK模式, False=已切换为遥控器模式
        """
        # if self.current_control_mode == ControlMode.REMOTE:
        if self.current_control_mode != ControlMode.AUTO_CLEANING:
            self.get_logger().info("[ROSNode] 切换到遥控器控制模式, 暂停RTK导航（保存上下文）")
            # 发布停止速度
            stop_speed = Vector3()
            stop_speed.x = 0.0
            stop_speed.y = 0.0
            stop_speed.z = 0.0  # brush speed
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
            
            # 处理RTK非固定解导致的暂停状态恢复
            if current_nav_state == NavState.PAUSE:
                pre_pause_state = self.nav_context.get("pre_pause_state", NavState.WAYPOINT_MOVE)
                self.get_logger().info(f"[ROSNode] 从RTK暂停状态恢复，恢复到：{pre_pause_state}")
                current_nav_state = pre_pause_state
                self.nav_context["nav_state"] = current_nav_state
                
                # 如果恢复的是校准状态，需要重新初始化校准生成器
                if current_nav_state == NavState.WAYPOINT_CALIB:
                    target_waypoint = self.nav_context.get("target_waypoint")
                    if target_waypoint:
                        target_heading = self.get_path_heading(target_waypoint)
                        self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_heading)
                        self.get_logger().info(f"重新初始化航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
            
            self.get_logger().info(f"从状态{current_nav_state}恢复导航")
            # 新增：恢复校准状态时，重新初始化校准生成器
            if current_nav_state == NavState.WAYPOINT_CALIB:
                target_waypoint = self.nav_context["target_waypoint"]
                if target_waypoint:
                    target_heading = self.get_path_heading(target_waypoint)
                    # 重新创建校准生成器（重置超时计时和误差计算）
                    self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_heading)
                    self.get_logger().info(f"重新初始化航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
            
            # 恢复导航时检查滚刷控制（处理暂停恢复场景）
            self.check_and_control_brush()

            # 新增：强制更新一次到目标航点的距离
            if self.nav_context["target_waypoint"]:
                distance = self.calc_distance_to_waypoint(self.nav_context["target_waypoint"])
                self.nav_context["last_distance"] = distance
        else:
            # 只有导航状态为IDLE时, 才重新初始化初始移动（解决重复进入第一个航点）
            if self.nav_context["nav_state"] == NavState.IDLE:
                current_nav_state = NavState.INITIAL_MOVE
                self.nav_context["nav_state"] = current_nav_state
                self.current_waypoint_idx = 0
                self.nav_context["target_waypoint"] = None
                self.nav_context["calib_generator"] = None
            else:
                current_nav_state = self.nav_context["nav_state"]
        
        # 首次启动导航时也检查滚刷控制
        self.publish_nav_state(current_nav_state)
        self.check_and_control_brush()
        if self.is_boundary_triggered:
            self.get_logger().info("boundary_triggered!!!")
            self.get_logger().warn("boundary_triggered!!!")

        # 2. 阶段1：初始移动（初始点→第一个航点）- 仅首次启动且非恢复时执行
        if current_nav_state == NavState.INITIAL_MOVE:
            self.get_logger().info("[ROSNode] 进入初始移动阶段：初始点→第一个航点")
            initial_move_generator = self.move_to_first_waypoint()
            # 新增1：判断生成器是否有效（空航点时返回False，无效）
            if not initial_move_generator:
                self.get_logger().fatal("[ROSNode] 初始移动生成器初始化失败，无航点数据，终止导航")
                self.nav_running = False
                self.publish_stop_speed()
                self.reset_nav_context()
                yield (0.0, 0.0)
                return
            while True:
                # 检查控制模式, 若切换则退出
                if not self.check_control_mode():
                    yield (0.0, 0.0)  # 返回停止速度
                    return
                # 获取初始移动速度
                try:
                    left_speed, right_speed = next(initial_move_generator)
                    if self.is_boundary_triggered or self.boundary_correct_locked:
                        # 触发边界 → 暂停校准, 执行矫正速度
                        left_speed, right_speed = self.get_boundary_correct_speed()
                    yield (left_speed, right_speed)
                except StopIteration:
                    is_recalib = self.nav_context.get("is_angle_recalib", False)
                    if is_recalib:
                        self.get_logger().info("[ROSNode] 角度异常重置后继续前往航点0")
                        self.nav_context["is_angle_recalib"] = False  # 重置标志
                    else:
                        if self.current_waypoint_idx >= len(self.waypoints) and len(self.waypoints) == 0:
                            self.get_logger().error("[ROSNode] 初始移动StopIteration：无航点数据，终止导航")
                            self.nav_running = False
                            self.publish_stop_speed()
                            self.reset_nav_context()
                            yield (0.0, 0.0)
                            return
                        self.current_waypoint_idx = 1
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
        
        # 3. 阶段2：多点航点循环（含实时角度矫正）
        last_waypoint_idx = self.current_waypoint_idx
        while True:
            # 检查退出条件：控制模式切换/航点全部完成
            if not self.check_control_mode():
                yield (0.0, 0.0)
                return
            # if self.current_waypoint_idx >= len(self.waypoints):
            #     break  # 所有航点导航完成
                # 核心修改：先获取目标航点，无航点时再终止循环（触发get_target_waypoint出仓点逻辑）
            target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
            if not target_waypoint:
                break  # 只有get_target_waypoint返回None时，才真正终止循环
            self.nav_context["target_waypoint"] = target_waypoint  # 赋值目标航点
            

            # 关键修复：若航点索引切换（新航点）, 强制重置为WAYPOINT_MOVE状态
            # 但如果当前是航向校准阶段（is_angle_recalib），则不重置状态
            is_calib_state = self.nav_context.get("nav_state") == NavState.WAYPOINT_CALIB
            if self.current_waypoint_idx != last_waypoint_idx and not is_calib_state:
                current_nav_state = NavState.WAYPOINT_MOVE
                self.nav_context["nav_state"] = current_nav_state
                self.get_logger().info(f"[航点切换] 进入路段{self.current_waypoint_idx - 1}→{self.current_waypoint_idx}")
                
                # 核心修正：处理「新文件第一个航点（索引0）+ 跨文件缓存」的场景
                if self.current_waypoint_idx == 0 and self.cross_file_last_waypoint is not None:
                    # 跨文件场景下，索引0的上一航点是cross_file_last_waypoint
                    self.last_waypoint_cache = self.cross_file_last_waypoint
                    self.get_logger().info(f"[航点切换] 跨文件场景 - 缓存上一个文件最后一个航点：{self.last_waypoint_cache}（作为当前航点0的前置参考）")
                    self.has_printed_coincide_log = False
                # 原有逻辑：处理当前文件内的航点切换（索引≥1）
                elif self.current_waypoint_idx - 1 >= 0 and len(self.waypoints) > self.current_waypoint_idx - 1:
                    self.last_waypoint_cache = self.waypoints[self.current_waypoint_idx - 1]
                    self.get_logger().info(f"[航点切换] 缓存当前文件上一个航点{self.current_waypoint_idx - 1}：{self.last_waypoint_cache}")
                    self.has_printed_coincide_log = False
                # else:
                #     # self.last_waypoint_cache = None
                #     if not self.has_printed_coincide_log:
                #         self.get_logger().warn("[航点切换] 无有效上一个航点，无法缓存")
                #         self.has_printed_coincide_log = True
                
                last_waypoint_idx = self.current_waypoint_idx
            
            # # 获取目标航点
            # if self.nav_context["target_waypoint"]:
            #     target_waypoint = self.nav_context["target_waypoint"]
            # else:
            #     target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
            #     if not target_waypoint:
            #         self.get_logger().warn("[ROSNode] 未获取到目标航点, 退出导航")
            #         yield (0.0, 0.0)
            #         return
            #     self.nav_context["target_waypoint"] = target_waypoint
            
            # 发布导航状态
            self.publish_nav_state(current_nav_state)
            
            # 子阶段A：航向校准（到达航点后执行）
            if current_nav_state == NavState.WAYPOINT_CALIB:
                calib_generator = self.nav_context["calib_generator"]
                if not calib_generator:
                    self.get_logger().warn("[ROSNode] 校准生成器不存在, 重新初始化")
                    target_heading = self.get_path_heading(target_waypoint)

                    calib_generator = self.calibrate_heading_at_waypoint(target_heading)
                    self.nav_context["calib_generator"] = calib_generator
                try:
                    # self.get_logger().info(f"{self.current_waypoint_idx}:航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
                    left_speed, right_speed = next(calib_generator)
                    if self.is_boundary_triggered or self.boundary_correct_locked:
                        # 触发边界 → 暂停校准, 执行矫正速度
                        left_speed, right_speed = self.get_boundary_correct_speed()
                    yield (left_speed, right_speed)
                except StopIteration as e:
                    calib_result = e.value if hasattr(e, 'value') else False
                    
                    is_recalib = self.nav_context.get("is_angle_recalib", False)
                    
                    if is_recalib:
                        # 角度异常后的重新校准：校准完成后继续前往当前航点
                        self.get_logger().info(
                            f"[ROSNode] 角度异常重新校准完成，继续前往航点{self.current_waypoint_idx}"
                        )
                        self.nav_context["is_angle_recalib"] = False
                        self.nav_context["calib_generator"] = None
                        self.nav_context["nav_state"] = NavState.WAYPOINT_MOVE
                        current_nav_state = NavState.WAYPOINT_MOVE
                        self.last_valid_heading = None  # 清空航向缓存，强制重新计算
                        if hasattr(self, 'heading_history'):
                            self.heading_history = []  # 清空航向历史
                    else:
                        # 正常航点间校准：切换到下一个航点
                        self.get_logger().info(
                            f"[ROSNode] 航点{self.current_waypoint_idx}航向校准完成, 结果：{calib_result}"
                        )
                        self.current_waypoint_idx += 1
                        self.nav_context["nav_state"] = NavState.WAYPOINT_MOVE
                        current_nav_state = NavState.WAYPOINT_MOVE
                        self.last_valid_heading = None  # 清空航向缓存，切换航点后使用实时航向
                        self.heading_history = []  # 清空航向历史
                    
                    # 检查滚刷控制
                    self.check_and_control_brush()
                    self.nav_context["calib_generator"] = None
                    self.nav_context["target_waypoint"] = None
                    self.nav_context["last_distance"] = 0.0  # 重置距离缓存
                    self.nav_context["last_target_heading"] = 0.0  # 重置航向缓存

                    if not is_recalib:
                        # 仅在正常航点切换时获取新航点
                        target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                        self.nav_context["target_waypoint"] = target_waypoint
                        if target_waypoint:
                            self.get_logger().info(f"[ROSNode] 切换到新航点{self.current_waypoint_idx}, 准备进入移动阶段")
                        else:
                            self.get_logger().warn(f"[ROSNode] 未获取到航点{self.current_waypoint_idx}，导航结束")
                            yield (0.0, 0.0)
                            return
                    yield (0.0, 0.0)
                except Exception as e:
                    self.get_logger().error(f"[ROSNav] 航向校准失败：{str(e)}")
                    yield (0.0, 0.0)
                    return
            
            # # 子阶段B：移动到当前航点（核心：实时RTK角度矫正）（核心：基于上一个航点航向计算偏角）
            # if current_nav_state == NavState.WAYPOINT_MOVE:
            #     # 实时获取当前位置和目标航点经纬度
            #     current_lon, current_lat = self.current_gps
            #     target_lon, target_lat, _ = target_waypoint
                
            #     # 核心：每帧实时计算朝向角
            #     real_time_heading = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
            #     # 新增：角度平滑（取当前与上一帧的平均值，减少波动）
            #     # real_time_heading = (real_time_heading * 0.7 + self.nav_context["last_target_heading"] * 0.3)
            #     self.nav_context["last_target_heading"] = real_time_heading
            #     # 计算实时距离
            #     distance = self.calc_distance_to_waypoint(target_waypoint)

            #     # 核心2：获取上一个航点的航向角（处理跨文件/当前文件场景）
            #     last_heading = (self.imu_yaw +180)%360 -180  # 归一化[-180,180] # 默认值，防止未定义
            #     if self.last_waypoint_cache:  # 优先使用当前文件内缓存
            #         last_heading = self.last_waypoint_cache[2]  # 上一个航点的航向角
            #     elif self.cross_file_last_waypoint:  # 跨文件场景（上一个文件中最后一个航点）
            #         last_heading = (self.cross_file_last_waypoint[2] - 180) % 360 + 180
            #     last_heading = (last_heading +180)%360 -180  # 归一化[-180,180]
            #     # 距离未达标：实时纠偏行驶
            #     last_target_heading = 0.0
            #     if distance >= RTK_WAYPOINT_TOLERANCE:
            #         target_heading = real_time_heading - last_heading
            #         # if target_heading - last_target_heading > 1.0:
            #             # self.get_logger().info(f"last_heading={last_heading:.2f}, real_time_heading={real_time_heading:.2f}, target_heading(before)={target_heading:.2f}")
            #         # self.waypoints[self.current_waypoint_idx - 1][2] # 使用实时朝向角与上一个朝向角的夹角
            #         target_heading = (target_heading + 180 ) % 360 - 180   # heading_error need -self.imu_yaw
            #         correction = self.straight_get_speed_correction(target_heading) * STRAIGHT_PID_SCALE
            #         # ========== 新增：距离<1米时线性减速 ==========
            #         if distance < LOW_DISTANCE:
            #             speed_scale = max(0.1, distance/LOW_DISTANCE * 0.5)  # 线性缩放，最低保留10%基础速度
            #             current_base_speed = LINEAR_SPEED_BASE * speed_scale
            #             correction = correction * speed_scale
            #             # self.get_logger().info(f"[ROSNode] 目标航点{self.current_waypoint_idx}：距离小，减速（当前基础速度={current_base_speed:.2f}）")
            #         else:
            #             current_base_speed = LINEAR_SPEED_BASE
            #         last_target_heading = target_heading
            #         # 核心修改：用动态基础速度current_base_speed替代固定LINEAR_SPEED_BASE
            #         last_left_speed = None
            #         last_right_speed = None
            #         left_speed = -current_base_speed + correction
            #         right_speed = current_base_speed + correction
            #         if left_speed != last_left_speed or right_speed != last_right_speed:
            #             self.get_logger().debug(f"初始移动：base_speed={current_base_speed:.2f}, correction={correction:.2f}, left_speed={left_speed:.2f}, right_speed={right_speed:.2f}")
            #         last_left_speed = left_speed
            #         last_right_speed = right_speed
            #         # 速度限制
            #         # left_speed = max(min(left_speed, SPEED_LIMIT), -SPEED_LIMIT)
            #         # right_speed = max(min(right_speed, SPEED_LIMIT), -SPEED_LIMIT)
                    
            #         # 边界触发优先
            #         if self.is_boundary_triggered:
            #             left_speed, right_speed = self.get_boundary_correct_speed()
                    
            #         yield (left_speed, right_speed)
            #     # 距离达标：切换到校准阶段
            #     else:
            #         self.get_logger().info(
            #             f"[ROSNode] 已到达航点{self.current_waypoint_idx}距离阈值（{distance:.2f}m ≤ {RTK_WAYPOINT_TOLERANCE}m）"
            #         )
            #         target_heading_1 = self.get_path_heading(target_waypoint)
            #         calib_generator = self.calibrate_heading_at_waypoint(target_heading_1)
            #         self.nav_context["calib_generator"] = calib_generator
            #         current_nav_state = NavState.WAYPOINT_CALIB
            #         self.nav_context["nav_state"] = current_nav_state
            #         yield (0.0, 0.0)
            #     # 打印日志（减少冗余）
            #     if (abs(self.nav_context["last_distance"] - distance) > 0.1 or
            #         abs(self.nav_context["last_target_heading"] - real_time_heading) > 0.1):
            #         self.nav_context["last_distance"] = distance
            #         self.nav_context["last_target_heading"] = real_time_heading
            #         self.get_logger().info(
            #             f"[ROSNode] 目标航点{self.current_waypoint_idx}：距离{distance:.2f}m, last_heading={last_heading}, 实时朝向偏差{target_heading:.2f}°"
            #         )
            if current_nav_state == NavState.WAYPOINT_MOVE:
                # 实时获取当前位置和目标航点经纬度
                current_lon, current_lat = self.current_gps
                target_lon, target_lat, _ = target_waypoint
                
                # 核心：每帧实时计算当前位置→目标航点的路径方位角（机器人需要行驶的正确方向）
                real_time_heading = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
                distance = self.calc_distance_to_waypoint(target_waypoint)
                # if distance < 0.5:
                #     # 固定目标航向=航点预设航向（避免近距离抖动）
                #     target_heading_fixed = self.get_heading_error(real_time_heading)
                #     # target_heading_fixed = self.get_path_heading(target_waypoint)
                #     real_time_heading = self.last_valid_heading  # 覆盖实时计算的航向
                #     self.get_logger().info(f"[近距离固定航向] 距离{distance:.2f}m < 0.5m，固定目标航向为{target_heading_fixed:.2f}°")
                # else:
                #     self.last_valid_heading = real_time_heading  # 缓存有效航向
                # 1. 先判断距离，决定是否更新实时航向
                if distance >= 0.3:
                    # 距离足够远时，正常计算实时航向，并更新缓存
                    real_time_heading = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
                    self.last_valid_heading = real_time_heading  # 缓存最新的有效航向
                else:
                    # 距离<0.5m时，使用缓存的最后一次有效航向（不再实时计算）
                    if self.last_valid_heading is not None:
                        real_time_heading = self.last_valid_heading  # 复用之前的航向
                        self.get_logger().info(
                            f"[近距离固定航向] 距离{distance:.2f}m < 0.5m，固定目标航向为{real_time_heading:.2f}°"
                        )
                    else:
                        # 极端情况：首次进入近距离就无缓存，降级使用实时计算（避免程序报错）
                        real_time_heading = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
                        self.get_logger().warning(
                            f"[航向缓存异常] 距离{distance:.2f}m < 0.5m但无有效航向缓存，临时使用实时计算航向{real_time_heading:.2f}°"
                        )
                # 初始化航向历史缓存（最多保留5帧，可调整）
                # if not hasattr(self, 'heading_history'):
                #     self.heading_history = []
                # self.heading_history.append(real_time_heading)
                # if len(self.heading_history) > 5:
                #     self.heading_history.pop(0)
                # # 滑动平均平滑（削弱单帧抖动的影响）
                # real_time_heading = sum(self.heading_history) / len(self.heading_history)
                self.nav_context["last_target_heading"] = real_time_heading
                # 计算实时距离
                # 距离未达标：实时纠偏行驶
                last_target_heading = 0.0
                if distance >= RTK_WAYPOINT_TOLERANCE:
                    # ========== 新增：慢速调节时距离异常变大→后退纠正 ==========
                    # 1. 定义“慢速调节”场景：当前基础速度 < LINEAR_SPEED_BASE * 0.3（低于30%基础速度）
                    # 2. 定义“距离异常变大”：当前距离比上一帧距离大0.2m以上，且连续2帧变大
                    if not hasattr(self, 'last_distance_for_backup'):
                        self.last_distance_for_backup = distance
                        self.distance_increase_count = 0  # 距离连续增大计数器
                    
                    # 计算距离变化量（当前距离 - 上一帧距离）
                    distance_delta = distance - self.last_distance_for_backup
                    self.last_distance_for_backup = distance  # 更新上一帧距离
                    
                    # 判断是否触发后退纠正
                    need_backup = False
                    current_base_speed = LINEAR_SPEED_BASE  # 初始化基础速度
                    if distance < LOW_DISTANCE:
                        speed_scale = max(0.2, distance/LOW_DISTANCE * 0.7)
                        current_base_speed = LINEAR_SPEED_BASE * speed_scale
                    
                    # 触发条件：慢速调节（≤30%基础速度）+ 距离连续多次增大（每帧增大≥阈值m）
                    self.get_logger().debug(f"[后退检测] 距离={distance:.2f}m, 速度={current_base_speed:.2f}, delta={distance_delta:.3f}m")
                    if current_base_speed <= LINEAR_SPEED_BASE * BACKUP_SPEED_SCALE :
                        if distance_delta > DISTANCE_INCREASE_THRESHOLD:
                            self.distance_increase_count += 1
                            self.get_logger().warn(f"[距离异常] 慢速调节时距离增大：当前{distance:.2f}m，上一帧{self.last_distance_for_backup:.2f}m，增大{distance_delta:.2f}m，连续{self.distance_increase_count}次")
                            if self.distance_increase_count >= DISTANCE_INCREASE_COUNT :
                                need_backup = True
                        elif distance_delta < 0:
                            self.distance_increase_count = 0  # 距离减小，重置计数器
                    
                    # 执行后退纠正（持续0.5秒后退，再重新纠正航向）
                    if need_backup:
                        self.get_logger().error(f"[后退纠正] 触发条件满足，执行后退纠正")
                        # 后退速度：-0.3倍基础速度（缓慢后退，避免碰撞）
                        backup_speed = -LINEAR_SPEED_BASE * 0.3
                        left_speed = -backup_speed
                        right_speed = backup_speed  # 左右轮同速后退（根据电机逻辑调整正负，确保后退方向正确）
                        
                        # 发布后退速度并持续2.0秒
                        start_backup_time = self.get_clock().now()
                        while (self.get_clock().now() - start_backup_time).nanoseconds / 1e9 < BACKUP_DURATION and rclpy.ok():
                            yield (left_speed, right_speed)
                        
                        # 后退后重置计数器，重新进入正常纠偏
                        self.distance_increase_count = 0
                        self.last_distance_for_backup = self.calc_distance_to_waypoint(target_waypoint)
                        self.get_logger().info(f"[后退完成] 重新计算距离：{self.last_distance_for_backup:.2f}m，恢复正常纠偏")
                    
                    # ========== 原有纠偏逻辑（后退纠正未触发时执行） ==========
                    else:
                        # 目标航向计算（删除冗余的last_heading偏移，直接用固定/平滑后的real_time_heading）
                        target_heading = real_time_heading
                        # 计算真实偏差角（IMU当前航向 - 目标航向）
                        yaw_error = self.get_heading_error(target_heading)
                        # 连续角度异常检测
                        yaw_error_abs = abs(yaw_error)
                        self.get_logger().debug(f"[角度检测] IMU航向={self.imu_yaw:.2f}°, 目标航向={target_heading:.2f}°, 误差={yaw_error:.2f}°")
                        if yaw_error_abs > 15.0:
                            # 增加角度异常计数器
                            self.nav_context["angle_abnormal_count"] = self.nav_context.get("angle_abnormal_count", 0) + 1
                            abnormal_count = self.nav_context["angle_abnormal_count"]
                            self.get_logger().warn(f"[角度异常] yaw_error={yaw_error:.2f}°（>{15.0}°），连续异常{abnormal_count}次")
                            
                            # 连续多次异常，重新进入角度校准阶段
                            if abnormal_count >= ANGLE_ABNORMAL_COUNT:
                                self.get_logger().warn(f"[角度异常] 连续{abnormal_count}次角度异常，切换到航向校准阶段")
                                # 创建校准生成器
                                # calib_generator = self.calibrate_heading_at_waypoint(target_waypoint[2])
                                calib_generator = self.calibrate_heading_at_waypoint(real_time_heading)
                                self.nav_context["calib_generator"] = calib_generator
                                self.nav_context["nav_state"] = NavState.WAYPOINT_CALIB
                                current_nav_state = NavState.WAYPOINT_CALIB  # 同步更新当前状态变量
                                self.nav_context["angle_abnormal_count"] = 0  # 重置计数器
                                self.nav_context["is_angle_recalib"] = True  # 设置重新校准标志
                                # 返回停止速度，等待切换状态
                                yield (0.0, 0.0)
                                continue
                            else:
                                # 角度异常但不满足触发条件，仅暂停当前帧，等待下一帧继续判断
                                yield (0.0, 0.0)
                                continue
                        else:
                            # 角度正常，重置计数器
                            self.nav_context["angle_abnormal_count"] = 0
                        
                        # 计算修正量（使用真实偏差角）
                        correction = self.straight_get_speed_correction(yaw_error) * STRAIGHT_PID_SCALE
                        
                        # 近距离线性减速（原有逻辑保留）
                        if distance < LOW_DISTANCE:
                            speed_scale = max(0.2, distance/LOW_DISTANCE * 0.7)
                            current_base_speed = LINEAR_SPEED_BASE * speed_scale
                            # correction = correction * speed_scale
                            # 修正量不随速度缩放，保证近距离纠偏力度
                        else:
                            current_base_speed = LINEAR_SPEED_BASE
                        
                        # 计算左右轮速度（原有逻辑保留）
                        last_left_speed = None
                        last_right_speed = None
                        left_speed = -current_base_speed + correction
                        right_speed = current_base_speed + correction
                        
                        if left_speed != last_left_speed or right_speed != last_right_speed:
                            self.get_logger().debug(f"正常纠偏：base_speed={current_base_speed:.2f}, correction={correction:.2f}, left_speed={left_speed:.2f}, right_speed={right_speed:.2f}")
                        last_left_speed = left_speed
                        last_right_speed = right_speed
                        
                        # 边界触发优先（原有逻辑保留）
                        if self.is_boundary_triggered or self.boundary_correct_locked:
                            left_speed, right_speed = self.get_boundary_correct_speed()
                        
                        yield (left_speed, right_speed)
                # # 距离未达标：实时纠偏行驶
                # if distance >= RTK_WAYPOINT_TOLERANCE:
                #     # 1. 目标路径航向：当前位置→目标航点的实时方位角
                #     target_path_heading = real_time_heading
                #     # 2. 核心：计算IMU当前航向 与 目标路径航向的归一化偏差角（[-180°,180°]）
                #     target_heading = self.get_heading_error(target_path_heading)
                #     # 3. 计算速度修正量（基于真实的IMU-目标航向偏差）
                #     correction = self.straight_get_speed_correction(target_heading) * STRAIGHT_PID_SCALE
                    
                #     # 近距离线性减速（保留原有逻辑，无需修改）
                #     if distance < LOW_DISTANCE:
                #         speed_scale = max(0.1, distance/LOW_DISTANCE * 0.5)
                #         current_base_speed = LINEAR_SPEED_BASE * speed_scale
                #         # 修正量不随速度缩放，保证近距离纠偏力度
                #     else:
                #         current_base_speed = LINEAR_SPEED_BASE
                    
                #     # 计算左右轮速度（保留原有逻辑）
                #     left_speed = -current_base_speed + correction
                #     right_speed = current_base_speed + correction
                #     # 速度限制（可选，根据实际电机性能开启）
                #     # left_speed = max(min(left_speed, SPEED_LIMIT), -SPEED_LIMIT)
                #     # right_speed = max(min(right_speed, SPEED_LIMIT), -SPEED_LIMIT)
                    
                #     # 边界触发优先（保留原有逻辑）
                #     if self.is_boundary_triggered:
                #         left_speed, right_speed = self.get_boundary_correct_speed()
                    
                #     yield (left_speed, right_speed)
                # 距离达标：切换到航向校准阶段（保留原有逻辑）
                else:
                    self.get_logger().info(
                        f"[ROSNode] 已到达航点{self.current_waypoint_idx}距离阈值（{distance:.2f}m ≤ {RTK_WAYPOINT_TOLERANCE}m）"
                    )
                    calib_target_heading = self.get_path_heading(target_waypoint)
                    calib_generator = self.calibrate_heading_at_waypoint(calib_target_heading)
                    self.nav_context["calib_generator"] = calib_generator
                    current_nav_state = NavState.WAYPOINT_CALIB
                    self.nav_context["nav_state"] = current_nav_state
                    yield (0.0, 0.0)
                
                # 打印日志（简化，只输出关键信息）
                if (abs(self.nav_context["last_distance"] - distance) > 0.1 or
                    abs(self.nav_context["last_target_heading"] - real_time_heading) > 0.1):
                    self.nav_context["last_distance"] = distance
                    self.nav_context["last_target_heading"] = real_time_heading
                    # 日志输出真实偏差：IMU航向、目标路径航向、偏差角
                    self.get_logger().info(
                        f"[ROSNode] 目标航点{self.current_waypoint_idx}：距离{distance:.2f}m, IMU航向{self.imu_yaw:.2f}°, 目标路径航向{real_time_heading:.2f}°, 偏差角{self.get_heading_error(real_time_heading):.2f}°"
                    )
        # 所有航点完成
        self.get_logger().info("[ROSNode] RTK多点导航全部完成")
        self.nav_context["nav_state"] = NavState.COMPLETED
        self.publish_nav_state(NavState.COMPLETED)
        self.nav_running = False
        self.brush_active = False  # 导航完成，关闭滚刷
        self.publish_stop_speed()
        self.reset_nav_context()        # reset nav /保留导航状态，支持恢复
        yield (0.0, 0.0)

    # ================== 原有RTKControlNode核心方法 ==================
    def publish_stop_speed(self):
        """发布停止电机速度指令"""
        stop_speed = Vector3()
        stop_speed.x = 0.0
        stop_speed.y = 0.0
        stop_speed.z = 0.0
        self.motor_speed_pub.publish(stop_speed)

    def publish_brush_speed(self, speed: float):
        """发布滚刷速度指令（仅更新标志位，不单独发布）"""
        self.brush_active = (speed != 0)
        self.get_logger().info(f"[RTKNav] 设置滚刷状态: {'开启' if self.brush_active else '关闭'}")

    def check_and_control_brush(self):
        """根据当前航点索引控制滚刷开关"""
        if not hasattr(self, 'brush_start_idx') or not hasattr(self, 'brush_stop_idx'):
            return
        
        idx = self.current_waypoint_idx
        
        if self.brush_start_idx is not None and idx >= self.brush_start_idx and not self.brush_active:
            self.publish_brush_speed(RTK_BRUSH_SPEED)
            self.brush_start_idx = None
        
        if self.brush_stop_idx is not None and idx >= self.brush_stop_idx and self.brush_active:
            self.publish_brush_speed(0.0)
            self.brush_stop_idx = None

    def publish_nav_state(self, state: NavState):
        """发布当前导航状态"""
        state_msg = String()
        state_msg.data = state if isinstance(state, str) else state.value
        self.nav_state_pub.publish(state_msg)

    def state_callback(self, msg: String):
        """电机状态回调函数：监听控制状态变化，HOLD暂停导航"""
        if msg.data == "HOLD" and self.last_state != "HOLD":
            self.current_control_mode = ControlMode.NORMAL
            self.get_logger().warn("[RTKNav] 电机状态为HOLD，强制停止导航")
            self.nav_running = False
            if hasattr(self, 'nav_context'):
                self.nav_context["brush_active"] = self.brush_active  # 保存滚刷状态
            self.publish_stop_speed()
        elif msg.data == "AUTO_CLEANING" and self.last_state != "AUTO_CLEANING":
            self.current_control_mode = ControlMode.AUTO_CLEANING
            self.get_logger().info("[RTKNav] 电机状态为AUTO_CLEANING，恢复导航")
            if hasattr(self, 'nav_context'):
                self.brush_active = self.nav_context.get("brush_active", False)  # 恢复滚刷状态
                if self.brush_active:
                    self.publish_brush_speed(RTK_BRUSH_SPEED)
                else:
                    self.publish_brush_speed(0.0)
        self.last_state = msg.data

    def route_change_callback(self, msg: String):
        """路径切换回调：接收新路径文件并重新加载"""
        new_path_file = msg.data.strip()
        if not new_path_file:
            self.get_logger().warn("[RTKNav] 收到空路径文件，跳过")
            return
        
        if not os.path.exists(new_path_file):
            self.get_logger().error(f"[RTKNav] 路径文件不存在: {new_path_file}")
            return
        
        self.get_logger().info(f"[RTKNav] 收到路径切换指令: {new_path_file}")
        self.cross_file_last_waypoint = None
        self.waypoints = []
        self.is_first_file_load = False
        self.rtk_path_file = new_path_file
        self.current_waypoint_idx = 0  # 重置航点索引为0
        self.get_logger().info(f"[RTKNav] 路径切换时重置航点索引为0")
        self.return_to_loading_added = False  # 重置出仓点追加标志
        self.brush_start_idx = None  # 重置滚刷开启索引
        self.brush_stop_idx = None   # 重置滚刷关闭索引
        self.brush_active = False    # 重置滚刷状态
        self.nav_context["nav_state"] = NavState.IDLE  # 重置导航状态为IDLE
        self.nav_context["pre_pause_state"] = None  # 重置暂停状态
        self.publish_nav_state(self.nav_context["nav_state"])
        self.load_waypoints_from_file(self.rtk_path_file)
        
        # 同步滚刷状态到nav_context（加载路径后会更新brush_active）
        if hasattr(self, 'nav_context'):
            self.nav_context["brush_active"] = self.brush_active
        
        if self.waypoints:
            self.get_logger().info(f"[RTKNav] 路径切换成功，共 {len(self.waypoints)} 个航点")
        else:
            self.get_logger().error(f"[RTKNav] 路径切换失败，无法加载航点")

    def mode_callback(self, msg: String):
        """接收电机节点的控制模式, 更新自身状态"""
        previous_mode = self.current_control_mode
        self.current_control_mode = msg.data
        # 切换到RTK模式时, 重置IMU校准
        # if self.current_control_mode == ControlMode.AUTO_CLEANING and previous_mode != ControlMode.AUTO_CLEANING:
        #     # self.reset_imu_calibration()
        #     # 新增：强制重置导航生成器和运行状态
        #     self.multi_waypoint_generator = None
        #     self.nav_running = False
        if self.current_control_mode == ControlMode.AUTO_CLEANING and previous_mode != ControlMode.AUTO_CLEANING:
            self.multi_waypoint_generator = None
            self.nav_running = False
            # 核心修改：若之前是初始移动中断，强制保留/重置为INITIAL_MOVE
            if self.nav_context["nav_state"] not in [NavState.IDLE, NavState.WAYPOINT_MOVE, NavState.WAYPOINT_CALIB, NavState.COMPLETED, NavState.PAUSE]:
                self.nav_context["nav_state"] = NavState.INITIAL_MOVE
            self.get_logger().info(f"切换到RTK模式，导航状态：{self.nav_context['nav_state']}")

        # 切换非RTK模式时, 保存导航状态, 停止导航
        if self.current_control_mode != ControlMode.AUTO_CLEANING:
            if self.multi_waypoint_generator:
                self.multi_waypoint_generator = None
            self.nav_running = False
            self.publish_stop_speed()
            # 新增：无航点时强制重置导航上下文，避免恢复时保留异常状态
            if not self.waypoints:
                self.reset_nav_context()
                self.get_logger().info("[ROSNode] 非RTK模式：无航点数据，重置导航上下文为IDLE")
            # 不重置导航上下文, 保存状态以便后续恢复

    def rtk_timer_callback(self):
        """10Hz定时器回调, 驱动多点导航逻辑"""
        # self.is_boundary_triggered = self.get_parameter('is_boundary_triggered').value
        # 仅在RTK导航模式下执行导航逻辑
        if self.current_control_mode == ControlMode.AUTO_CLEANING:
            # 新增：启动/恢复导航前，强制校验航点有效性
            if not self.waypoints:
                self.get_logger().error("[ROSNode] RTK模式启动失败：无有效航点数据，请先加载航点")
                self.nav_running = False
                self.multi_waypoint_generator = None
                self.reset_nav_context()  # 重置导航状态为IDLE
                self.publish_nav_state(NavState.IDLE)
                return
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
                    # 根据滚刷状态设置速度
                    brush_speed = RTK_BRUSH_SPEED if getattr(self, 'brush_active', False) else 0.0
                    speed_msg.z = brush_speed
                    self.motor_speed_pub.publish(speed_msg)
            except StopIteration:
                # 导航生成器执行完毕（全部航点完成/主动退出）
                self.get_logger().info("[ROSNode] 多点导航生成器执行完毕")
                self.publish_nav_state(NavState.COMPLETED)
                self.multi_waypoint_generator = None
                self.nav_running = False
                self.brush_active = False  # 导航完成，关闭滚刷
            except Exception as e:
                self.get_logger().error(f"[ROSNode] RTK多点导航错误：{str(e)}")
                # 发布停止指令
                self.publish_stop_speed()
                self.brush_active = False  # 异常退出，关闭滚刷
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