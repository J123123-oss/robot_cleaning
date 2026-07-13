#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
from typing import Optional, List, Dict, Tuple, Generator
from collections import deque
import json
import time
import rclpy
import re
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Vector3  # 用于发布左右轮速度
from std_msgs.msg import String, UInt8, Float32, Int16, Bool  # 用于发布控制模式和导航状态
from custom_msgs.msg import WTRTK
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType

# -------------------------- 全局配置与枚举 --------------------------
# 新增：信号漂移过滤配置
GPS_SMOOTH_WINDOW = 5  # GPS经纬度滑动平均窗口大小（帧）

# 滚刷速度
RTK_BRUSH_SPEED = 18.0 #18.0  # 负数表示正常运行速度，与前进反方向，安装转轴后相反

# RTK导航配置
RTK_WAYPOINT_TOLERANCE = 0.10 # 多点导航距离阈值
RTK_HEADING_TOLERANCE = 1.0  # 多点导航角度阈值 0.2
LINEAR_SPEED_BASE = 10.0   # origin 8.0
# TURN_SPEED = 1.0 *2     # origin 0.1
INITIAL_MOVE_TOLERANCE = 0.1 #起始点距离阈值
RTK_CALIBRATION_TIMEOUT = 5.0
RTK_DATA_TIMEOUT = 1.0
RTK_TIMEOUT_LOG_INTERVAL = 2.0
IMU_CALIBRATION_TIMEOUT = 3.0
HEADING_CALIBRATION_TIMEOUT = 40.0
HEADING_ABNORMAL_TIMEOUT = 15.0  # 航向角异常全局超时（秒），超时后暂停导航
HEADING_RECOVERY_CHECK_INTERVAL = 3.0  # 航向异常超时后恢复检查间隔（秒）
CALIB_STUCK_MAX_RETRIES = 3  # 校准卡滞最大重试次数，超此次数后才永久暂停

TURN_SPEED_FAST = 1.5 # 大误差快速转向基准速度
TURN_SPEED_MID = 1.0  # 中误差中等转向基准速度
TURN_SPEED_SLOW = 0.4 # 小误差慢速转向基准速度（草地需更高最低速克服静摩擦）
MAX_CORRECTION = 2.0   # 车体停止后，旋转调整最大修正量
# STRAIGHT_MAX_CORRECTION = 3.0 # 直线运行最大纠正量
# straight line speed correction factor
STRAIGHT_PID_SCALE = 2.0  # 2.0
SPEED_LIMIT = 1.3 * LINEAR_SPEED_BASE

#近距离减速/REVERSE阈值
LOW_DISTANCE = 1.5
BACKUP_DURATION = 2.0  # 后退纠正持续时间（秒）
BACKUP_SPEED_SCALE = 0.3  # 后退速度缩放系数（相对于基础速度）
DISTANCE_INCREASE_THRESHOLD = 0.05  # 距离增大触发阈值（米）
DISTANCE_INCREASE_COUNT = 3  # 连续增大次数阈值
ANGLE_ABNORMAL_COUNT = 5  # 连续角度异常次数阈值（触发重新进入角度校准）
HEADING_ABNORMAL_THRESHOLD = 15.0  # 航向异常阈值（度），连续超限后重新校准
FORCE_BEARING_MAX_RECALIB = 3  # force_bearing 反复原地对准上限，超限判定为极限环并暂停上报
FORCE_BEARING_DIVERGE_DIST = 0.5  # force_bearing 背离目标阈值（米），比历史最近点远出此值开始计数
FORCE_BEARING_DIVERGE_COUNT = 10  # force_bearing 连续背离帧数上限（10Hz≈1s），超限暂停上报
MANUAL_INTERVENTION_PAUSE_REASONS = frozenset({
    "calib_stuck",
    "force_bearing_limit_cycle",
    "force_bearing_diverge",
})
# 初始航向校验（仅检查航向是否稳定无漂移，不限制固定角度——出仓后无论车头朝哪，稳住即放行）
# 航向稳定性由 heading_callback 中的 HEADING_STABILITY_WINDOW / HEADING_STABILITY_RANGE 控制
# INITIAL_HEADING_TARGET / INITIAL_HEADING_TOLERANCE 已废弃，不再使用固定角度判定
# Stanley控制器参数
STANLEY_K = 2.0  # Stanley增益，控制横向误差响应强度
STANLEY_MIN_SPEED = 0.15
STANLEY_K_BASE = 0.5
STANLEY_MAX_K = 1.0
MAX_LATERAL_ERROR = 1.0
STRAIGHT_MAX_CORRECTION = 1.5
SPEED_CMD_TO_MPS = 0.0345  # 电机指令值 → 实际速度 (m/s) 的转换系数

# 固定进仓RTK航点: (lon, lat, heading)。
# 填入现场标定的固定进仓点后，RTK导航结束会把该点追加到最后一个航点，且每轮任务只追加一次。
BUILTIN_LOADING_GPS = (0.0, 0.0, 0.0)

BOUNDARY_TRIGGER_CONFIRM_FRAMES = 3
BOUNDARY_CLEAR_CONFIRM_FRAMES = 2
# 边界航向角闭环矫正
TURN_AWAY_DEG = 25.0               # 偏离边缘的目标转角（度）
BOUNDARY_SLOW_PERSIST_FRAMES = 15  # 单传感器慢速通道：持续N帧(1.5s@10Hz)触发
BOUNDARY_CORRECTION_TIMEOUT = 15.0 # 整体纠偏超时（秒）
ERROR_RTK_NOT_FIXED = 4
ERROR_RTK_TIMEOUT = 8
ERROR_TILT_FAULT = 64        # 倾斜/跌落故障
ERROR_CALIB_TIMEOUT = 256    # 航向校准超时

# 倾斜检测配置
TILT_ANGLE_THRESHOLD = 15.0  # 倾角阈值（度），abs(angle_x)或abs(angle_y)超此值判定倾斜
TILT_CONFIRM_FRAMES = 30     # 连续倾斜帧数确认（防抖），避免颠簸误报，约3秒
TILT_RECOVERY_FRAMES = 5     # 连续正常帧数清除故障
TILT_SUDDEN_DELTA = 5.0      # 突变阈值（度），1s内角度变化超此值才视为真实倾斜，过滤IMU零偏漂移
TILT_BASELINE_SAMPLES = 10   # 基线窗口样本数（1秒@10Hz），取中位数作为漂移基线
TILT_STABILIZE_TIMEOUT = 120.0  # 跌落后INS稳定等待时间（秒），重新AUTO_CLEANING后等待此时间再开始
TILT_SHORT_DURATION = 10.0    # 短促倾斜阈值（秒），倾斜持续低于此值视为颠簸，跳过稳定等待

# 航向稳定性检查（基于 ins_heading，静止时无漂移才放行）
# 不限制固定角度范围——出仓后遥控接管等场景下车头朝向不固定，只要IMU不漂移就判定为稳定
HEADING_STABILITY_WINDOW = 5.0      # 稳定性检查窗口（秒）
HEADING_STABILITY_RANGE = 3.0       # 窗口内最大允许变化（度），超出判定漂移中

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
        self.waypoint_areas: List[str] = []
        self.current_waypoint_idx = 0
        self.current_cleaning_area = ""
        self.last_published_cleaning_area = None
        self.current_gps: Optional[Tuple[float, float]] = [0.0, 0.0]
        self.current_lon = 0.0
        self.current_lat = 0.0
        self.imu_yaw = 0.0
        self.rtk_install_offset = 90 #-90.0    #-90.0(old)  # RTK安装偏移角度

         # 例如：天线在车体中心前方0.31米，左侧0.2米（根据实际安装位置调整）
        self.antenna_offset_front = 0.2517   # 前向偏移（+：天线在车体前，-：在后）
        self.antenna_offset_left = 0.19625  #old -0.19625   # 左向偏移（-：天线在车体左，+：在右）

        # 新增：出仓点基准缓存与偏移量
        self.base_loading_waypoint = None  # 基准出仓点（首次接收的出仓点），格式：(lon, lat, heading)
        self.waypoint_offset = {
            "lon_offset": 0.0,    # 经纬度偏移量（°）
            "lat_offset": 0.0,
            "heading_offset": 0.0 # 航向角偏移量（°）
        }
        self.laste_state = None  # 保存上一个  "status": "ENABLE"状态
        self.offset_calculated = False  # 偏移量是否已计算（避免重复计算）

        self.imu_initialized = False
        self._heading_stability_history = deque()  # (timestamp, vehicle_heading_360)
        self._last_heading_stable = False
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

        # 固定进仓点（不再跟随每次出仓完成时的实时/unloading_gps漂移）
        self.declare_parameter("loading_gps", list(BUILTIN_LOADING_GPS))
        self.loading_waypoint = self.load_builtin_loading_gps()  # 格式：(lon, lat, heading)
        self.return_to_loading_added = False  # 出仓点是否已追加标志
        self.pending_next_path_file = None
        self.waiting_for_next_unloading = False

        # 边界矫正状态机
        self.boundary_correct_state = BoundaryCorrectState.IDLE
        self.boundary_correct_start_time = None
        self.boundary_correct_direction = None  # 'left' 或 'right'
        self.boundary_correct_locked = False    # 锁定标志，锁定后不受传感器条件影响
        self.BOUNDARY_TURN_DURATION = 1.0    # 偏转持续时间（秒）
        self.BOUNDARY_BACK_DURATION = 4.0    # 后退持续时间（秒）
        self.BOUNDARY_RETURN_DURATION = 1.0  # 反向偏转退回持续时间（秒）
        self.boundary_active_count = 0
        self.boundary_clear_count = 0
        self.boundary_last_raw_trigger = False
        self.boundary_stop_published = False
        self.boundary_trigger_yaw = 0.0                # 触发瞬间的IMU航向
        self.boundary_target_yaw = 0.0                 # TURNING/RETURNING目标航向
        self.boundary_slow_count = 0                  # 单传感器持续帧计数
        self.boundary_correction_start_time = None    # 整体超时起点（monotonic秒）
        self.confirmed_sensors = set()               # 已确认触发的传感器名（经debounce后）
        self.blocked_directions = set()              # 基于confirmed_sensors计算的禁止方向
        self._last_motor_left = 0.0                  # 最近发布的左轮速度
        self._last_motor_right = 0.0                 # 最近发布的右轮速度

        self.gps_cache = []
        # 距离历史缓存（用于异常检测）
        self.distance_cache = []
        # 目标航向角滑动缓存
        self.heading_cache = []
        # IMU偏航角滑动缓存
        self.imu_yaw_cache = []

        self.last_valid_heading = None  # 存储距离≥0.5m时最后一次计算的航向
        self.bad_error_counter = 0   #yaw_error绝对值大于15度的次数
        
        self.brush_start_indices = []  # 滚刷开启的航点索引队列
        self.brush_stop_indices = []   # 滚刷关闭的航点索引队列
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
        self.is_boundary_triggered = False
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
            "pause_reason": None,
            "manual_intervention_seen": False,  # 人工锁定后是否已切出AUTO_CLEANING处理
            "angle_abnormal_count": 0,  # 连续角度异常计数器
            "is_angle_recalib": False,  # 是否是角度异常后的重新校准
            "waypoint_recalib_count": 0,  # 同航点校准次数（打滑检测）
            "force_bearing_mode": False,  # 跳过循迹，直接用方位角直行
            "force_bearing_target": None,  # 当前帧实时目标方位角（仅用于诊断）
            "force_bearing_recalib_count": 0,  # force_bearing 原地对准反复触发计数（极限环兜底）
            "force_bearing_min_distance": float('inf'),  # force_bearing 期间到目标的历史最近距离（背离检测基准）
            "distance_increase_count": 0,  # force_bearing 连续背离目标帧数（背离兜底）
            "bearing_mode_locked": False,  # 一旦t>1.0锁定方位角模式，防振荡
            "tilt_confirm_count": 0,     # 连续倾斜帧数（触发确认）
            "tilt_normal_count": 0,      # 连续正常帧数（恢复确认）
            "tilt_fault": False,         # 倾斜故障标志
            "calib_retry_count": 0,      # 校准卡滞重试次数
            "calib_target_heading": None,  # 当前校准目标航向（重试/恢复时复用，保证一致性）
        }

        # ================== 原有RTKControlNode属性 ==================
        self.rate = self.create_rate(10)  # 10Hz, origin 4
        self.nav_generator: Optional[Generator] = None
        self.nav_running = False
        self.last_wtrtk_time = time.monotonic()
        self.rtk_data_timed_out = False
        self.last_gps_status = -1
        self.last_orientation_status = -1
        self.position_data_valid = False
        self.rtk_solution_ready = False
        self.rtk_error_code = 0
        self.last_rtk_timeout_log_time = 0.0
        self.last_heading_check_log_time = 0.0
        self.heading_abnormal_start_time = None  # 航向角异常开始时间，None表示当前正常
        self.heading_timed_out = False  # 航向角异常导致的超时标志
        self.last_tilt_time = 0.0       # 最后一次倾斜故障确认时间，用于跌落后稳定等待
        self.last_tilt_duration = 0.0  # 上一次倾斜故障的持续时间（秒），用于判断是否短促颠簸
        self._angle_x_history = deque(maxlen=TILT_BASELINE_SAMPLES)  # 倾角基线窗口，滤IMU漂移
        self._angle_y_history = deque(maxlen=TILT_BASELINE_SAMPLES)
        self._last_heading_recovery_check = 0.0  # 上次航向恢复检查时间
        self._last_nav_context_publish = 0.0      # 上次 nav_context 发布时间
        self.multi_waypoint_generator = None  # 多点导航生成器
        self.real_velocity = 0.0  # 当前真实速度 (m/s)

        # ROS2发布器/订阅器
        self.motor_speed_pub = self.create_publisher(Vector3, "/rtk/motor_speed", 10)
        self.nav_state_pub = self.create_publisher(String, "/rtk/nav_state", 10)
        self.cleaning_area_pub = self.create_publisher(String, "/rtk/cleaning_area", 10)
        self.current_route_pub = self.create_publisher(String, "/rtk/current_route_id", 10)
        self.rtk_error_pub = self.create_publisher(Int16, "/rtk/error_status", 10)
        self.imu_heading_pub = self.create_publisher(Float32, "/imu_heading", 10)
        self.heading_stable_pub = self.create_publisher(Bool, "/heading_stable", 10)
        self.velocity_pub = self.create_publisher(Float32, "/rtk/velocity", 10)
        self.car_center_gps_pub = self.create_publisher(NavSatFix, "car_center_gps", 10)
        self.nav_context_pub = self.create_publisher(String, "/rtk/nav_context", 10)  # 调试用：nav_context状态快照

        self.control_mode_sub = self.create_subscription(String, "/control/mode", self.mode_callback, 10)
        # self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
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
            self.waypoint_areas = []
            self.current_waypoint_idx = 0
            current_area = ""
            
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
                            start_idx = len(self.waypoints)
                            self.brush_start_indices.append(start_idx)
                            self.get_logger().info(f"[RTKNav] 检测到#start标记，滚刷将在航点{start_idx}开启")
                            if start_idx == 0:
                                self.brush_active = True
                        elif 'stop' in comment:
                            stop_idx = len(self.waypoints)
                            self.brush_stop_indices.append(stop_idx)
                            self.get_logger().info(f"[RTKNav] 检测到#stop标记，滚刷将在航点{stop_idx}关闭")
                        if comment:
                            current_area = line[1:].strip()
                        continue
                    
                    parts = line.split(',')
                    if len(parts) < 4:
                        self.get_logger().warn(f"[RTKNav] 航点行格式错误，跳过: {line}")
                        continue
                    
                    seq, raw_lon, raw_lat, raw_heading = parts[0], parts[1], parts[2], parts[3]
                    self.waypoints.append((float(raw_lon), float(raw_lat), float(raw_heading)))
                    self.waypoint_areas.append(current_area)
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
            self.update_cleaning_area(force=True)
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
        边界触发时的航向角闭环矫正速度计算（状态机版本）
        逻辑：偏转至目标航向 → 直行后退 → 旋转回原始航向
        TURNING/RETURNING 使用 IMU 航向闭环，复用 get_adaptive_turn_speed + get_speed_correction
        """
        base_correct_speed = self.correct_speed_scale * LINEAR_SPEED_BASE
        left_speed = 0.0
        right_speed = 0.0
        current_time = time.time()

        # ---- 整体超时保护 ----
        if self.boundary_correct_locked and self.boundary_correction_start_time is not None:
            if current_time - self.boundary_correction_start_time > BOUNDARY_CORRECTION_TIMEOUT:
                self.get_logger().error(
                    f"[RTKNav] 边界矫正超时({BOUNDARY_CORRECTION_TIMEOUT}s)，强制退出")
                self._reset_boundary_correction()
                return (0.0, 0.0)

        # ---- IDLE：检测触发并启动 ----
        if self.boundary_correct_state == BoundaryCorrectState.IDLE and not self.boundary_correct_locked:
            direction = self._determine_boundary_direction()
            if direction is not None:
                self._start_boundary_correction(current_time, direction)

        # ---- TURNING：航向角闭环旋转到 boundary_target_yaw ----
        if self.boundary_correct_state == BoundaryCorrectState.TURNING:
            heading_error = self.get_heading_error(self.boundary_target_yaw)
            if abs(heading_error) <= RTK_HEADING_TOLERANCE:
                self.boundary_correct_state = BoundaryCorrectState.BACKING
                self.boundary_correct_start_time = current_time
                self.get_logger().info(
                    f"[RTKNav] 边界矫正：偏转完成(误差={heading_error:.1f}°)，进入后退")
            else:
                turn_speed = self.get_adaptive_turn_speed(abs(heading_error))
                correction = self.get_speed_correction(self.boundary_target_yaw)
                min_positive_speed = 0.5
                if heading_error > 0:
                    left_speed = -max(turn_speed - correction, min_positive_speed)
                    right_speed = -max(turn_speed - correction, min_positive_speed)
                else:
                    left_speed = max(turn_speed + correction, min_positive_speed)
                    right_speed = max(turn_speed + correction, min_positive_speed)

        # ---- BACKING：直行后退（时间基准） ----
        elif self.boundary_correct_state == BoundaryCorrectState.BACKING:
            elapsed = current_time - self.boundary_correct_start_time
            if self.boundary_correct_direction == 'behind':
                # 后方边缘 → 前进远离（不能后退）
                left_speed = -base_correct_speed       # -v = 前进
                right_speed = base_correct_speed        # +v = 前进
            else:
                # 前方/侧方边缘 → 后退远离
                left_speed = base_correct_speed        # +v = 后退
                right_speed = -base_correct_speed       # -v = 后退
            if elapsed >= self.BOUNDARY_BACK_DURATION:
                self.boundary_correct_state = BoundaryCorrectState.RETURNING
                self.boundary_correct_start_time = current_time
                self.boundary_target_yaw = self.boundary_trigger_yaw
                action = "前进" if self.boundary_correct_direction == 'behind' else "后退"
                self.get_logger().info(
                    f"[RTKNav] 边界矫正：{action}完成，返回原始航向{self.boundary_trigger_yaw:.1f}°")

        # ---- RETURNING：航向角闭环旋转回 boundary_trigger_yaw ----
        elif self.boundary_correct_state == BoundaryCorrectState.RETURNING:
            heading_error = self.get_heading_error(self.boundary_trigger_yaw)
            if abs(heading_error) <= RTK_HEADING_TOLERANCE:
                self.get_logger().info(
                    f"[RTKNav] 边界矫正完成(误差={heading_error:.1f}°)，恢复正常导航")
                self._reset_boundary_correction()
            else:
                turn_speed = self.get_adaptive_turn_speed(abs(heading_error))
                correction = self.get_speed_correction(self.boundary_trigger_yaw)
                min_positive_speed = 0.5
                if heading_error > 0:
                    left_speed = -max(turn_speed - correction, min_positive_speed)
                    right_speed = -max(turn_speed - correction, min_positive_speed)
                else:
                    left_speed = max(turn_speed + correction, min_positive_speed)
                    right_speed = max(turn_speed + correction, min_positive_speed)

        self.get_logger().debug(
            f"[RTKNav] 边界矫正状态: {self.boundary_correct_state}, "
            f"target_yaw={self.boundary_target_yaw}, "
            f"左={left_speed:.2f}, 右={right_speed:.2f}")
        return (left_speed, right_speed)

    def _determine_boundary_direction(self) -> Optional[str]:
        """
        根据 mid/back 共4路传感器判定边缘方向。
        返回 'left' / 'right' / 'ahead' / 'behind'，无传感器触发返回 None。
        """
        left_active = self.mid_left or self.back_left
        right_active = self.mid_right or self.back_right

        if left_active and right_active:
            # 区分前方边缘（mid触发）和后方边缘（back触发）
            # 后方双侧触发 → 前进远离，不能后退
            if self.back_left and self.back_right and not (self.mid_left and self.mid_right):
                return 'behind'
            return 'ahead'
        elif left_active:
            return 'left'
        elif right_active:
            return 'right'
        else:
            return None

    def _start_boundary_correction(self, current_time: float, direction: str):
        """初始化边界矫正状态机：捕获触发航向 → 计算目标航向 → 锁定。"""
        self.boundary_trigger_yaw = self.imu_yaw
        self.boundary_correction_start_time = current_time
        self.boundary_correct_direction = direction
        self.boundary_correct_locked = True

        if direction == 'ahead':
            self.boundary_correct_state = BoundaryCorrectState.BACKING
            self.boundary_correct_start_time = current_time
            self.boundary_target_yaw = self.boundary_trigger_yaw
            self.get_logger().info(
                f"[RTKNav] 边界矫正启动：前方双侧触发(yaw={self.boundary_trigger_yaw:.1f}°)，"
                f"直接后退 [锁定]")
        elif direction == 'behind':
            self.boundary_correct_state = BoundaryCorrectState.BACKING
            self.boundary_correct_start_time = current_time
            self.boundary_target_yaw = self.boundary_trigger_yaw
            self.get_logger().info(
                f"[RTKNav] 边界矫正启动：后方双侧触发(yaw={self.boundary_trigger_yaw:.1f}°)，"
                f"直接前进远离 [锁定]")
        elif direction == 'left':
            self.boundary_target_yaw = self.normalize_angle(
                self.boundary_trigger_yaw - TURN_AWAY_DEG)
            self.boundary_correct_state = BoundaryCorrectState.TURNING
            self.boundary_correct_start_time = current_time
            self.get_logger().info(
                f"[RTKNav] 边界矫正启动：左侧触发 → 左转避障 "
                f"(trigger={self.boundary_trigger_yaw:.1f}° → target={self.boundary_target_yaw:.1f}°) [锁定]")
        else:  # 'right'
            self.boundary_target_yaw = self.normalize_angle(
                self.boundary_trigger_yaw + TURN_AWAY_DEG)
            self.boundary_correct_state = BoundaryCorrectState.TURNING
            self.boundary_correct_start_time = current_time
            self.get_logger().info(
                f"[RTKNav] 边界矫正启动：右侧触发 → 右转避障 "
                f"(trigger={self.boundary_trigger_yaw:.1f}° → target={self.boundary_target_yaw:.1f}°) [锁定]")

        if not self.boundary_stop_published:
            self.publish_stop_speed()
            self.boundary_stop_published = True

    def _reset_boundary_correction(self):
        """清空所有边界矫正状态，回到 IDLE。"""
        self.boundary_correct_state = BoundaryCorrectState.IDLE
        self.boundary_correct_start_time = None
        self.boundary_correct_direction = None
        self.boundary_correct_locked = False
        self.boundary_trigger_yaw = 0.0
        self.boundary_target_yaw = 0.0
        self.boundary_slow_count = 0
        self.boundary_correction_start_time = None
        self.boundary_stop_published = False
        self.boundary_active_count = 0
        self.boundary_clear_count = 0

    def _update_blocked_directions(self):
        """
        根据 confirmed_sensors 计算禁止的行驶方向（与 motor_control 一致）。
        - 前部传感器(mid)触发 → 禁止 FORWARD
        - 后部传感器(back)触发 → 禁止 BACKWARD
        - 左侧传感器触发 → 禁止 LEFT
        - 右侧传感器触发 → 禁止 RIGHT
        """
        old = self.blocked_directions
        new = set()
        if 'mid_left' in self.confirmed_sensors or 'mid_right' in self.confirmed_sensors:
            new.add('FORWARD')
        if 'back_left' in self.confirmed_sensors or 'back_right' in self.confirmed_sensors:
            new.add('BACKWARD')
        if 'mid_left' in self.confirmed_sensors or 'back_left' in self.confirmed_sensors:
            new.add('LEFT')
        if 'mid_right' in self.confirmed_sensors or 'back_right' in self.confirmed_sensors:
            new.add('RIGHT')
        self.blocked_directions = new
        if old != new:
            self.get_logger().info(
                f"[RTKNav] 禁止方向更新：{sorted(old) if old else '无'} → {sorted(new) if new else '无'}，"
                f"confirmed={sorted(self.confirmed_sensors)}")

    def _is_motion_blocked(self) -> bool:
        """当前运动方向是否被边界传感器禁止"""
        motion = self._get_current_motion_direction()
        return motion is not None and motion in self.blocked_directions

    def _is_speed_blocked(self, left_speed: float, right_speed: float) -> bool:
        """根据即将发布的轮速判断该动作是否被边界传感器禁止。"""
        motion = self._get_motion_direction_from_speed(left_speed, right_speed)
        return motion is not None and motion in self.blocked_directions

    def _get_motion_direction_from_speed(self, left_speed: float, right_speed: float) -> Optional[str]:
        """将左右轮速度映射为车体运动方向。"""
        if abs(left_speed) < 0.1 and abs(right_speed) < 0.1:
            return None
        if left_speed < 0 and right_speed > 0:
            return 'FORWARD'
        if left_speed > 0 and right_speed < 0:
            return 'BACKWARD'
        if left_speed > 0 and right_speed > 0:
            return 'LEFT'
        if left_speed < 0 and right_speed < 0:
            return 'RIGHT'
        return None

    def _get_current_motion_direction(self) -> Optional[str]:
        """
        基于 NavState 和两轮速度判断当前运动方向。
        返回 'FORWARD' / 'BACKWARD' / 'LEFT' / 'RIGHT' / None。

        两轮速度约定（与 calibrate_heading_at_waypoint 一致）：
          左=-v, 右=+v → FORWARD
          左=+v, 右=-v → BACKWARD
          左=+v, 右=+v → LEFT（原地左转）
          左=-v, 右=-v → RIGHT（原地右转）
        """
        nav_state = self.nav_context.get("nav_state")
        # NavState 优先判断
        if nav_state == NavState.WAYPOINT_MOVE:
            return 'FORWARD'
        if nav_state == NavState.INITIAL_MOVE:
            return 'FORWARD'
        if nav_state in (NavState.IDLE, NavState.COMPLETED, NavState.PAUSE):
            return None

        # WAYPOINT_CALIB：根据 heading_error 判断旋转方向（比轮速更准确，可在启动前判断）
        if nav_state == NavState.WAYPOINT_CALIB:
            target = self.nav_context.get("calib_target_heading")
            if target is not None:
                hdg_err = self.get_heading_error(target)
                if abs(hdg_err) <= RTK_HEADING_TOLERANCE:
                    return None
                return 'RIGHT' if hdg_err > 0 else 'LEFT'

        # 兜底：根据最近发布的电机速度判断
        if hasattr(self, '_last_motor_left') and hasattr(self, '_last_motor_right'):
            return self._get_motion_direction_from_speed(
                self._last_motor_left, self._last_motor_right)
        return None

    def _retreat_to_waypoint(self, waypoint: Tuple[float, float, float],
                             retreat_speed: float = 2.0,
                             distance_threshold: float = 0.2,
                             timeout: float = 30.0) -> Generator[Tuple[float, float], None, None]:
        """
        打滑撤退：背对航点反方向 → 后退归位。
        校准旋转中传感器触发时调用，通过GPS距离闭环回到航点。

        Args:
            waypoint: 目标航点 (lon, lat, heading)
            retreat_speed: 后退速度（电机指令值）
            distance_threshold: 距离阈值(米)，小于此值视为已归位
            timeout: 超时(秒)
        """
        lon, lat, heading = waypoint
        anti_heading = self.normalize_angle(heading + 180.0)
        start_time = time.time()

        # Phase 1: 转向反方向（背对航点）
        # 旋转中若被边界阻挡，根据 blocked_directions 直线远离创造空间，再继续旋转。
        # 远离方向独立于 _determine_boundary_direction：
        #   BACKWARD被禁 → 前进远离后方；FORWARD被禁 → 后退远离前方；仅侧方 → 前进。
        ESCAPE_DURATION = 1.5       # 单次远离持续时间（秒）
        ESCAPE_SPEED = 3.0          # 远离速度
        # 临时更新 calib_target_heading，使 _is_motion_blocked() 能正确判断
        # retreat P1 的实际旋转方向（anti_heading），而非原始校准目标
        saved_calib_target = self.nav_context.get("calib_target_heading")
        self.nav_context["calib_target_heading"] = anti_heading
        try:
            self.get_logger().info(f"[RTKNav] 撤退P1: 转向反方向 {anti_heading:.1f}°")
            calib_gen = self.calibrate_heading_at_waypoint(anti_heading)
            while True:
                try:
                    left_speed, right_speed = next(calib_gen)
                except StopIteration as e:
                    if not e.value:
                        self.get_logger().warn("[RTKNav] 撤退P1转向未精确到位，继续后退")
                    break

                if time.time() - start_time > timeout:
                    self.get_logger().error("[RTKNav] 撤退P1超时")
                    yield (0.0, 0.0)
                    return

                # 旋转中被边界阻挡 → 根据 blocked_directions 直接选择安全远离方向
                if self._is_motion_blocked() and not self.boundary_correct_locked:
                    if 'BACKWARD' in self.blocked_directions:
                        escape_speeds = (-ESCAPE_SPEED, ESCAPE_SPEED)   # 前进远离后方
                        dir_label = "前进"
                    elif 'FORWARD' in self.blocked_directions:
                        escape_speeds = (ESCAPE_SPEED, -ESCAPE_SPEED)   # 后退远离前方
                        dir_label = "后退"
                    else:
                        escape_speeds = (-ESCAPE_SPEED, ESCAPE_SPEED)   # 仅侧方触发→前进
                        dir_label = "前进"

                    self.get_logger().warn(
                        f"[RTKNav] 撤退P1旋转触发传感器(blocked={sorted(self.blocked_directions)})，"
                        f"直线{dir_label}远离{ESCAPE_DURATION}s")
                    t0 = time.time()
                    while time.time() - t0 < ESCAPE_DURATION:
                        if time.time() - start_time > timeout:
                            self.get_logger().error("[RTKNav] 撤退P1超时")
                            yield (0.0, 0.0)
                            return
                        if self.boundary_correct_locked:
                            yield self.get_boundary_correct_speed()
                            continue
                        if not self._is_motion_blocked():
                            break  # 传感器释放，提前结束远离
                        yield escape_speeds
                    yield (0.0, 0.0)
                    continue  # 远离完成，继续旋转（heading_error 实时重算）

                yield (left_speed, right_speed)
        finally:
            self.nav_context["calib_target_heading"] = saved_calib_target

        # Phase 2: 后退归位（GPS闭环 + 距离比例速度 + 航向修正）
        self.get_logger().info(f"[RTKNav] 撤退P2: 后退归位 (目标<{distance_threshold}m)")
        last_dist = float('inf')
        while True:
            dist = self.calc_distance_to_waypoint(waypoint)
            if dist < distance_threshold:
                self.get_logger().info(f"[RTKNav] 撤退P2完成: dist={dist:.3f}m")
                break
            # 距离不收敛 → 可能打滑，降速
            if dist > last_dist + 0.05:
                self.get_logger().warn(f"[RTKNav] 撤退距离反向增长({last_dist:.3f}→{dist:.3f})，降速")
            last_dist = dist
            if time.time() - start_time > timeout:
                self.get_logger().warn(f"[RTKNav] 撤退P2超时: dist={dist:.3f}m")
                break
            # 距离比例速度缩放（参考LOW_DISTANCE逻辑，避免打滑/过冲）
            if dist < LOW_DISTANCE:
                speed_scale = max(0.5, dist / LOW_DISTANCE)
            else:
                speed_scale = 1.0
            effective_speed = retreat_speed * speed_scale
            # 航向修正：保持背对航点方向
            hdg_err = self.get_heading_error(anti_heading)
            correction = max(-1.0, min(1.0, hdg_err * 0.05))  # P项，限幅±1.0
            left_speed = effective_speed - correction    # +v=后退, correction负→左轮慢→右转
            right_speed = -effective_speed - correction   # -v=后退, correction负→右轮快→右转
            # 防止换向（最低保持0.3避免完全停止）
            left_speed = max(0.3, left_speed)
            right_speed = min(-0.3, right_speed)
            yield (left_speed, right_speed)
            self.get_logger().debug(f"[RTKNav] 撤退: dist={dist:.3f}m, speed={effective_speed:.1f}, hdg_err={hdg_err:.1f}°")

        yield (0.0, 0.0)
        self.get_logger().info("[RTKNav] 撤退完成，可重新执行校准")

    def _calibrate_with_boundary_retreat(
            self,
            target_heading: float,
            target_waypoint: Optional[Tuple[float, float, float]],
            label: str) -> Generator[Tuple[float, float], None, bool]:
        """
        统一的原地校准保护入口。
        任何原地旋转校准都临时按 WAYPOINT_CALIB 判断运动方向，避免超声波触发被绕过。
        """
        saved_nav_state = self.nav_context.get("nav_state")
        saved_calib_target = self.nav_context.get("calib_target_heading")
        self.nav_context["nav_state"] = NavState.WAYPOINT_CALIB
        self.nav_context["calib_target_heading"] = target_heading

        try:
            while rclpy.ok():
                calib_gen = self.calibrate_heading_at_waypoint(target_heading)
                while rclpy.ok():
                    try:
                        left_speed, right_speed = next(calib_gen)
                    except StopIteration as e:
                        return bool(e.value)

                    if self._is_motion_blocked() or self.boundary_correct_locked:
                        if self.boundary_correct_locked:
                            left_speed, right_speed = self.get_boundary_correct_speed()
                        elif target_waypoint is None:
                            self.get_logger().warn(
                                f"[RTKNav] {label}触发传感器({sorted(self.blocked_directions)})，"
                                "无目标航点，启动边界矫正")
                            left_speed, right_speed = self.get_boundary_correct_speed()
                        else:
                            self.get_logger().warn(
                                f"[RTKNav] {label}触发传感器({sorted(self.blocked_directions)})，"
                                "执行GPS撤退回航点")
                            retreat_gen = self._retreat_to_waypoint(target_waypoint)
                            try:
                                while True:
                                    left_speed, right_speed = next(retreat_gen)
                                    if self.boundary_correct_locked:
                                        left_speed, right_speed = self.get_boundary_correct_speed()
                                    elif self._is_speed_blocked(left_speed, right_speed):
                                        self.get_logger().warn(
                                            f"[RTKNav] {label}撤退动作被边界禁止"
                                            f"({sorted(self.blocked_directions)})，启动边界矫正")
                                        left_speed, right_speed = self.get_boundary_correct_speed()
                                    yield (left_speed, right_speed)
                            except StopIteration:
                                pass
                            self.get_logger().info(f"[RTKNav] {label}撤退完成，重新执行航向校准")
                            break

                    yield (left_speed, right_speed)
            return False
        finally:
            self.nav_context["nav_state"] = saved_nav_state
            self.nav_context["calib_target_heading"] = saved_calib_target

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
        self.sensors_status = (
            int(self.front_left)
            | (int(self.front_right) << 1)
            | (int(self.mid_left) << 2)
            | (int(self.mid_right) << 3)
            | (int(self.back_left) << 4)
            | (int(self.back_right) << 5)
        )
        # 4传感器分层触发（mid/back × 左右）
        if self.current_control_mode == ControlMode.AUTO_CLEANING and not self.boundary_correct_locked:
            same_row = (self.mid_left and self.mid_right) or (self.back_left and self.back_right)
            same_side = (self.mid_left and self.back_left) or (self.mid_right and self.back_right)
            active_set = set()
            if self.mid_left: active_set.add('mid_left')
            if self.mid_right: active_set.add('mid_right')
            if self.back_left: active_set.add('back_left')
            if self.back_right: active_set.add('back_right')
            num_active = len(active_set)
            spatial_consensus = same_row or same_side or num_active >= 3

            changed = False
            # 已确认但当前帧不再触发的传感器 → 立即移除
            stale = self.confirmed_sensors - active_set
            if stale:
                self.confirmed_sensors -= stale
                changed = True
                self.get_logger().info(f"[RTKNav] 边界传感器释放：{sorted(stale)}")

            if spatial_consensus:
                self.boundary_active_count += 1
                self.boundary_clear_count = 0
                self.boundary_slow_count = 0
                if self.boundary_active_count >= BOUNDARY_TRIGGER_CONFIRM_FRAMES:
                    if active_set != self.confirmed_sensors:
                        self.confirmed_sensors = active_set
                        changed = True
                        self.get_logger().warn(
                            f"[RTKNav] 边界快触发：{sorted(active_set)}，"
                            f"mid=({self.mid_left},{self.mid_right}), back=({self.back_left},{self.back_right})")
            elif num_active == 1:
                self.boundary_slow_count += 1
                self.boundary_active_count = 0
                self.boundary_clear_count = 0
                if self.boundary_slow_count >= BOUNDARY_SLOW_PERSIST_FRAMES:
                    sensor_name = next(iter(active_set))
                    if sensor_name not in self.confirmed_sensors:
                        self.confirmed_sensors.add(sensor_name)
                        changed = True
                        self.get_logger().warn(
                            f"[RTKNav] 边界慢触发：{sensor_name} 持续{BOUNDARY_SLOW_PERSIST_FRAMES}帧 "
                            f"({BOUNDARY_SLOW_PERSIST_FRAMES/10:.1f}s)")
            else:
                self.boundary_active_count = 0
                self.boundary_slow_count = 0
                self.boundary_clear_count += 1
                if self.boundary_clear_count >= BOUNDARY_CLEAR_CONFIRM_FRAMES:
                    if self.confirmed_sensors:
                        self.confirmed_sensors.clear()
                        changed = True
                        self.get_logger().info("[RTKNav] 边界传感器全部释放，恢复导航")

            if changed:
                self._update_blocked_directions()
        else:
            self.boundary_active_count = 0
            self.boundary_slow_count = 0
            self.boundary_clear_count = 0
            if not self.boundary_correct_locked:
                if self.confirmed_sensors:
                    self.confirmed_sensors.clear()
                    self._update_blocked_directions()
    # def unloading_gps_callback(self, msg: Vector3):
    #     loading_lon = msg.x
    #     loading_lat = msg.y
    #     heading = msg.z
    #     self.loading_waypoint = (loading_lon, loading_lat, heading)
    #     self.get_logger().info(f"[RTKNav] 收到出仓GPS坐标: 经度={loading_lon}, 纬度={loading_lat}")
    def load_builtin_loading_gps(self) -> Optional[Tuple[float, float, float]]:
        loading_gps = self.get_parameter("loading_gps").value
        if not isinstance(loading_gps, (list, tuple)) or len(loading_gps) != 3:
            self.get_logger().error("[RTKNav] loading_gps参数格式错误，应为[lon, lat, heading]，将不追加固定进仓点")
            return None

        try:
            lon, lat, heading = (float(loading_gps[0]), float(loading_gps[1]), float(loading_gps[2]))
        except (TypeError, ValueError):
            self.get_logger().error("[RTKNav] loading_gps参数无法转换为浮点数，将不追加固定进仓点")
            return None

        if lon == 0.0 and lat == 0.0:
            self.get_logger().warn("[RTKNav] loading_gps尚未配置，导航结束后不会追加固定进仓点")
            return None

        heading = heading % 360.0
        self.get_logger().info(f"[RTKNav] 已加载固定进仓GPS: 经度={lon:.6f}, 纬度={lat:.6f}, 航向={heading:.2f}°")
        return (lon, lat, heading)

    def unloading_gps_callback(self, msg: Vector3):
        self.get_logger().info(
            f"[RTKNav] 收到出仓GPS坐标但不再作为进仓航点: 经度={msg.x:.6f}, 纬度={msg.y:.6f}, 航向={msg.z:.2f}°；"
            f"固定进仓点={self.loading_waypoint}"
        )
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
            file_pattern = re.compile(r'^\d{3}[-_].*\.txt$')
            all_files = [f for f in os.listdir(self.path_dir) if file_pattern.match(f)]
            
            if not all_files:
                self.get_logger().warn("[RTKNav] 路径目录下未找到符合规则的路径文件")
                return None
            
            # 2. 按文件名最前面的3位数字序号排序（提取如002, 004等）
            def extract_sequence_number(filename: str) -> int:
                match = re.search(r'^(\d{3})[-_]', filename)
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
            
            # 5. 最后一个文件时循环回到第一个
            if current_idx >= total_files - 1:
                first_file = all_files[0]
                self.get_logger().info(f"[RTKNav] 已执行到最后一个路径文件（{current_file}）, 循环回到第一个: {first_file}")
                return os.path.join(self.path_dir, first_file)
            
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
                            start_idx = len(self.waypoints)
                            self.brush_start_indices.append(start_idx)
                            self.get_logger().info(f"[RTKNav] 检测到#start标记，滚刷将在航点{start_idx}开启")
                            if start_idx == 0:
                                self.brush_active = True
                        elif 'stop' in comment:
                            # 记录滚刷关闭的下一个航点索引
                            stop_idx = len(self.waypoints)
                            self.brush_stop_indices.append(stop_idx)
                            self.get_logger().info(f"[RTKNav] 检测到#stop标记，滚刷将在航点{stop_idx}关闭")
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
            if self.brush_start_indices:
                self.get_logger().info(f"[RTKNav] 滚刷开启航点索引: {self.brush_start_indices}")
            if self.brush_stop_indices:
                self.get_logger().info(f"[RTKNav] 滚刷关闭航点索引: {self.brush_stop_indices}")
            return True
        except Exception as e:
            self.get_logger().error(f"解析RTK文件失败：{str(e)}")
            return False
    def reset_base_loading_waypoint(self):
        """重置基准出仓点（用于更换任务场景）"""
        self.base_loading_waypoint = None
        self.offset_calculated = False
        self.get_logger().info("[RTKNav] 已重置基准出仓点，下次接收将重新缓存")

    def handle_rtk_data_timeout(self) -> bool:
        """RTK数据超时 + 航向角异常超时检测，触发时停车并暂停导航"""
        now = time.monotonic()

        # —— 航向角异常超时检测（独立于RTK数据超时） ——
        if self.heading_abnormal_start_time is not None:
            heading_abnormal_elapsed = now - self.heading_abnormal_start_time
            if heading_abnormal_elapsed > HEADING_ABNORMAL_TIMEOUT:
                if not self.heading_timed_out:
                    # 仅在主动导航状态（非空闲/暂停/完成/校准中）时触发heading_timeout暂停
                    # 校准中(WAYPOINT_CALIB)有自己的40s超时，不应被heading_timeout打断
                    if self.nav_context["nav_state"] not in [NavState.IDLE, NavState.PAUSE, NavState.COMPLETED, NavState.WAYPOINT_CALIB]:
                        self.nav_context["pre_pause_state"] = self.nav_context["nav_state"]
                        self.nav_context["pause_reason"] = "heading_timeout"
                        self.nav_context["brush_active"] = self.brush_active
                        self.nav_context["nav_state"] = NavState.PAUSE
                        self.publish_nav_state(NavState.PAUSE)
                        self.nav_running = False
                        self.heading_timed_out = True
                        self.publish_stop_speed()
                        self.get_logger().warn(
                            f"[航向异常超时] 航向角异常已持续{heading_abnormal_elapsed:.1f}s，已停车并暂停导航"
                        )
                        self.last_rtk_timeout_log_time = now
                        self._last_heading_recovery_check = now
                        self.set_rtk_error_bits(ERROR_RTK_TIMEOUT)
                        return True
                # 周期性恢复检查：检查航向是否已恢复正常
                if now - self._last_heading_recovery_check >= HEADING_RECOVERY_CHECK_INTERVAL:
                    self._last_heading_recovery_check = now
                    if self._is_heading_normal():
                        self.heading_abnormal_start_time = None
                        self.heading_timed_out = False
                        self.clear_rtk_error_bits(ERROR_RTK_TIMEOUT)
                        # 检查跌落故障是否活跃，避免倾斜状态下恢复导航
                        if self.nav_context.get("tilt_fault", False):
                            self.get_logger().warn("[航向恢复] 航向已恢复但跌落故障仍活跃，保持PAUSE等待倾斜恢复")
                            return False
                        if (
                            self.nav_context["nav_state"] == NavState.PAUSE
                            and self.nav_context.get("pause_reason") == "heading_timeout"
                        ):
                            self.nav_context["nav_state"] = self.nav_context["pre_pause_state"]
                            self.nav_context["pause_reason"] = None
                            self.brush_active = self.nav_context.get("brush_active", False)
                            if self.brush_active:
                                self.publish_brush_speed(RTK_BRUSH_SPEED)
                            else:
                                self.publish_brush_speed(0.0)
                            self.get_logger().info("[航向恢复] 航向角已恢复正常，自动恢复导航")
                        self.nav_running = True
                    return False
                if now - self.last_rtk_timeout_log_time >= RTK_TIMEOUT_LOG_INTERVAL:
                    self.publish_stop_speed()
                    self.get_logger().warn(
                        f"[航向异常超时] 航向角仍异常，已持续{heading_abnormal_elapsed:.1f}s，保持停车"
                    )
                    self.last_rtk_timeout_log_time = now
                self.set_rtk_error_bits(ERROR_RTK_TIMEOUT)
                return True

        # —— RTK数据超时检测 ——
        if self.last_wtrtk_time is None:
            return False

        elapsed = now - self.last_wtrtk_time
        if elapsed <= RTK_DATA_TIMEOUT:
            return False

        if not self.rtk_data_timed_out:
            if self.nav_context["nav_state"] not in [NavState.IDLE, NavState.PAUSE, NavState.COMPLETED]:
                self.nav_context["pre_pause_state"] = self.nav_context["nav_state"]
                self.nav_context["pause_reason"] = "rtk_timeout"
                self.nav_context["brush_active"] = self.brush_active
                self.nav_context["nav_state"] = NavState.PAUSE
                self.publish_nav_state(NavState.PAUSE)
            self.nav_running = False
            self.rtk_data_timed_out = True
            self.publish_stop_speed()
            self.get_logger().warn(
                f"[RTK数据超时] {elapsed:.2f}s未收到/wtrtk_data，已停车并暂停导航"
            )
            self.last_rtk_timeout_log_time = now
        elif now - self.last_rtk_timeout_log_time >= RTK_TIMEOUT_LOG_INTERVAL:
            self.publish_stop_speed()
            self.get_logger().warn(
                f"[RTK数据超时] 仍未收到/wtrtk_data，已持续{elapsed:.2f}s，保持停车"
            )
            self.last_rtk_timeout_log_time = now

        self.set_rtk_error_bits(ERROR_RTK_TIMEOUT)
        return True

    def _is_heading_normal(self) -> bool:
        """检查是否可恢复导航，用于航向异常超时后的恢复判断。

        不检查航向误差绝对值——IMU 停车后持续漂移，误差可能始终偏大导致死锁。
        只检查 IMU 数据是否仍在更新：数据新鲜说明 IMU 未死机/卡死，
        Stanley 控制器或航向重校准可主动纠正漂移误差。
        """
        if self.last_wtrtk_time is not None:
            return (time.monotonic() - self.last_wtrtk_time) <= RTK_DATA_TIMEOUT
        return True  # 无法判断时假定正常，避免永久阻塞

    def heading_callback(self, msg: WTRTK) -> None:
        self.last_wtrtk_time = time.monotonic()
        if self.rtk_data_timed_out:
            self.get_logger().info("[RTK数据恢复] 已重新收到/wtrtk_data")
            self.rtk_data_timed_out = False

        ins_heading_deg = msg.ins_heading
        self.imu_yaw = ins_heading_deg  + self.rtk_install_offset # + x degree
        self.imu_yaw = math.fmod(self.imu_yaw + 180.0, 360.0) - 180.0
        self.imu_initialized = True
        imu_msg= Float32()
        imu_msg.data = self.imu_yaw
        self.imu_heading_pub.publish(imu_msg)

        # —— 航向稳定性追踪（排除主动旋转的状态：航点校准）——
        nav_state = self.nav_context.get("nav_state", NavState.IDLE)
        tracking_active = nav_state != NavState.WAYPOINT_CALIB

        if tracking_active:
            now = time.monotonic()
            vehicle_heading_360 = (self.imu_yaw + 360) % 360
            self._heading_stability_history.append((now, vehicle_heading_360))
            while (self._heading_stability_history
                   and now - self._heading_stability_history[0][0] > HEADING_STABILITY_WINDOW):
                self._heading_stability_history.popleft()

            is_stable = False
            window_dur = now - self._heading_stability_history[0][0] if self._heading_stability_history else 0.0
            samples_enough = len(self._heading_stability_history) >= 20
            time_enough = window_dur >= HEADING_STABILITY_WINDOW - 0.5
            if samples_enough and time_enough:
                headings = [h for _, h in self._heading_stability_history]
                h_min, h_max = min(headings), max(headings)
                h_range = h_max - h_min
                # 不限制固定角度范围：出仓后只要IMU无漂移（波动小）即视为稳定，
                # 无论车头朝哪个方向。避免遥控接管后角度变化导致永远无法通过校验。
                is_stable = h_range <= HEADING_STABILITY_RANGE

            if is_stable != self._last_heading_stable:
                if is_stable:
                    self.get_logger().info(f"[航向稳定] ins_heading 5s内波动≤{HEADING_STABILITY_RANGE}°，"
                                           f"无漂移判定为稳定（不限制固定角度）")
                else:
                    self.get_logger().warn(f"[航向不稳定] ins_heading 失去稳定（5s内波动>{HEADING_STABILITY_RANGE}°）")
            self._last_heading_stable = is_stable

        stable_msg = Bool()
        stable_msg.data = self._last_heading_stable
        self.heading_stable_pub.publish(stable_msg)

        position_status = msg.position_status
        orientation_status = msg.fix_status
        position_data_valid = msg.position_data_valid

        if position_status < 0:
            self.get_logger().warn("GPS信号无效")
            self.last_gps_status = -1

        status_map = {0: "未定位", 1: "单点", 2: "差分", 5: "RTK Float", 4: "RTK Fixed"}
        if position_status != self.last_gps_status:
            # self.get_logger().info(f"GPS状态：{status_map[position_status]}")
            self.last_gps_status = position_status
        self.last_orientation_status = orientation_status
        self.position_data_valid = position_data_valid
        self.rtk_solution_ready = (
            position_data_valid
            and position_status == 4
            and orientation_status == 4
        )

        if not self.rtk_data_timed_out and not self.heading_timed_out:
            self.clear_rtk_error_bits(ERROR_RTK_TIMEOUT)

        if not self.rtk_solution_ready:
            self.set_rtk_error_bits(ERROR_RTK_NOT_FIXED)
            if hasattr(self, 'nav_context') and self.nav_context["nav_state"] not in [NavState.IDLE, NavState.PAUSE, NavState.COMPLETED]:
                self.nav_context["pre_pause_state"] = self.nav_context["nav_state"]
                self.nav_context["pause_reason"] = "rtk_not_fixed"
                self.nav_context["brush_active"] = self.brush_active
                self.get_logger().warn(
                    f"[RTK状态] 定位={status_map.get(position_status, '未知')}，"
                    f"定向={status_map.get(orientation_status, '未知')}，"
                    f"GGA有效={position_data_valid}，暂停导航"
                    f"（保存状态：{self.nav_context['pre_pause_state']}）"
                )
                self.nav_context["nav_state"] = NavState.PAUSE
                self.nav_running = False
                stop_speed = Vector3()
                self.publish_stop_speed()
            elif (
                hasattr(self, 'nav_context')
                and self.nav_context["nav_state"] == NavState.PAUSE
                and self.nav_context.get("pause_reason") == "rtk_timeout"
            ):
                # 数据恢复但定位仍非固定解：由超时暂停转为非固定解暂停，继续停车。
                self.nav_context["pause_reason"] = "rtk_not_fixed"
                self.nav_running = False
                self.publish_stop_speed()
                self.get_logger().warn(
                    f"[RTK状态] 数据已恢复但定位={status_map.get(position_status, '未知')}，"
                    f"定向={status_map.get(orientation_status, '未知')}，"
                    f"GGA有效={position_data_valid}，"
                    "暂停原因切换为rtk_not_fixed"
                )
        else:
            if hasattr(self, 'nav_context') and self.nav_context["nav_state"] == NavState.PAUSE and self.heading_timed_out:
                return  # 航向异常导致的暂停，不清理错误码也不恢复导航
            self.clear_rtk_error_bits(ERROR_RTK_NOT_FIXED)
            if (
                hasattr(self, 'nav_context')
                and self.nav_context["nav_state"] == NavState.PAUSE
                and self.nav_context.get("pause_reason") == "rtk_not_fixed"
            ):
                self.get_logger().info("[RTK状态] 定位与定向均恢复固定解，自动恢复导航")
                self.nav_context["nav_state"] = self.nav_context["pre_pause_state"]
                self.nav_context["pause_reason"] = None
                self.brush_active = self.nav_context.get("brush_active", False)
                if self.brush_active:
                    self.publish_brush_speed(RTK_BRUSH_SPEED)
                else:
                    self.publish_brush_speed(0.0)
                self.nav_running = True

        # —— 倾斜/跌落检测（基于 angle_x/angle_y），仅在 AUTO_CLEANING 模式生效 ——
        if self.current_control_mode == ControlMode.AUTO_CLEANING:
            self._angle_x_history.append(msg.angle_x)
            self._angle_y_history.append(msg.angle_y)

            over_x = abs(msg.angle_x) > TILT_ANGLE_THRESHOLD
            over_y = abs(msg.angle_y) > TILT_ANGLE_THRESHOLD
            over_threshold = over_x or over_y

            # 突变检测：当前值 vs 1s滑动窗口中位数，过滤IMU零偏缓慢漂移
            already_counting = self.nav_context["tilt_confirm_count"] > 0
            sudden = False
            if len(self._angle_x_history) >= TILT_BASELINE_SAMPLES:
                sorted_x = sorted(self._angle_x_history)
                sorted_y = sorted(self._angle_y_history)
                baseline_x = sorted_x[len(sorted_x) // 2]
                baseline_y = sorted_y[len(sorted_y) // 2]
                sudden = (abs(msg.angle_x - baseline_x) > TILT_SUDDEN_DELTA
                          or abs(msg.angle_y - baseline_y) > TILT_SUDDEN_DELTA)

            # 倾斜判定：超阈值 AND (突变 OR 已在计数中，锁存防止持续倾斜时基线追上)
            is_tilted = over_threshold and (sudden or already_counting)

            if is_tilted:
                self.nav_context["tilt_normal_count"] = 0
                self.nav_context["tilt_confirm_count"] += 1
                if self.nav_context["tilt_confirm_count"] >= TILT_CONFIRM_FRAMES and not self.nav_context["tilt_fault"]:
                    self.nav_context["tilt_fault"] = True
                    self.last_tilt_time = time.monotonic()
                    if self.nav_context["nav_state"] not in [NavState.IDLE, NavState.PAUSE, NavState.COMPLETED]:
                        self.nav_context["pre_pause_state"] = self.nav_context["nav_state"]
                        self.nav_context["pause_reason"] = "tilt_fault"
                        self.nav_context["brush_active"] = self.brush_active
                        self.nav_context["nav_state"] = NavState.PAUSE
                        self.publish_nav_state(NavState.PAUSE)
                    self.nav_running = False
                    self.publish_stop_speed()
                    self.set_rtk_error_bits(ERROR_TILT_FAULT)
                    self.get_logger().error(
                        f"[倾斜故障] angle_x={msg.angle_x:.2f}°, angle_y={msg.angle_y:.2f}° "
                        f"连续{self.nav_context['tilt_confirm_count']}帧超阈值({TILT_ANGLE_THRESHOLD}°)，已停车并暂停导航"
                    )
            else:
                self.nav_context["tilt_confirm_count"] = 0
                if self.nav_context["tilt_fault"]:
                    self.nav_context["tilt_normal_count"] += 1
                    if self.nav_context["tilt_normal_count"] >= TILT_RECOVERY_FRAMES:
                        self.nav_context["tilt_fault"] = False
                        self.nav_context["tilt_normal_count"] = 0
                        self.last_tilt_duration = time.monotonic() - self.last_tilt_time
                        self.clear_rtk_error_bits(ERROR_TILT_FAULT)
                        self.multi_waypoint_generator = None
                        self.nav_running = False  # 确保重建条件成立
                        if self.last_tilt_duration < TILT_SHORT_DURATION:
                            self.get_logger().info(
                                f"[倾斜恢复] 倾角已恢复，倾斜仅持续{self.last_tilt_duration:.1f}s"
                                f"(<{TILT_SHORT_DURATION}s)，将跳过稳定等待"
                            )
                        else:
                            self.get_logger().info("[倾斜恢复] 倾角已恢复正常，等待生成器重建进入稳定等待流程")

        raw_lon = msg.ins_longitude
        raw_lat = msg.ins_latitude
        
        # if self.current_gps and self.current_gps != [0.0, 0.0]:
        #     lon_diff = abs(raw_lon - self.current_gps[0])
        #     lat_diff = abs(raw_lat - self.current_gps[1])
        
        # self.gps_cache.append((raw_lon, raw_lat))
        # if len(self.gps_cache) > GPS_SMOOTH_WINDOW:
        #     self.gps_cache.pop(0)
        # smooth_lon = sum([x[0] for x in self.gps_cache]) / len(self.gps_cache)
        # smooth_lat = sum([x[1] for x in self.gps_cache]) / len(self.gps_cache)
        
        self.current_gps = (raw_lon, raw_lat)
        self.current_lon = raw_lon
        self.current_lat = raw_lat
    
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
        car_gps_msg.status.status = msg.position_status
        car_gps_msg.status.service = NavSatStatus.SERVICE_GPS
        car_gps_msg.latitude = car_lat
        car_gps_msg.longitude = car_lon
        car_gps_msg.altitude = msg.ins_altitude  # 使用WTRTK的ins_altitude字段
        self.car_center_gps_pub.publish(car_gps_msg)

        self.current_lat = car_lat
        self.current_lon = car_lon
        self.current_gps = (car_lon, car_lat)
        self.get_logger().debug(f"当前车体中心坐标：经度={car_lon:.6f}, 纬度={car_lat:.6f}")

    def get_target_waypoint(self, current_waypoint_idx: int = None) -> Optional[Tuple[float, float, float]]:
        """获取当前目标航点（含航向角）。当前任务结束后只记录下一个路径，等待下一次任务启动。"""
        idx = current_waypoint_idx if current_waypoint_idx is not None else self.current_waypoint_idx
        
        # 检查是否到达最后一个航点
        if idx >= len(self.waypoints):
            self.get_logger().info("[RTKNav] 已到达当前路径文件的最后一个航点, 本轮任务准备返回进仓点")

            if self.pending_next_path_file is None:
                self.pending_next_path_file = self.get_next_path_file()
                if self.pending_next_path_file:
                    self.get_logger().info(f"[RTKNav] 已记录下一任务路径，待本轮COMPLETED后切换: {self.pending_next_path_file}")
                else:
                    self.get_logger().info("[RTKNav] 没有更多路径文件，本轮返仓后结束全部路径")

            if hasattr(self, 'loading_waypoint') and self.loading_waypoint is not None and not self.return_to_loading_added:
                self.get_logger().info(f"[RTKNav] 开始执行返回进仓点：{self.loading_waypoint}")
                self.waypoints.append(self.loading_waypoint)
                self.waypoint_areas.append("")
                self.current_waypoint_idx = idx
                self.return_to_loading_added = True
                self.get_logger().info(f"[RTKNav] 进仓点追加成功，当前航点索引：{self.current_waypoint_idx}，总航点数：{len(self.waypoints)}")
                return self.loading_waypoint

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


    def normalize_angle(self, angle: float) -> float:
        """
        角度归一化到[-180°, 180°]
        """
        return ((angle + 180.0) % 360.0) - 180.0

    def calculate_path_bearing(self, start_lon: float, start_lat: float,
                              end_lon: float, end_lat: float) -> float:
        """
        计算路径段的方向角（从起点到终点的方位角）
        """
        x1, y1 = self.latlon_to_utm(start_lat, start_lon)
        x2, y2 = self.latlon_to_utm(end_lat, end_lon)
        dx = x2 - x1
        dy = y2 - y1
        bearing = math.degrees(math.atan2(dx, dy))
        return self.normalize_angle(bearing)

    def calculate_lateral_error(self, current_pos: Tuple[float, float],
                               path_start: Tuple[float, float],
                               path_end: Tuple[float, float]) -> float:
        """
        计算横向误差（cross-track error）
        """
        cx, cy = self.latlon_to_utm(current_pos[1], current_pos[0])
        ax, ay = self.latlon_to_utm(path_start[1], path_start[0])
        bx, by = self.latlon_to_utm(path_end[1], path_end[0])
        dx = bx - ax
        dy = by - ay
        len_sq = dx * dx + dy * dy
        if len_sq < 0.0001:
            return math.hypot(cx - ax, cy - ay)
        ap_dx = cx - ax
        ap_dy = cy - ay
        t = max(0.0, min(1.0, (ap_dx * dx + ap_dy * dy) / len_sq))
        cross = ap_dx * dy - ap_dy * dx
        lateral_error = cross / math.sqrt(len_sq)
        return lateral_error

    def _get_projection_ratio(self, pos, path_start, path_end):
        """返回未clamp的投影比例 t：0=起点, 1=终点, >1=已越过终点"""
        cx, cy = self.latlon_to_utm(pos[1], pos[0])
        ax, ay = self.latlon_to_utm(path_start[1], path_start[0])
        bx, by = self.latlon_to_utm(path_end[1], path_end[0])
        dx = bx - ax
        dy = by - ay
        len_sq = dx * dx + dy * dy
        if len_sq < 0.0001:
            return 0.0
        ap_dx = cx - ax
        ap_dy = cy - ay
        return (ap_dx * dx + ap_dy * dy) / len_sq

    def _force_bearing_diverging(self, distance: float) -> bool:
        """force_bearing 直行时检测是否持续背离目标。

        每次接近都刷新历史最近距离；只有连续帧都比最近点远出阈值才累计，
        中间任一帧回到阈值内立即清零，避免稀疏 GPS 抖动跨帧累积误判。
        """
        min_d = self.nav_context.get("force_bearing_min_distance", float('inf'))
        if distance < min_d:
            self.nav_context["force_bearing_min_distance"] = distance
            self.nav_context["distance_increase_count"] = 0
        elif distance > min_d + FORCE_BEARING_DIVERGE_DIST:
            self.nav_context["distance_increase_count"] += 1
            if self.nav_context["distance_increase_count"] >= FORCE_BEARING_DIVERGE_COUNT:
                return True
        else:
            self.nav_context["distance_increase_count"] = 0
        return False

    def get_adaptive_stanley_k(self, velocity, distance_to_target):
        if distance_to_target < 1.3:
            return 0.42
        return 0.45

    def stanley_steering_control(self, current_pos: Tuple[float, float],
                                 current_heading: float,
                                 path_start: Tuple[float, float],
                                 path_end: Tuple[float, float],
                                 path_direction: float,
                                 velocity: float,
                                 distance_to_target: float = float('inf'),
                                 bearing_only: bool = False) -> Tuple[float, float]:
        """
        Stanley控制器计算左右轮速度
        使用自适应K值和横向误差限幅
        velocity: 电机指令值（非真实速度 m/s）
        bearing_only: force_bearing 方位角直行模式，抑制横向项，只用航向误差
                      （path_direction 已是实时指向目标的方位角，横向项基于旧固定
                       路径段会与之冲突，导致自激极限环，故置零）
        """
        heading_error = self.normalize_angle(path_direction - current_heading)
        real_velocity = velocity * SPEED_CMD_TO_MPS
        self.real_velocity = real_velocity
        if bearing_only:
            lateral_error = 0.0
            steering_correction = 0.0
            k = 0.0
        else:
            lateral_error = self.calculate_lateral_error(current_pos, path_start, path_end)
            lateral_error = max(-MAX_LATERAL_ERROR, min(MAX_LATERAL_ERROR, lateral_error))
            k = self.get_adaptive_stanley_k(real_velocity, distance_to_target)
            steering_correction = math.degrees(math.atan(k * lateral_error / max(real_velocity, STANLEY_MIN_SPEED)))
        total_steering = steering_correction - heading_error
        # if abs(heading_error) > 20.0:
        #     total_steering = heading_error * 0.5 + steering_correction * 0.5
        total_steering_clamped = max(min(total_steering, 45.0), -45.0)
        steering_factor = total_steering_clamped / 45.0
        speed_diff = steering_factor * STRAIGHT_MAX_CORRECTION
        if not hasattr(self, '_stanley_log_counter'):
            self._stanley_log_counter = 0
        self._stanley_log_counter += 1
        if self._stanley_log_counter % 5 == 0:
            self.get_logger().info(
                f"[Stanley-DBG] hdg_err={heading_error:.1f}°, lat_err={lateral_error:.3f}m, "
                f"st_corr={steering_correction:.1f}°, total={total_steering:.1f}°, "
                f"real_v={real_velocity:.3f}m/s, k={k:.2f}, path_dir={path_direction:.1f}°, imu={current_heading:.1f}°"
            )
        left_speed = -velocity + speed_diff
        right_speed = velocity + speed_diff
        left_speed = max(min(left_speed, SPEED_LIMIT), -SPEED_LIMIT)
        right_speed = max(min(right_speed, SPEED_LIMIT), -SPEED_LIMIT)
        return (left_speed, right_speed)

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
        # if abs(yaw_error - self.last_yaw_error) > 0.1:
        #     self.get_logger().info(f"直线：yaw_error={yaw_error:.2f}，修正量={correction_clamped:.2f}")
        
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
        if yaw_error_abs > 45:
            return TURN_SPEED_FAST  # type: ignore # 大误差（>45°）：快速转向
        elif yaw_error_abs > 20:
            return TURN_SPEED_MID   # 中误差（5°~30°）：中等速度
        else:
            return TURN_SPEED_SLOW  # 小误差（<5°）：慢速转向，防止超调

    def start_heading_recalibration(self, path_direction: float, heading_error: float, reason: str) -> None:
        """进入航向重新校准，校准目标使用当前有效路径方向。"""
        self.get_logger().warn(
            f"[Stanley] {reason}：hdg_err={heading_error:.1f}°，"
            f"阈值={HEADING_ABNORMAL_THRESHOLD:.1f}°，重新校准到路径方向{path_direction:.1f}°"
        )
        self.nav_context["angle_abnormal_count"] = 0
        self.nav_context["is_angle_recalib"] = True
        self.nav_context["calib_target_heading"] = path_direction
        self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(path_direction)
        self.nav_context["nav_state"] = NavState.WAYPOINT_CALIB
        

    def calibrate_heading_at_waypoint(self, target_heading: float) -> Generator[Tuple[float, float], None, bool]:
        self.clear_rtk_error_bits(ERROR_CALIB_TIMEOUT)
        calib_start_time = self.get_clock().now()
        last_heading_error_deg = None
        stuck_start_time = None
        escalated = False
        STUCK_ERROR_CHANGE = 0.3    # 误差变化小于此值视为卡滞（度）
        STUCK_ESCALATE_TIME = 5.0   # 卡滞超此时长后提速（秒）
        STUCK_ESCALATE_SPEED = 0.7  # 卡滞提速目标转速

        while rclpy.ok():
            heading_error = self.get_heading_error(target_heading)
            heading_error = math.fmod(heading_error + 180.0, 360.0) - 180.0
            heading_error_deg = abs(heading_error)
            # 校准达标
            if heading_error_deg <= RTK_HEADING_TOLERANCE:
                self.get_logger().info(f"航向校准完成！误差：{heading_error_deg:.2f}°")
                self.clear_rtk_error_bits(ERROR_CALIB_TIMEOUT)
                return True

            elapsed_time = (self.get_clock().now() - calib_start_time).nanoseconds / 1e9

            # 卡滞检测：误差不收敛时自动提速到FAST档
            if last_heading_error_deg is not None:
                error_change = abs(heading_error_deg - last_heading_error_deg)
                if error_change < STUCK_ERROR_CHANGE:
                    if stuck_start_time is None:
                        stuck_start_time = self.get_clock().now()
                    stuck_duration = (self.get_clock().now() - stuck_start_time).nanoseconds / 1e9
                    if stuck_duration > STUCK_ESCALATE_TIME and not escalated:
                        self.get_logger().warn(
                            f"[航向校准] 卡滞检测：{stuck_duration:.1f}s内误差变化<{STUCK_ERROR_CHANGE}°，"
                            f"提速至{STUCK_ESCALATE_SPEED}重试"
                        )
                        escalated = True
                else:
                    stuck_start_time = None
            last_heading_error_deg = heading_error_deg

            # 超时处理
            if elapsed_time > HEADING_CALIBRATION_TIMEOUT:
                self.set_rtk_error_bits(ERROR_CALIB_TIMEOUT)
                if heading_error_deg > 5.0:
                    self.get_logger().error(
                        f"[航向校准] 超时且误差仍大({heading_error_deg:.1f}°>5°)，"
                        f"可能机械卡死或阻力过大，暂停导航"
                    )
                    return False
                self.get_logger().warn(f"航向校准超时！误差：{heading_error_deg:.2f}°")
                return True

            # 选择转速：卡滞提速后使用固定提速档
            if escalated:
                turn_speed = STUCK_ESCALATE_SPEED
            else:
                turn_speed = self.get_adaptive_turn_speed(heading_error_deg)

            # 额外的方向修正项（基于航向误差的速度修正）
            correction = self.get_speed_correction(target_heading)

            # 根据修正后的误差计算转向方向
            min_positive_speed = 0.5  # 最小转向速度，草地需更高值克服静摩擦
            if heading_error > 0:
                left_speed = -max(turn_speed - correction, min_positive_speed)
                right_speed = -max(turn_speed - correction, min_positive_speed)
            else:
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
        heading_aligned = False
        last_heading = None
        try:
            heading_aligned = yield from self._calibrate_with_boundary_retreat(
                target_heading, first_waypoint, "初始航向对准")
        except Exception as e:
            self.get_logger().error(f"初始航向对准失败: {e}")
            return False
        if heading_aligned:
            self.get_logger().info("初始航向对准完成, 开始实时纠偏直线行驶")
            last_heading = (target_heading + 180) % 360 - 180
            self.get_logger().info(f"记录初始旋转完成航向角：{last_heading:.2f}°")
        else:
            self.get_logger().warn("初始航向对准未完成，继续进入实时纠偏直线行驶")
        
        # ========== 步骤3：Stanley控制器直线行驶 ==========
        last_distance = 0.0
        consecutive_threshold = 5
        consecutive_count = 0

        last_left_speed = None
        last_right_speed = None

        path_start = (init_lon, init_lat)
        path_end = (first_lon, first_lat)
        fixed_path_dir = self.calculate_path_bearing(init_lon, init_lat, first_lon, first_lat)
        self.get_logger().info(f"[Stanley] 初始路径：({init_lon:.6f}, {init_lat:.6f}) → ({first_lon:.6f}, {first_lat:.6f}), 方向={fixed_path_dir:.1f}°")

        while rclpy.ok():
            current_lon, current_lat = self.current_gps
            current_pos = (current_lon, current_lat)
            target_lon, target_lat, target_heading = first_waypoint

            distance = self.calc_distance_to_waypoint(first_waypoint)

            if abs(last_distance - distance) > 0.1:
                self.get_logger().info(f"到第一个航点距离：{distance:.2f} m")
                last_distance = distance

            if distance < INITIAL_MOVE_TOLERANCE:
                consecutive_count += 1
                self.get_logger().info(f"距离达标, 连续计数：{consecutive_count}/{consecutive_threshold}")
                if consecutive_count >= consecutive_threshold:
                    self.get_logger().info(f"已到达第一个航点距离阈值：{distance:.2f} m")
                    target_waypoint_heading = self.get_path_heading(first_waypoint)
                    self.get_logger().info(f"开始最终航向校准：目标{target_waypoint_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
                    self.nav_context["nav_state"] = NavState.WAYPOINT_CALIB
                    self.nav_context["calib_generator"] = None
                    yield from self._calibrate_with_boundary_retreat(
                        target_waypoint_heading, first_waypoint, "第一个航点最终校准")
                    self.nav_context["calib_generator"] = None
                    self.nav_context["target_waypoint"] = None
                    return True
            else:
                consecutive_count = 0

            if distance < LOW_DISTANCE:
                speed_scale = max(0.3, distance / LOW_DISTANCE * 0.7)
                current_base_speed = LINEAR_SPEED_BASE * speed_scale
            else:
                current_base_speed = LINEAR_SPEED_BASE

            t = self._get_projection_ratio(current_pos, path_start, path_end)
            if self.nav_context.get("force_bearing_mode"):
                current_bearing = self.calculate_bearing(current_lat, current_lon, first_lat, first_lon)
                self.nav_context["force_bearing_target"] = current_bearing
                bearing_err = abs(self.normalize_angle(current_bearing - self.imu_yaw))
                if bearing_err > 15.0:
                    self.nav_context["force_bearing_recalib_count"] += 1
                    if self.nav_context["force_bearing_recalib_count"] > FORCE_BEARING_MAX_RECALIB:
                        self.get_logger().error(
                            f"[Stanley] force_bearing 原地对准反复触发{self.nav_context['force_bearing_recalib_count']}次"
                            f"（疑似极限环），暂停导航等待人工介入"
                        )
                        self.set_rtk_error_bits(ERROR_CALIB_TIMEOUT)
                        self.nav_context["calib_generator"] = None
                        self.nav_context["nav_state"] = NavState.PAUSE
                        self.nav_context["pre_pause_state"] = NavState.INITIAL_MOVE
                        self.nav_context["pause_reason"] = "force_bearing_limit_cycle"
                        self.nav_context["manual_intervention_seen"] = False
                        self.nav_context["brush_active"] = self.brush_active
                        self.nav_running = False
                        self.publish_nav_state(NavState.PAUSE)
                        self.publish_stop_speed()
                        yield (0.0, 0.0)
                        return False
                    self.get_logger().warn(
                        f"[Stanley] force_bearing 航向偏差{bearing_err:.1f}°>15°，原地旋转对准{current_bearing:.1f}°")
                    yield from self._calibrate_with_boundary_retreat(
                        current_bearing, first_waypoint, "初始force_bearing原地对准")
                    continue
                if self._force_bearing_diverging(distance):
                    self.get_logger().error(
                        f"[Stanley] force_bearing 持续背离目标（距最近点+{FORCE_BEARING_DIVERGE_DIST}m超"
                        f"{FORCE_BEARING_DIVERGE_COUNT}帧，当前{distance:.2f}m），暂停导航等待人工介入"
                    )
                    self.set_rtk_error_bits(ERROR_CALIB_TIMEOUT)
                    self.nav_context["calib_generator"] = None
                    self.nav_context["nav_state"] = NavState.PAUSE
                    self.nav_context["pre_pause_state"] = NavState.INITIAL_MOVE
                    self.nav_context["pause_reason"] = "force_bearing_diverge"
                    self.nav_context["manual_intervention_seen"] = False
                    self.nav_context["brush_active"] = self.brush_active
                    self.nav_running = False
                    self.publish_nav_state(NavState.PAUSE)
                    self.publish_stop_speed()
                    yield (0.0, 0.0)
                    return False
                path_direction = current_bearing
            elif t > 1.0:
                self.nav_context["force_bearing_mode"] = True
                self.nav_context["force_bearing_target"] = self.calculate_bearing(
                    current_lat, current_lon, first_lat, first_lon
                )
                continue
            elif self.nav_context.get("bearing_mode_locked"):
                path_direction = self.calculate_bearing(current_lat, current_lon, first_lat, first_lon)
            else:
                path_direction = fixed_path_dir

            left_speed, right_speed = self.stanley_steering_control(
                current_pos=current_pos,
                current_heading=self.imu_yaw,
                path_start=path_start,
                path_end=path_end,
                path_direction=path_direction,
                velocity=current_base_speed,
                distance_to_target=distance,
                bearing_only=self.nav_context.get("force_bearing_mode", False)
            )

            heading_err = self.normalize_angle(path_direction - self.imu_yaw)
            # force_bearing_mode 时航向误差来自侧向接近目标，非 IMU 异常，跳过重校准
            in_bearing_mode = self.nav_context.get("force_bearing_mode", False)
            if abs(heading_err) > HEADING_ABNORMAL_THRESHOLD:
                if not in_bearing_mode:
                    self.nav_context["angle_abnormal_count"] += 1
                    if self.heading_abnormal_start_time is None:
                        self.heading_abnormal_start_time = time.monotonic()
                else:
                    self.nav_context["angle_abnormal_count"] = 0
                    self.heading_abnormal_start_time = None
                    self.heading_timed_out = False
            else:
                self.nav_context["angle_abnormal_count"] = 0
                self.heading_abnormal_start_time = None
                self.heading_timed_out = False

            # 连续航向异常触发即时重校准；同航点≥2次则切换到方位角直行模式
            if self.nav_context["angle_abnormal_count"] >= ANGLE_ABNORMAL_COUNT:
                self.nav_context["waypoint_recalib_count"] += 1
                self.get_logger().warn(
                    f"[Stanley] 航向异常持续{self.nav_context['angle_abnormal_count']}帧："
                    f"hdg_err={heading_err:.1f}°，同航点第{self.nav_context['waypoint_recalib_count']}次校准"
                )
                self.nav_context["angle_abnormal_count"] = 0
                if self.nav_context["waypoint_recalib_count"] >= 2:
                    self.nav_context["force_bearing_mode"] = True
                    self.heading_abnormal_start_time = None
                    self.heading_timed_out = False
                    fixed_bearing = self.calculate_bearing(current_lat, current_lon, first_lat, first_lon)
                    self.nav_context["force_bearing_target"] = fixed_bearing
                    self.nav_context["calib_target_heading"] = fixed_bearing
                    self.get_logger().warn(
                        f"[Stanley] 同航点≥2次航向异常，先原地旋转校准到目标方位{fixed_bearing:.1f}°，再直行"
                    )
                    yield from self._calibrate_with_boundary_retreat(
                        fixed_bearing, first_waypoint, "初始航向异常方位校准")
                    continue
                yield from self._calibrate_with_boundary_retreat(
                    path_direction, first_waypoint, "初始航向异常路径校准")
                continue

            if left_speed != last_left_speed or right_speed != last_right_speed:
                lateral_err = self.calculate_lateral_error(current_pos, path_start, path_end)
                self.get_logger().info(
                    f"[Stanley] 初始移动：left={left_speed:.2f}, right={right_speed:.2f}, "
                    f"lat_err={lateral_err:.3f}m, hdg_err={heading_err:.1f}°, path_dir={path_direction:.1f}°, t={t:.3f}"
                )
            last_left_speed = left_speed
            last_right_speed = right_speed

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
            "pause_reason": None,
            "manual_intervention_seen": False,
            "angle_abnormal_count": 0,
            "is_angle_recalib": False,
            "waypoint_recalib_count": 0,
            "force_bearing_mode": False,
            "force_bearing_target": None,
            "force_bearing_recalib_count": 0,
            "force_bearing_min_distance": float('inf'),
            "distance_increase_count": 0,
            "bearing_mode_locked": False,
            "brush_active": False,  # 滚刷是否激活
            "tilt_confirm_count": 0,
            "tilt_normal_count": 0,
            "tilt_fault": False,
            "calib_retry_count": 0,
            "calib_target_heading": None,
        }
        self.clear_rtk_error_bits(ERROR_TILT_FAULT)  # 同步清除，防止tilt_fault与rtk_error_code脱钩
        self.heading_abnormal_start_time = None  # 重置航向异常计时
        self.heading_timed_out = False
        self.return_to_loading_added = False  # 重置出仓点追加标志
        self.brush_active = False  # 重置滚刷状态
        self.is_boundary_triggered = False
        self.boundary_correct_state = BoundaryCorrectState.IDLE
        self.boundary_correct_start_time = None
        self.boundary_correct_direction = None
        self.boundary_correct_locked = False
        self.boundary_active_count = 0
        self.boundary_clear_count = 0
        self.boundary_stop_published = False
        self.boundary_trigger_yaw = 0.0
        self.boundary_target_yaw = 0.0
        self.boundary_slow_count = 0
        self.boundary_correction_start_time = None
        self.confirmed_sensors.clear()
        self.blocked_directions.clear()
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

    def load_pending_path_after_task(self):
        """任务COMPLETED并重置为IDLE后，加载下一条路径，等待下一次UNLOADING启动。"""
        if not self.pending_next_path_file:
            return

        next_file = self.pending_next_path_file
        self.pending_next_path_file = None
        self.get_logger().info(f"[RTKNav] 任务收尾完成，切换到下一次清扫路径: {next_file}")

        self.cross_file_last_waypoint = None
        self.waypoints = []
        self.waypoint_areas = []
        self.current_cleaning_area = ""
        self.update_cleaning_area(force=True)
        self.current_waypoint_idx = 0
        self.return_to_loading_added = False
        self.brush_start_indices.clear()
        self.brush_stop_indices.clear()
        self.brush_active = False
        self.rtk_path_file = next_file

        if self.load_waypoints_from_file(self.rtk_path_file):
            self.nav_context["nav_state"] = NavState.IDLE
            self.nav_context["pre_pause_state"] = None
            self.publish_nav_state(NavState.IDLE)
            self.publish_current_route_id()
            self.waiting_for_next_unloading = True
            self.get_logger().info(f"[RTKNav] 下一次清扫路径已加载，共 {len(self.waypoints)} 个航点，等待下一次UNLOADING后启动")
        else:
            self.get_logger().error(f"[RTKNav] 下一次清扫路径加载失败: {next_file}")

    def finish_navigation_task(self):
        """统一处理任务完成后的停车、状态重置和下一路径预加载。"""
        has_pending_path = self.pending_next_path_file is not None
        self.publish_nav_state(NavState.COMPLETED)
        self.multi_waypoint_generator = None
        self.nav_running = False
        self.brush_active = False
        self.publish_stop_speed()
        self.reset_nav_context()
        self.publish_nav_state(NavState.IDLE)
        self.get_logger().info("[ROSNode] 导航COMPLETED收尾完成，已清理上下文并发布IDLE")
        if has_pending_path:
            self.load_pending_path_after_task()
        else:
            self.waypoints = []
            self.waypoint_areas = []
            self.current_cleaning_area = ""
            self.update_cleaning_area(force=True)
            self.waiting_for_next_unloading = True
            self.get_logger().info("[RTKNav] 没有下一条清扫路径，已清空当前航点并等待退出AUTO_CLEANING")

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

    def _is_manual_intervention_pause(self) -> bool:
        return (
            self.nav_context.get("nav_state") == NavState.PAUSE
            and self.nav_context.get("pause_reason") in MANUAL_INTERVENTION_PAUSE_REASONS
        )

    def _resume_manual_intervention_pause(self) -> bool:
        """人工处理完成后，从当前航点重新进入WAYPOINT_MOVE。"""
        if not self._is_manual_intervention_pause():
            return False
        if not self.nav_context.get("manual_intervention_seen", False):
            return False
        if not (0 <= self.current_waypoint_idx < len(self.waypoints)):
            self.get_logger().error(
                f"[RTKNav] 人工暂停无法恢复：航点索引{self.current_waypoint_idx}无效，"
                f"航点总数={len(self.waypoints)}"
            )
            return False

        pause_reason = self.nav_context.get("pause_reason")
        target_waypoint = self.waypoints[self.current_waypoint_idx]

        self.nav_context["nav_state"] = NavState.WAYPOINT_MOVE
        self.nav_context["target_waypoint"] = target_waypoint
        self.nav_context["calib_generator"] = None
        self.nav_context["calib_target_heading"] = None
        self.nav_context["calib_retry_count"] = 0
        self.nav_context["angle_abnormal_count"] = 0
        self.nav_context["is_angle_recalib"] = False
        self.nav_context["waypoint_recalib_count"] = 0
        self.nav_context["force_bearing_mode"] = False
        self.nav_context["force_bearing_target"] = None
        self.nav_context["force_bearing_recalib_count"] = 0
        self.nav_context["force_bearing_min_distance"] = float('inf')
        self.nav_context["distance_increase_count"] = 0
        self.nav_context["bearing_mode_locked"] = False
        self.nav_context["pre_pause_state"] = None
        self.nav_context["pause_reason"] = None
        self.nav_context["manual_intervention_seen"] = False

        self.stanley_path_start = None
        if self.current_waypoint_idx > 0:
            self.last_waypoint_cache = self.waypoints[self.current_waypoint_idx - 1]
        else:
            # 航点0没有前置航点，下一轮以当前GPS作为Stanley路径起点。
            self.last_waypoint_cache = None

        self.heading_abnormal_start_time = None
        self.heading_timed_out = False
        self.clear_rtk_error_bits(ERROR_CALIB_TIMEOUT)
        self.multi_waypoint_generator = None
        self.nav_running = False
        self.publish_nav_state(NavState.WAYPOINT_MOVE)
        self.get_logger().info(
            f"[RTKNav] 人工处理完成，解除{pause_reason}锁定，"
            f"从航点{self.current_waypoint_idx}重新进入WAYPOINT_MOVE"
        )
        return True

    def multi_waypoint_nav_generator(self, resume: bool = False):
        """
        多点RTK导航生成器（适配ROS2定时器回调）
        每次yield返回左右轮速度, 支持中断恢复
        """
        # 1. 初始化/恢复导航状态
        if resume:
            current_nav_state = self.nav_context["nav_state"]
            pause_reason = self.nav_context.get("pause_reason", "")
            
            # 处理暂停状态恢复：检查故障条件是否已清除
            if current_nav_state == NavState.PAUSE:
                if pause_reason in MANUAL_INTERVENTION_PAUSE_REASONS:
                    self.get_logger().error(
                        f"[ROSNode] {pause_reason}为人工介入锁定暂停，"
                        "拒绝自动恢复；请先切出AUTO_CLEANING处理，再重新进入AUTO_CLEANING"
                    )
                    self.publish_stop_speed()
                    yield (0.0, 0.0)
                    return
                fault_active = False
                if pause_reason == "tilt_fault" and self.nav_context.get("tilt_fault", False):
                    self.get_logger().warn("[ROSNode] 跌落故障仍活跃，保持PAUSE等待倾斜恢复")
                    fault_active = True
                elif pause_reason == "heading_timeout" and self.heading_timed_out:
                    self.get_logger().warn("[ROSNode] 航向超时仍活跃，保持PAUSE等待航向恢复")
                    fault_active = True
                elif pause_reason == "rtk_not_fixed" and not self.rtk_solution_ready:
                    self.get_logger().warn(
                        f"[ROSNode] RTK未就绪(position={self.last_gps_status}, "
                        f"orientation={self.last_orientation_status}, "
                        f"gga_valid={self.position_data_valid})，保持PAUSE"
                    )
                    fault_active = True
                elif pause_reason == "rtk_timeout" and self.rtk_data_timed_out:
                    self.get_logger().warn("[ROSNode] RTK数据超时仍活跃，保持PAUSE等待数据恢复")
                    fault_active = True

                if not fault_active:
                    pre_pause_state = self.nav_context.get("pre_pause_state", NavState.WAYPOINT_MOVE)
                    self.get_logger().info(f"[ROSNode] 从RTK暂停状态恢复，恢复到：{pre_pause_state}")
                    current_nav_state = pre_pause_state
                    self.nav_context["nav_state"] = current_nav_state

                    # 跌落后稳定等待：倾斜故障导致的暂停恢复后，需等待INS数据稳定
                    if pause_reason == "tilt_fault" and self.last_tilt_time > 0:
                        if self.last_tilt_duration > 0 and self.last_tilt_duration < TILT_SHORT_DURATION:
                            self.get_logger().info(
                                f"[跌落稳定] 倾斜仅持续{self.last_tilt_duration:.1f}s"
                                f"(<{TILT_SHORT_DURATION}s)，短促颠簸，跳过稳定等待"
                            )
                        else:
                            elapsed_since_tilt = time.monotonic() - self.last_tilt_time
                            if elapsed_since_tilt < TILT_STABILIZE_TIMEOUT:
                                remaining = TILT_STABILIZE_TIMEOUT - elapsed_since_tilt
                                self.get_logger().warn(
                                    f"[跌落稳定] 距上次倾斜故障仅{elapsed_since_tilt:.0f}s，"
                                    f"需等待{remaining:.0f}s让INS数据稳定后自动恢复清扫"
                                )
                                last_stabilize_log = 0.0
                                while True:
                                    if not self.check_control_mode():
                                        yield (0.0, 0.0)
                                        return
                                    now = time.monotonic()
                                    elapsed_since_tilt = now - self.last_tilt_time
                                    if elapsed_since_tilt >= TILT_STABILIZE_TIMEOUT:
                                        self.get_logger().info(
                                            f"[跌落稳定] 等待完成，已过{elapsed_since_tilt:.0f}s，恢复清扫"
                                        )
                                        break
                                    # 每30s输出一次等待进度
                                    if now - last_stabilize_log >= 30.0:
                                        last_stabilize_log = now
                                        remaining = TILT_STABILIZE_TIMEOUT - elapsed_since_tilt
                                        self.get_logger().info(
                                            f"[跌落稳定] 等待INS稳定中... 剩余{remaining:.0f}s"
                                        )
                                    self.publish_stop_speed()
                                    yield (0.0, 0.0)

                    # 如果恢复的是校准状态，需要重新初始化校准生成器
                    if current_nav_state == NavState.WAYPOINT_CALIB:
                        target_waypoint = self.nav_context.get("target_waypoint")
                        if target_waypoint:
                            saved = self.nav_context.get("calib_target_heading")
                            target_heading = saved if saved is not None else self.get_path_heading(target_waypoint)
                            self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_heading)
                            self.get_logger().info(f"重新初始化航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
                        # 清零航向异常状态，避免恢复后立即触发heading_timeout杀死校准
                        self.heading_abnormal_start_time = None
                        self.heading_timed_out = False
                        self.nav_context["angle_abnormal_count"] = 0

            if current_nav_state != NavState.PAUSE:
                self.get_logger().info(f"从状态{current_nav_state}恢复导航")
            # 新增：恢复校准状态时，重新初始化校准生成器
            if current_nav_state == NavState.WAYPOINT_CALIB:
                target_waypoint = self.nav_context["target_waypoint"]
                if target_waypoint:
                    saved = self.nav_context.get("calib_target_heading")
                    target_heading = saved if saved is not None else self.get_path_heading(target_waypoint)
                    # 重新创建校准生成器（重置超时计时和误差计算）
                    self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_heading)
                    self.get_logger().info(f"重新初始化航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
                # 清零航向异常状态，避免恢复后立即触发heading_timeout杀死校准
                self.heading_abnormal_start_time = None
                self.heading_timed_out = False
                self.nav_context["angle_abnormal_count"] = 0
            
            # 恢复可自动恢复暂停前保存的滚刷状态
            # brush_start_indices已被消费(pop)，无法通过check_and_control_brush自动重启
            saved_brush = self.nav_context.get("brush_active", False)
            if saved_brush and not self.brush_active:
                self.brush_active = True
                self.get_logger().info(f"[{pause_reason}恢复] 恢复暂停前滚刷开启状态")

            # 恢复导航时检查滚刷控制（处理暂停恢复场景）
            self.check_and_control_brush()

            # 新增：强制更新一次到目标航点的距离
            if self.nav_context["target_waypoint"]:
                distance = self.calc_distance_to_waypoint(self.nav_context["target_waypoint"])
                self.nav_context["last_distance"] = distance

                if current_nav_state == NavState.WAYPOINT_MOVE and self.current_gps:
                    target_lon, target_lat, _ = self.nav_context["target_waypoint"]
                    current_lon, current_lat = self.current_gps
                    if hasattr(self, 'stanley_path_direction') and self.stanley_path_direction is not None:
                        path_direction = self.stanley_path_direction
                    else:
                        path_direction = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)

                    heading_err = self.normalize_angle(path_direction - self.imu_yaw)
                    if abs(heading_err) > HEADING_ABNORMAL_THRESHOLD:
                        self.start_heading_recalibration(path_direction, heading_err, "恢复导航时航向异常")
                        current_nav_state = NavState.WAYPOINT_CALIB
                        if self.heading_abnormal_start_time is None:
                            self.heading_abnormal_start_time = time.monotonic()
                    else:
                        self.nav_context["angle_abnormal_count"] = 0
                        self.heading_abnormal_start_time = None
                        self.heading_timed_out = False
        else:
            # 只有导航状态为IDLE时, 才重新初始化初始移动（解决重复进入第一个航点）
            if self.nav_context["nav_state"] == NavState.IDLE:
                # 跌落后稳定等待：倾斜故障后首次AUTO_CLEANING需等待INS数据稳定
                if self.last_tilt_time > 0:
                    if self.last_tilt_duration > 0 and self.last_tilt_duration < TILT_SHORT_DURATION:
                        self.get_logger().info(
                            f"[跌落稳定] 倾斜仅持续{self.last_tilt_duration:.1f}s"
                            f"(<{TILT_SHORT_DURATION}s)，短促颠簸，跳过稳定等待"
                        )
                    else:
                        elapsed_since_tilt = time.monotonic() - self.last_tilt_time
                        if elapsed_since_tilt < TILT_STABILIZE_TIMEOUT:
                            remaining = TILT_STABILIZE_TIMEOUT - elapsed_since_tilt
                            self.get_logger().warn(
                                f"[跌落稳定] 距上次倾斜故障仅{elapsed_since_tilt:.0f}s，"
                                f"需等待{remaining:.0f}s让INS数据稳定后自动开始清扫"
                            )
                            while True:
                                if not self.check_control_mode():
                                    yield (0.0, 0.0)
                                    return
                                elapsed_since_tilt = time.monotonic() - self.last_tilt_time
                                if elapsed_since_tilt >= TILT_STABILIZE_TIMEOUT:
                                    self.get_logger().info(
                                        f"[跌落稳定] 等待完成，已过{elapsed_since_tilt:.0f}s，开始清扫"
                                    )
                                    break
                                self.publish_stop_speed()
                                yield (0.0, 0.0)
                # 初始航向校验：IMU在90°±15°范围内 + 5s无漂移
                while True:
                    if not self.check_control_mode():
                        yield (0.0, 0.0)
                        return
                    stable_ok = self._last_heading_stable
                    if stable_ok:
                        self.get_logger().info(
                            f"[初始航向校验] 通过：IMU航向={self.imu_yaw:.1f}°，"
                            f"航向稳定（5s内波动≤{HEADING_STABILITY_RANGE}°），"
                            f"不限制固定角度，出仓后无漂移即放行"
                        )
                        break
                    reasons = []
                    if not stable_ok:
                        reasons.append(f"5s内航向不稳定（波动>{HEADING_STABILITY_RANGE}°），"
                                       f"出仓后需IMU无漂移才可开始导航")
                    now = time.monotonic()
                    if now - self.last_heading_check_log_time >= 10.0:
                        self.get_logger().warn(
                            f"[初始航向校验] 等待航向就绪：IMU航向={self.imu_yaw:.1f}°，"
                            + "；".join(reasons) + "，保持停车等待..."
                        )
                        self.last_heading_check_log_time = now
                    self.publish_stop_speed()
                    yield (0.0, 0.0)
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
        if self.blocked_directions:
            self.get_logger().warn(f"boundary blocked: {sorted(self.blocked_directions)}")

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
                    if (self.nav_context.get("nav_state") != NavState.WAYPOINT_CALIB
                            and (self._is_motion_blocked() or self.boundary_correct_locked)):
                        # 行进方向被禁止时，首次调用会启动边界矫正状态机
                        left_speed, right_speed = self.get_boundary_correct_speed()
                    yield (left_speed, right_speed)
                except StopIteration:
                    # force_bearing 极限环兜底：子生成器已置 PAUSE 并停车，
                    # 不能落入下方"到达第一航点"逻辑误切 WAYPOINT_MOVE
                    if self.nav_context.get("nav_state") == NavState.PAUSE:
                        yield (0.0, 0.0)
                        return
                    if len(self.waypoints) == 0:
                        self.get_logger().error("[ROSNode] 初始移动StopIteration：无航点数据，终止导航")
                        self.nav_running = False
                        self.publish_stop_speed()
                        self.reset_nav_context()
                        yield (0.0, 0.0)
                        return
                    # 初始移动已经完成航点0的距离到达和最终航向校准，后续必须进入航点1。
                    # 避免历史 is_angle_recalib 标志残留导致继续导航到航点0，生成零长度Stanley路径。
                    if self.nav_context.get("is_angle_recalib", False):
                        self.get_logger().warn("[ROSNode] 初始移动完成时发现残留角度异常标志，已清除并切换到航点1")
                        self.nav_context["is_angle_recalib"] = False
                    if len(self.waypoints) <= 1:
                        self.get_logger().info("[ROSNode] 当前路径只有1个航点，初始航点完成后按原逻辑追加进仓点")
                    if self.waypoints:
                        self.last_waypoint_cache = self.waypoints[0]
                    self.current_waypoint_idx = 1
                    current_nav_state = NavState.WAYPOINT_MOVE
                    self.nav_context["nav_state"] = current_nav_state
                    self.stanley_path_start = None
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
                self.stanley_path_start = None
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

            # PAUSE状态处理：停止电机并等待故障恢复
            if current_nav_state == NavState.PAUSE:
                yield (0.0, 0.0)
                current_nav_state = self.nav_context["nav_state"]
                continue

            # 子阶段A：航向校准（到达航点后执行）
            if current_nav_state == NavState.WAYPOINT_CALIB:
                calib_generator = self.nav_context["calib_generator"]
                if not calib_generator:
                    self.get_logger().warn("[ROSNode] 校准生成器不存在, 重新初始化")
                    saved = self.nav_context.get("calib_target_heading")
                    target_heading = saved if saved is not None else self.get_path_heading(target_waypoint)
                    self.nav_context["calib_target_heading"] = target_heading
                    calib_generator = self.calibrate_heading_at_waypoint(target_heading)
                    self.nav_context["calib_generator"] = calib_generator
                try:
                    # self.get_logger().info(f"{self.current_waypoint_idx}:航向校准：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
                    left_speed, right_speed = next(calib_generator)
                    if self._is_motion_blocked() or self.boundary_correct_locked:
                        if self.boundary_correct_locked:
                            left_speed, right_speed = self.get_boundary_correct_speed()
                        else:
                            # 旋转打滑偏移 → GPS撤退回航点，重新定位后重试
                            self.get_logger().warn(
                                f"[RTKNav] 校准打滑触发传感器({sorted(self.blocked_directions)})，"
                                f"执行GPS撤退回航点{self.current_waypoint_idx}")
                            retreat_gen = self._retreat_to_waypoint(target_waypoint)
                            try:
                                while True:
                                    left_speed, right_speed = next(retreat_gen)
                                    if self.boundary_correct_locked:
                                        left_speed, right_speed = self.get_boundary_correct_speed()
                                    elif self._is_speed_blocked(left_speed, right_speed):
                                        # 撤退生成器已给出安全远离动作时直接放行；
                                        # 只有即将发布的动作仍被禁止，才交给边界矫正接管。
                                        self.get_logger().warn(
                                            f"[RTKNav] 撤退动作被边界禁止({sorted(self.blocked_directions)})，"
                                            "启动边界矫正")
                                        left_speed, right_speed = self.get_boundary_correct_speed()
                                    yield (left_speed, right_speed)
                            except StopIteration:
                                pass
                            # 撤退完成，重新初始化校准
                            target_heading = self.get_path_heading(target_waypoint)
                            self.nav_context["calib_target_heading"] = target_heading
                            calib_generator = self.calibrate_heading_at_waypoint(target_heading)
                            self.nav_context["calib_generator"] = calib_generator
                            self.get_logger().info("[RTKNav] 撤退完成，重新执行航向校准")
                            continue
                    yield (left_speed, right_speed)
                except StopIteration as e:
                    calib_result = e.value if hasattr(e, 'value') else False

                    # 校准失败（卡滞无法克服）
                    if not calib_result:
                        self.nav_context["calib_retry_count"] += 1
                        retry_count = self.nav_context["calib_retry_count"]
                        if retry_count <= CALIB_STUCK_MAX_RETRIES:
                            self.get_logger().warn(
                                f"[ROSNode] 航向校准失败（卡滞），第{retry_count}/{CALIB_STUCK_MAX_RETRIES}次自动重试"
                            )
                            # 第2次及以上重试：先短暂后退脱困，再重新校准
                            if retry_count >= 2:
                                self.get_logger().info("[航向校准] 后退1.5s脱困...")
                                backup_start = self.get_clock().now()
                                while (self.get_clock().now() - backup_start).nanoseconds / 1e9 < 1.5:
                                    if self._is_motion_blocked() or self.boundary_correct_locked:
                                        if self.boundary_correct_locked:
                                            yield self.get_boundary_correct_speed()
                                        else:
                                            yield (0.0, 0.0)
                                    else:
                                        yield (1.0, -1.0) # 后退速度
                                yield (0.0, 0.0)
                            self.heading_abnormal_start_time = None
                            self.heading_timed_out = False
                            self.nav_context["angle_abnormal_count"] = 0
                            saved = self.nav_context.get("calib_target_heading")
                            target_heading = saved if saved is not None else self.get_path_heading(target_waypoint)
                            self.nav_context["calib_generator"] = self.calibrate_heading_at_waypoint(target_heading)
                            self.get_logger().info(f"重新初始化航向校准（重试{retry_count}）：目标{target_heading:.2f}°, 当前{self.imu_yaw:.2f}°")
                            yield (0.0, 0.0)
                            continue
                        self.get_logger().error(
                            f"[ROSNode] 航向校准失败（卡滞/阻力过大），已重试{CALIB_STUCK_MAX_RETRIES}次，暂停导航等待人工介入"
                        )
                        self.set_rtk_error_bits(ERROR_CALIB_TIMEOUT)
                        self.nav_context["calib_generator"] = None
                        self.nav_context["nav_state"] = NavState.PAUSE
                        self.nav_context["pre_pause_state"] = NavState.WAYPOINT_CALIB
                        self.nav_context["pause_reason"] = "calib_stuck"
                        self.nav_context["manual_intervention_seen"] = False
                        self.nav_context["brush_active"] = self.brush_active
                        self.nav_running = False
                        self.publish_nav_state(NavState.PAUSE)
                        self.publish_stop_speed()
                        yield (0.0, 0.0)
                        return

                    is_recalib = self.nav_context.get("is_angle_recalib", False)

                    # 校准成功，清零卡滞重试计数和暂停原因（防止残留导致StopIteration误判）
                    if self.nav_context.get("calib_retry_count", 0) > 0:
                        self.get_logger().info(f"[航向校准] 校准成功，清零卡滞重试计数（之前重试{self.nav_context['calib_retry_count']}次）")
                        self.nav_context["calib_retry_count"] = 0
                    self.nav_context["pause_reason"] = None

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
                        self.stanley_path_start = None
                        if self.current_waypoint_idx - 1 >= 0 and len(self.waypoints) > self.current_waypoint_idx - 1:
                            self.last_waypoint_cache = self.waypoints[self.current_waypoint_idx - 1]
                        self.last_valid_heading = None  # 清空航向缓存，切换航点后使用实时航向
                        self.heading_history = []  # 清空航向历史
                    
                    # 检查滚刷控制
                    self.check_and_control_brush()
                    self.nav_context["angle_abnormal_count"] = 0
                    self.nav_context["calib_generator"] = None
                    self.nav_context["target_waypoint"] = None
                    self.nav_context["last_distance"] = 0.0  # 重置距离缓存
                    self.nav_context["last_target_heading"] = 0.0  # 重置航向缓存

                    if not is_recalib:
                        # 正常航点切换时才清零打滑计数器和方位角直行模式
                        self.nav_context["waypoint_recalib_count"] = 0
                        self.nav_context["force_bearing_mode"] = False
                        self.nav_context["force_bearing_target"] = None
                        self.nav_context["force_bearing_recalib_count"] = 0
                        self.nav_context["force_bearing_min_distance"] = float('inf')
                        self.nav_context["distance_increase_count"] = 0
                        self.nav_context["bearing_mode_locked"] = False
                        # 仅在正常航点切换时获取新航点
                        target_waypoint = self.get_target_waypoint(self.current_waypoint_idx)
                        self.nav_context["target_waypoint"] = target_waypoint
                        if target_waypoint:
                            self.get_logger().info(f"[ROSNode] 切换到新航点{self.current_waypoint_idx}, 准备进入移动阶段")
                        else:
                            # 返仓点校准完成（return_to_loading_added=True且无更多航点）
                            # 直接走完成流程，不通过StopIteration（避免pause_reason残留导致误判）
                            self.get_logger().info("[ROSNode] 返回进仓点校准完成，导航任务全部完成")
                            self.nav_context["pause_reason"] = None  # 清除可能的残留状态
                            self.nav_context["nav_state"] = NavState.COMPLETED
                            self.publish_nav_state(NavState.COMPLETED)
                            self.finish_navigation_task()
                            yield (0.0, 0.0)
                            return
                    else:
                        # 航向异常重校准后清除 bearing_mode_locked 和路径缓存
                        # 让下一帧用当前位姿重新初始化 Stanley 路径段，避免
                        # t>1.0 立即触发 bearing mode 压制后续航向异常检测
                        self.nav_context["bearing_mode_locked"] = False
                        self.stanley_path_start = None
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
                current_lon, current_lat = self.current_gps
                current_pos = (current_lon, current_lat)
                target_lon, target_lat, target_heading = target_waypoint

                distance = self.calc_distance_to_waypoint(target_waypoint)

                if distance < RTK_WAYPOINT_TOLERANCE:
                    self.get_logger().info(
                        f"[ROSNode] 已到达航点{self.current_waypoint_idx}距离阈值（{distance:.2f}m ≤ {RTK_WAYPOINT_TOLERANCE}m）"
                    )
                    self.nav_context["angle_abnormal_count"] = 0
                    calib_target_heading = self.get_path_heading(target_waypoint)
                    self.nav_context["calib_target_heading"] = calib_target_heading
                    calib_generator = self.calibrate_heading_at_waypoint(calib_target_heading)
                    self.nav_context["calib_generator"] = calib_generator
                    current_nav_state = NavState.WAYPOINT_CALIB
                    self.nav_context["nav_state"] = current_nav_state
                    yield (0.0, 0.0)
                    continue

                path_initialized_now = False
                if not hasattr(self, 'stanley_path_start') or self.stanley_path_start is None:
                    if self.last_waypoint_cache:
                        path_start_lon, path_start_lat = self.last_waypoint_cache[0], self.last_waypoint_cache[1]
                    elif self.cross_file_last_waypoint:
                        path_start_lon, path_start_lat = self.cross_file_last_waypoint[0], self.cross_file_last_waypoint[1]
                    else:
                        path_start_lon, path_start_lat = current_lon, current_lat

                    self.stanley_path_start = (path_start_lon, path_start_lat)
                    self.stanley_path_direction = self.calculate_path_bearing(
                        path_start_lon, path_start_lat, target_lon, target_lat
                    )
                    self.get_logger().info(
                        f"[Stanley] 航点{self.current_waypoint_idx}路径："
                        f"({path_start_lon:.6f}, {path_start_lat:.6f}) → ({target_lon:.6f}, {target_lat:.6f}), "
                        f"方向={self.stanley_path_direction:.1f}°"
                    )
                    path_initialized_now = True

                if distance < LOW_DISTANCE:
                    speed_scale = max(0.3, distance / LOW_DISTANCE * 0.7)
                    current_base_speed = LINEAR_SPEED_BASE * speed_scale
                else:
                    current_base_speed = LINEAR_SPEED_BASE

                path_end = (target_lon, target_lat)
                t = self._get_projection_ratio(current_pos, self.stanley_path_start, path_end)
                if self.nav_context.get("force_bearing_mode"):
                    # 实时算目标方位，偏差大则先原地旋转对准再走，偏差小直接跟
                    current_bearing = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
                    self.nav_context["force_bearing_target"] = current_bearing
                    bearing_err = abs(self.normalize_angle(current_bearing - self.imu_yaw))
                    if bearing_err > 15.0:
                        self.nav_context["force_bearing_recalib_count"] += 1
                        if self.nav_context["force_bearing_recalib_count"] > FORCE_BEARING_MAX_RECALIB:
                            self.get_logger().error(
                                f"[Stanley] force_bearing 原地对准反复触发{self.nav_context['force_bearing_recalib_count']}次"
                                f"（疑似极限环），暂停导航等待人工介入"
                            )
                            self.set_rtk_error_bits(ERROR_CALIB_TIMEOUT)
                            self.nav_context["calib_generator"] = None
                            self.nav_context["nav_state"] = NavState.PAUSE
                            self.nav_context["pre_pause_state"] = NavState.WAYPOINT_MOVE
                            self.nav_context["pause_reason"] = "force_bearing_limit_cycle"
                            self.nav_context["manual_intervention_seen"] = False
                            self.nav_context["brush_active"] = self.brush_active
                            self.nav_running = False
                            self.publish_nav_state(NavState.PAUSE)
                            self.publish_stop_speed()
                            yield (0.0, 0.0)
                            return
                        # 航向偏差大→先原地旋转对准目标，防止追尾螺旋
                        self.get_logger().warn(
                            f"[Stanley] force_bearing 航向偏差{bearing_err:.1f}°>15°，原地旋转对准{current_bearing:.1f}°")
                        yield from self._calibrate_with_boundary_retreat(
                            current_bearing, target_waypoint, "force_bearing原地对准")
                        continue
                    # 偏差可接受→用实时方位角（偏差小不会螺旋，且自适应车体位移）
                    if self._force_bearing_diverging(distance):
                        self.get_logger().error(
                            f"[Stanley] force_bearing 持续背离目标（距最近点+{FORCE_BEARING_DIVERGE_DIST}m超"
                            f"{FORCE_BEARING_DIVERGE_COUNT}帧，当前{distance:.2f}m），暂停导航等待人工介入"
                        )
                        self.set_rtk_error_bits(ERROR_CALIB_TIMEOUT)
                        self.nav_context["calib_generator"] = None
                        self.nav_context["nav_state"] = NavState.PAUSE
                        self.nav_context["pre_pause_state"] = NavState.WAYPOINT_MOVE
                        self.nav_context["pause_reason"] = "force_bearing_diverge"
                        self.nav_context["manual_intervention_seen"] = False
                        self.nav_context["brush_active"] = self.brush_active
                        self.nav_running = False
                        self.publish_nav_state(NavState.PAUSE)
                        self.publish_stop_speed()
                        yield (0.0, 0.0)
                        return
                    path_direction = current_bearing
                elif t > 1.0:
                    self.nav_context["force_bearing_mode"] = True
                    self.nav_context["force_bearing_target"] = self.calculate_bearing(
                        current_lat, current_lon, target_lat, target_lon
                    )
                    continue
                elif self.nav_context.get("bearing_mode_locked"):
                    path_direction = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
                else:
                    path_direction = self.stanley_path_direction

                heading_err = self.normalize_angle(path_direction - self.imu_yaw)
                # force_bearing_mode 时航向误差来自侧向接近目标，非 IMU 异常，跳过重校准
                in_bearing_mode = self.nav_context.get("force_bearing_mode", False)
                if (path_initialized_now
                        and not in_bearing_mode
                        and abs(heading_err) > HEADING_ABNORMAL_THRESHOLD):
                    self.start_heading_recalibration(
                        path_direction,
                        heading_err,
                        "新航段首帧航向差过大，先停车校准"
                    )
                    current_nav_state = NavState.WAYPOINT_CALIB
                    yield (0.0, 0.0)
                    continue
                if abs(heading_err) > HEADING_ABNORMAL_THRESHOLD:
                    if not in_bearing_mode:
                        self.nav_context["angle_abnormal_count"] += 1
                        if self.heading_abnormal_start_time is None:
                            self.heading_abnormal_start_time = time.monotonic()
                    else:
                        self.nav_context["angle_abnormal_count"] = 0
                        self.heading_abnormal_start_time = None
                        self.heading_timed_out = False
                else:
                    self.nav_context["angle_abnormal_count"] = 0
                    self.heading_abnormal_start_time = None
                    self.heading_timed_out = False

                # 连续航向异常触发即时重校准；同航点≥2次则切换到方位角直行模式
                if self.nav_context["angle_abnormal_count"] >= ANGLE_ABNORMAL_COUNT:
                    self.nav_context["waypoint_recalib_count"] += 1
                    self.get_logger().warn(
                        f"[Stanley] 航向异常持续{self.nav_context['angle_abnormal_count']}帧："
                        f"hdg_err={heading_err:.1f}°，同航点第{self.nav_context['waypoint_recalib_count']}次校准"
                    )
                    self.nav_context["angle_abnormal_count"] = 0
                    if self.nav_context["waypoint_recalib_count"] >= 2:
                        self.nav_context["force_bearing_mode"] = True
                        self.heading_abnormal_start_time = None
                        self.heading_timed_out = False
                        target_lon, target_lat, _ = target_waypoint
                        fixed_bearing = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
                        self.nav_context["force_bearing_target"] = fixed_bearing
                        self.get_logger().warn(
                            f"[Stanley] 同航点≥2次航向异常，先原地旋转校准到目标方位{fixed_bearing:.1f}°，再直行"
                        )
                        self.start_heading_recalibration(
                            fixed_bearing,
                            self.normalize_angle(fixed_bearing - self.imu_yaw),
                            f"同航点第{self.nav_context['waypoint_recalib_count']}次航向异常"
                        )
                        current_nav_state = NavState.WAYPOINT_CALIB
                        yield (0.0, 0.0)
                        continue
                    self.start_heading_recalibration(
                        path_direction,
                        heading_err,
                        f"航向异常持续{self.nav_context['angle_abnormal_count']}帧"
                    )
                    current_nav_state = NavState.WAYPOINT_CALIB
                    yield (0.0, 0.0)
                    continue

                left_speed, right_speed = self.stanley_steering_control(
                    current_pos=current_pos,
                    current_heading=self.imu_yaw,
                    path_start=self.stanley_path_start,
                    path_end=path_end,
                    path_direction=path_direction,
                    velocity=current_base_speed,
                    distance_to_target=distance,
                    bearing_only=in_bearing_mode
                )

                if self._is_motion_blocked() or self.boundary_correct_locked:
                    # 行进方向被禁止时，首次调用会启动边界矫正状态机
                    left_speed, right_speed = self.get_boundary_correct_speed()

                if abs(self.nav_context["last_distance"] - distance) > 0.5:
                    self.nav_context["last_distance"] = distance
                    lateral_err = self.calculate_lateral_error(current_pos, self.stanley_path_start, path_end)
                    self.get_logger().info(
                        f"[Stanley] 航点{self.current_waypoint_idx}：距离{distance:.2f}m, left={left_speed:.2f}, right={right_speed:.2f}, "
                        f"lat_err={lateral_err:.3f}m, hdg_err={heading_err:.1f}°, path_dir={path_direction:.1f}°, imu={self.imu_yaw:.1f}°, t={t:.3f}"
                    )

                yield (left_speed, right_speed)
        # 所有航点完成
        self.get_logger().info("[ROSNode] RTK多点导航全部完成")
        self.nav_context["nav_state"] = NavState.COMPLETED
        self.finish_navigation_task()
        yield (0.0, 0.0)

    # ================== 原有RTKControlNode核心方法 ==================
    def publish_stop_speed(self):
        """发布停止电机速度指令（不清除brush_active，由调用方决定）"""
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
        """根据当前航点索引控制滚刷开关，支持多段区域"""
        if not hasattr(self, 'brush_start_indices') or not hasattr(self, 'brush_stop_indices'):
            return

        idx = self.current_waypoint_idx

        # 清理已过期的开启索引（因 _mid 等含 start 子串的注释产生的冗余条目）
        while self.brush_start_indices and idx > self.brush_start_indices[0]:
            stale = self.brush_start_indices.pop(0)
            self.get_logger().warn(f"[RTKNav] 跳过过期滚刷开启索引 {stale}（当前航点{idx}）")

        if self.brush_start_indices and idx >= self.brush_start_indices[0] and not self.brush_active:
            self.publish_brush_speed(RTK_BRUSH_SPEED)
            self.brush_start_indices.pop(0)

        while self.brush_stop_indices and idx > self.brush_stop_indices[0]:
            stale = self.brush_stop_indices.pop(0)
            self.get_logger().warn(f"[RTKNav] 跳过过期滚刷关闭索引 {stale}（当前航点{idx}）")

        if self.brush_stop_indices and idx >= self.brush_stop_indices[0] and self.brush_active:
            self.publish_brush_speed(0.0)
            self.brush_stop_indices.pop(0)

    def publish_nav_state(self, state: NavState):
        """发布当前导航状态"""
        state_msg = String()
        state_msg.data = state if isinstance(state, str) else state.value
        self.nav_state_pub.publish(state_msg)

    def publish_nav_context(self):
        """发布 nav_context 快照（JSON），便于调试定位问题"""
        ctx = {
            "nav_state": self.nav_context.get("nav_state", ""),
            "current_waypoint_idx": self.current_waypoint_idx,
            "total_waypoints": len(self.waypoints),
            "nav_running": self.nav_running,
            "rtk_error_code": self.rtk_error_code,
            "tilt_fault": self.nav_context.get("tilt_fault", False),
            "tilt_confirm_count": self.nav_context.get("tilt_confirm_count", 0),
            "tilt_normal_count": self.nav_context.get("tilt_normal_count", 0),
            "pre_pause_state": self.nav_context.get("pre_pause_state"),
            "pause_reason": self.nav_context.get("pause_reason"),
            "manual_intervention_seen": self.nav_context.get("manual_intervention_seen", False),
            "force_bearing_mode": self.nav_context.get("force_bearing_mode", False),
            "force_bearing_target": self.nav_context.get("force_bearing_target"),
            "bearing_mode_locked": self.nav_context.get("bearing_mode_locked", False),
            "angle_abnormal_count": self.nav_context.get("angle_abnormal_count", 0),
            "is_angle_recalib": self.nav_context.get("is_angle_recalib", False),
            "heading_timed_out": self.heading_timed_out,
            "rtk_data_timed_out": self.rtk_data_timed_out,
            "position_status": self.last_gps_status,
            "orientation_status": self.last_orientation_status,
            "position_data_valid": self.position_data_valid,
            "rtk_solution_ready": self.rtk_solution_ready,
            "control_mode": self.current_control_mode,
            "brush_active": self.brush_active,
            "ts": time.strftime("%H:%M:%S"),
        }
        msg = String()
        msg.data = json.dumps(ctx, ensure_ascii=False)
        self.nav_context_pub.publish(msg)
        self._last_nav_context_publish = time.monotonic()

    def update_rtk_error_status(self, error_code: int, force: bool = False):
        if not force and error_code == self.rtk_error_code:
            return
        self.rtk_error_code = error_code
        msg = Int16()
        msg.data = int(error_code)
        self.rtk_error_pub.publish(msg)

    def set_rtk_error_bits(self, error_bits: int):
        self.rtk_error_code = self.rtk_error_code | int(error_bits)

    def clear_rtk_error_bits(self, error_bits: int):
        self.rtk_error_code = self.rtk_error_code & ~int(error_bits)

    def get_cleaning_area_for_waypoint(self, idx: int = None) -> str:
        idx = self.current_waypoint_idx if idx is None else idx
        if 0 <= idx < len(self.waypoint_areas):
            return self.waypoint_areas[idx]
        return ""

    def update_cleaning_area(self, force: bool = False):
        area = self.get_cleaning_area_for_waypoint()
        self.current_cleaning_area = area
        if force or area != self.last_published_cleaning_area:
            area_msg = String()
            area_msg.data = area
            self.cleaning_area_pub.publish(area_msg)
            self.last_published_cleaning_area = area

    def publish_current_route_id(self):
        route_id = os.path.splitext(os.path.basename(self.rtk_path_file))[0] if self.rtk_path_file else "default"
        route_msg = String()
        route_msg.data = route_id
        self.current_route_pub.publish(route_msg)
        self.get_logger().info(f"[RTKNav] 已发布当前路径ID: {route_id}")

    def state_callback(self, msg: String):
        """电机状态回调函数：监听控制状态变化"""
        if msg.data == "HOLD" and self.last_state != "HOLD":
            self.current_control_mode = ControlMode.NORMAL
            self.get_logger().warn("[RTKNav] 电机状态为HOLD，强制停止导航")
            if self._is_manual_intervention_pause():
                self.nav_context["manual_intervention_seen"] = True
                self.get_logger().info("[RTKNav] 已确认人工接管，等待重新进入AUTO_CLEANING恢复当前航点")
            self.nav_running = False
            self.heading_abnormal_start_time = None
            self.heading_timed_out = False
            self.nav_context["calib_retry_count"] = 0
            if hasattr(self, 'nav_context'):
                self.nav_context["brush_active"] = self.brush_active
            self.publish_stop_speed()
        elif msg.data == "AUTO_CLEANING" and self.last_state != "AUTO_CLEANING":
            self.current_control_mode = ControlMode.AUTO_CLEANING
            if self._is_manual_intervention_pause():
                if not self._resume_manual_intervention_pause():
                    self.get_logger().error(
                        f"[RTKNav] 收到AUTO_CLEANING状态但尚未确认人工接管，保持PAUSE"
                        f"（pause_reason={self.nav_context.get('pause_reason')}）"
                    )
                    self.publish_stop_speed()
            elif (hasattr(self, 'nav_context')
                  and self.nav_context["nav_state"] == NavState.PAUSE
                  and self.nav_context.get("tilt_fault", False)):
                self.get_logger().warn(
                    f"[RTKNav] AUTO_CLEANING请求但跌落故障仍活跃，保持PAUSE"
                    f"（pause_reason={self.nav_context.get('pause_reason')}）"
                )
            else:
                self.get_logger().info("[RTKNav] 电机状态为AUTO_CLEANING，恢复导航")
                if self.waiting_for_next_unloading:
                    self.waiting_for_next_unloading = False
                    self.multi_waypoint_generator = None
                    self.nav_running = False
                    self.get_logger().info("[RTKNav] 清除waiting_for_next_unloading标志（state_callback）")
                if hasattr(self, 'nav_context'):
                    self.brush_active = self.nav_context.get("brush_active", False)
                    if self.brush_active:
                        self.publish_brush_speed(RTK_BRUSH_SPEED)
                    else:
                        self.publish_brush_speed(0.0)
        elif msg.data == "DISABLE" and self.last_state == "LOADING":
            self.get_logger().info("[RTKNav] LOADING完成→DISABLE，重置导航上下文")
            self.reset_nav_context()
            self.multi_waypoint_generator = None
            self.nav_running = False
            self.publish_stop_speed()
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
        self.pending_next_path_file = None
        self.waiting_for_next_unloading = False
        self.cross_file_last_waypoint = None
        self.waypoints = []
        self.waypoint_areas = []
        self.current_cleaning_area = ""
        self.update_cleaning_area(force=True)
        self.is_first_file_load = False
        self.rtk_path_file = new_path_file
        self.current_waypoint_idx = 0  # 重置航点索引为0
        self.get_logger().info(f"[RTKNav] 路径切换时重置航点索引为0")
        self.return_to_loading_added = False  # 重置出仓点追加标志
        self.brush_start_indices.clear()  # 重置滚刷开启索引队列
        self.brush_stop_indices.clear()   # 重置滚刷关闭索引队列
        self.brush_active = False         # 重置滚刷状态
        self.nav_context["nav_state"] = NavState.IDLE  # 重置导航状态为IDLE
        self.nav_context["pre_pause_state"] = None  # 重置暂停状态
        self.nav_context["pause_reason"] = None
        self.nav_context["manual_intervention_seen"] = False
        self.nav_context["calib_generator"] = None
        self.nav_context["calib_retry_count"] = 0
        self.nav_context["force_bearing_mode"] = False
        self.nav_context["force_bearing_target"] = None
        self.nav_context["force_bearing_recalib_count"] = 0
        self.nav_context["force_bearing_min_distance"] = float('inf')
        self.nav_context["distance_increase_count"] = 0
        self.nav_context["bearing_mode_locked"] = False
        self.clear_rtk_error_bits(ERROR_CALIB_TIMEOUT)
        # 路径切换时清零航向异常状态，避免旧航向计时污染新路径首段转向
        self.heading_abnormal_start_time = None
        self.heading_timed_out = False
        self.publish_nav_state(self.nav_context["nav_state"])
        self.load_waypoints_from_file(self.rtk_path_file)
        self.publish_current_route_id()
        
        # 同步滚刷状态到nav_context（加载路径后会更新brush_active）
        if hasattr(self, 'nav_context'):
            self.nav_context["brush_active"] = self.brush_active
        
        if self.waypoints:
            self.get_logger().info(f"[RTKNav] 路径切换成功，共 {len(self.waypoints)} 个航点")
        else:
            self.get_logger().error(f"[RTKNav] 路径切换失败，无法加载航点")

    def mode_callback(self, msg: String):
        """接收电机节点的控制模式, 更新自身状态"""
        previous_mode = getattr(self, '_last_mode_msg', None)
        self._last_mode_msg = msg.data
        self.current_control_mode = msg.data
        # 切换到RTK模式时, 重置IMU校准
        # if self.current_control_mode == ControlMode.AUTO_CLEANING and previous_mode != ControlMode.AUTO_CLEANING:
        #     # self.reset_imu_calibration()
        #     # 新增：强制重置导航生成器和运行状态
        #     self.multi_waypoint_generator = None
        #     self.nav_running = False
        if self.current_control_mode == ControlMode.AUTO_CLEANING and previous_mode != ControlMode.AUTO_CLEANING:
            if self.waiting_for_next_unloading:
                self.waiting_for_next_unloading = False
                self.get_logger().info("[RTKNav] 检测到下一次UNLOADING后的AUTO_CLEANING，允许启动预加载路径")
            self.multi_waypoint_generator = None
            self.nav_running = False
            # 进入RTK模式时清零航向异常状态，避免旧计时污染新导航任务
            self.heading_abnormal_start_time = None
            self.heading_timed_out = False
            self.nav_context["calib_retry_count"] = 0  # 清零校准卡滞重试计数
            # 核心修改：若之前是初始移动中断，强制保留/重置为INITIAL_MOVE
            if self.nav_context["nav_state"] not in [NavState.IDLE, NavState.WAYPOINT_MOVE, NavState.WAYPOINT_CALIB, NavState.COMPLETED, NavState.PAUSE]:
                self.nav_context["nav_state"] = NavState.INITIAL_MOVE
            if self._is_manual_intervention_pause():
                if not self._resume_manual_intervention_pause():
                    self.get_logger().error(
                        f"切换到RTK模式但尚未确认人工接管："
                        f"{self.nav_context.get('pause_reason')}，保持PAUSE"
                    )
            else:
                self.get_logger().info(f"切换到RTK模式，导航状态：{self.nav_context['nav_state']}")

        # 切换非RTK模式时, 保存导航状态, 停止导航
        if self.current_control_mode != ControlMode.AUTO_CLEANING:
            if (self._is_manual_intervention_pause()
                    and not self.nav_context.get("manual_intervention_seen", False)):
                self.nav_context["manual_intervention_seen"] = True
                self.get_logger().info(
                    f"[RTKNav] 已切出AUTO_CLEANING处理{self.nav_context.get('pause_reason')}，"
                    "下次进入AUTO_CLEANING将从当前航点恢复"
                )
            if self.multi_waypoint_generator:
                self.multi_waypoint_generator = None
            self.nav_running = False
            self.heading_abnormal_start_time = None
            self.heading_timed_out = False
            self.nav_context["calib_retry_count"] = 0  # 清零校准卡滞重试计数
            self.publish_stop_speed()
            # 新增：无航点时强制重置导航上下文，避免恢复时保留异常状态
            if not self.waypoints:
                self.reset_nav_context()
                self.get_logger().info("[ROSNode] 非RTK模式：无航点数据，重置导航上下文为IDLE")
            # 不重置导航上下文, 保存状态以便后续恢复

    def rtk_timer_callback(self):
        """10Hz定时器回调, 驱动多点导航逻辑"""
        # 周期性发布 nav_context（每2s），便于 ros2 topic echo 调试
        if time.monotonic() - self._last_nav_context_publish >= 2.0:
            self.publish_nav_context()

        self.update_cleaning_area()
        # self.is_boundary_triggered = self.get_parameter('is_boundary_triggered').value
        # 仅在RTK导航模式下执行导航逻辑
        if self.current_control_mode == ControlMode.AUTO_CLEANING:
            # 仅在RTK导航模式下上报导航错误码
            self.update_rtk_error_status(self.rtk_error_code, force=True)
            if self._is_manual_intervention_pause():
                self.multi_waypoint_generator = None
                self.nav_running = False
                self.publish_stop_speed()
                self.publish_nav_state(NavState.PAUSE)
                return
            if self.waiting_for_next_unloading:
                self.publish_stop_speed()
                self.publish_nav_state(NavState.IDLE)
                return

            if self.handle_rtk_data_timeout():
                return

            # 自动导航仅在GGA定位固定、WTRTK定向固定且数据有效时启动/继续。
            if not self.rtk_solution_ready:
                self.set_rtk_error_bits(ERROR_RTK_NOT_FIXED)
                self.multi_waypoint_generator = None
                self.nav_running = False
                self.publish_stop_speed()
                return

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
                    # 缓存最近发布的电机速度（供边界方向判断用）
                    self._last_motor_left = float(left_speed)
                    self._last_motor_right = float(right_speed)
                    # 根据滚刷状态设置速度
                    brush_speed = RTK_BRUSH_SPEED if getattr(self, 'brush_active', False) else 0.0
                    speed_msg.z = brush_speed
                    self.motor_speed_pub.publish(speed_msg)
                    velocity_msg = Float32()
                    velocity_msg.data = float(getattr(self, 'real_velocity', 0.0))
                    self.velocity_pub.publish(velocity_msg)
            except StopIteration:
                # 人工介入锁定导致的退出，不触发finish_navigation_task
                if self._is_manual_intervention_pause():
                    self.get_logger().info(
                        f"[ROSNode] {self.nav_context.get('pause_reason')}导致导航锁定暂停，"
                        "保持PAUSE等待人工切出并重新进入AUTO_CLEANING"
                    )
                    self.multi_waypoint_generator = None
                    self.nav_running = False
                else:
                    self.get_logger().info("[ROSNode] 多点导航生成器执行完毕")
                    self.finish_navigation_task()
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
            self.update_rtk_error_status(0)
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
