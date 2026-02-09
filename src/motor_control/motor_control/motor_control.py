#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from threading import Timer
import serial
import struct
import time
import os
import sys
import math
from typing import Optional, List, Dict, Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Float32, Float32MultiArray
from geometry_msgs.msg import Vector3
import traceback
import json
import subprocess
import threading
from enum import Enum
from sensor_msgs.msg import NavSatFix


sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 注：保留你的原有导入，此处省略（实际使用时直接替换原有代码即可）
from motor_control.motor_driver import CanMotorDriver
from motor_control.remote_control import SBUSRemoteController

# -------------------------- 全局配置与枚举 --------------------------
class RobotStateKey(Enum):
    STOP = "z"
    START = "x"
    FORWARD = "w"
    BACKWARD = "s"
    TURN_LEFT = "a"
    TURN_RIGHT = "d"
    LOADING = "l"
    UNLOADING = "u"
    RTK_NAV = "r"

STATE_DICT = {e.value: e.name for e in RobotStateKey}  # {'z':'STOP', 'x':'START'...}

MAX_SPEED = 10.0   # 遥控器最大速度
MIN_SPEED = -10.0  # 遥控器最小速度
BRUSH_SPEED = 15.0
CH2_SENSITIVITY = 1.0  # 前进后退灵敏度
CH3_SENSITIVITY = 0.5  # 左右旋转灵敏度
DEAD_ZONE = 0.05       # 控制死区
RC_CH_MAX_VALUE = 1722

# 新增：定义RTK Fixed最大等待时间（可根据实际需求调整，如30s）
MAX_GPS_WAIT_TIME = 30.0

#PID参数
MAX_CORRECTION = 0.5

# -------------------------- 电机控制节点（独立ROS2节点） --------------------------
class MotorControlNode(Node):
    def __init__(self, node_name='motor_control_node'):
        super().__init__(node_name)

        # 循环频率：10Hz（兼容原有逻辑，可调整）
        self.rate = self.create_rate(10)
        self.last_yaw_error = 0.0  # 上一次的航向误差

        # UNLOADING parameters
        self.unloading_forword_threshold = 10.0 # seconds
        self.unloading_turn_start_time = None
        self.unloading_turn_time_max = 30.0
        self.unloading_phase = None  # None/"FORWARD"/"UNLOADING_TURN"/"COMPLETE"
        self.unloading_start_time = 0.0  # 出仓开始时间
        self.unloading_turn_target_deg = 0.0  # 出仓转向目标角
        self.unloading_timer: Optional[Timer] = None  # 出仓专用定时器

        self.loading_turn_target_deg = -169.24    #-153.05 #-159.63  # 进仓转向目标角
        self.loading_backward_threshold = 10.0  # 进仓后退时长（秒）
        self.loading_turn_time = 30.0
        self.loading_phase = None
        self.loading_start_time = 0.0
        self.loading_timer: Optional[Timer] = None
        self.loading_backward_start_time = 0.0

        self.yaw_diff_min = 0.5 # 0.1 degree
        # 新增：IMU角度更新时间戳，用于过滤旧数据
        self.last_imu_update_time = 0.0
        self.imu_update_interval = 0.2  # 要求IMU至少100ms更新一次（适配常见IMU发布频率）

        self.nav_status = None

        self.battery_total_voltage = None  # 电池总电压
        self.battery_current = None  # 电池电流
        self.battery_remaining = None # 电池百分比
        self.battery_temperatures = [] # 电池温度，共3个

        # 1. 初始化电机控制模块
        self.motor_ctrl = CanMotorDriver(node_name='can_motor_driver', channel='can0', interface='socketcan', baudrate=1000000)
        self.get_logger().info("[ROSNode] 开始初始化CAN串口...")
        
        # 2. 初始化遥控器模块
        self.sbus_remote = SBUSRemoteController()
        if not self.sbus_remote.is_connected:
            self.get_logger().warn("[ROSNode] 遥控器串口初始化失败，仅支持RTK和键盘控制")
        self.current_location = None
        self.rtk_status = 0  # 初始化RTK状态为0（无效状态）
        self.current_lon = 0.0
        self.current_lat = 0.0
        #UNLOADING完成后GPS坐标记录（用于后续验证和日志）
        self.unloading_lon = None
        self.unloading_lat = None

        # 3. ROS2 订阅器
        self.keyboard_sub = self.create_subscription(
            String,
            "/keyboard/control",
            self.keyboard_callback,
            10  # QoS深度
        )
        self.rtk_speed_sub = self.create_subscription(
            Vector3,
            "/rtk/motor_speed",
            self.rtk_speed_callback,
            10
        )
        self.rtk_nav_sub = self.create_subscription(
            String,
            "/rtk/nav_state",
            self.rtk_nav_status_callback,
            10
        )
        self.io_subscription = self.create_subscription(
            UInt8,
            "io_data",
            self.io_data_callback,
            10
        )
        self.imu_heading_sub = self.create_subscription(
            Float32,
            "imu_heading",
            self.imu_heading_callback,
            10
        )
        self.battery_subscription = self.create_subscription(
            Float32MultiArray,
            "battery_data",
            self.battery_callback,
            10
        )
        # 4. ROS2 发布器
        self.state_pub = self.create_publisher(String, "/motor/state", 10)  # 电机状态
        self.speed_pub = self.create_publisher(Vector3, "/motor/current_speed", 10)  # 电机当前速度
        self.unloading_gps_pub = self.create_publisher(Vector3, "/unloading_gps", 10)  # 电机当前速度
        self.mode_pub = self.create_publisher(String, "/control/mode", 10)  # 当前控制模式
        self.robot_state_pub = self.create_publisher(String, "/robot_state", 10)  # mqtt msg
        self.gps_sub = self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)

        # 全局变量
        self.current_control_mode = "NORMAL"  # 默认普通模式
        self.rtk_left_speed = 0.0  # 存储RTK订阅的左轮速度
        self.rtk_right_speed = 0.0 # 存储RTK订阅的右轮速度

        self.status_list = [
            "STOP", "START", "FORWARD", "BACKWARD", "LOADING", "UNLOADING",
            "TURN_LEFT", "TURN_RIGHT", "PAUSE", "RTK_NAV"
        ]
        self.current_status = self.status_list[0]

        # 新增：进出仓状态标记（用于优先级判断，解决速度穿插问题）
        self.is_in_bin_process = False  # True=正在进出仓，False=正常状态
        # 新增：进出仓暂停与来源模式（用于切换模式时暂停/恢复流程）
        self.bin_process_paused = False
        self.bin_process_origin_mode = None

        # 确保IMU航向默认值存在
        self.imu_yaw_deg = None

        # 初始化电机（进入START状态）
        self.switch_state('x')

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1秒 = 10Hz

        # add mqtt 
        self.main_board = True # 主控板状态MQTT
        self.imu_sensor = True # IMU传感器状态MQTT
        self.motor_driver = True # 电机驱动器状态MQTT

        self.robot_cmd_subscription = self.create_subscription(
            String,
            "robot_cmd",
            self.status_callback,
            10
        )

    def gps_callback(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            self.get_logger().warn("GPS信号无效")
            return

        status_map = {1: "弱", 2: "差分", 3: "RTK", 4: "RTK固定解"}
        if msg.status.status in status_map:
            self.get_logger().debug(f"GPS状态：{status_map[msg.status.status]}")

        self.rtk_status = msg.status.status
        self.current_lon = msg.longitude
        self.current_lat = msg.latitude

    def imu_heading_callback(self, msg:Float32):
        """IMU航向回调 - 新增时间戳，过滤旧数据"""
        self.imu_yaw_deg = msg.data
        self.last_imu_update_time = time.time()  # 记录最新更新时间
        # 确保角度归一化到[-180, 180]
        self.imu_yaw_deg = (self.imu_yaw_deg + 180) % 360 - 180

    def timer_callback(self):
        self.current_control_mode = self.sbus_remote.control_mode
        # 检查CAN串口是否正常打开
        if not self.motor_ctrl.bus:
            # self.get_logger().warn("[ROSNode] CAN串口连接断开，尝试重连...")
            self.motor_ctrl.reconnect_can_bus()

        # 1. 发布当前控制模式（给RTK节点）
        mode_msg = String()
        mode_msg.data = self.current_control_mode if isinstance(self.current_control_mode, str) else "NORMAL"
        self.mode_pub.publish(mode_msg)

        # 进出仓流程的暂停/恢复逻辑：
        # - 当正在进/出仓（is_in_bin_process=True）且当前控制模式不等于流程发起时，暂停流程；
        # - 当控制模式恢复到流程发起模式时，恢复流程；
        # 暂停时不取消定时器，仅停止电机并在 handler 中短路，从而可在回到原模式后继续。
        if self.is_in_bin_process:
            # 如果尚未记录来源模式，则以当前模式作为来源
            if self.bin_process_origin_mode is None:
                self.bin_process_origin_mode = self.current_control_mode

            if self.current_control_mode != self.bin_process_origin_mode:
                # 需要暂停进/出仓流程
                if not self.bin_process_paused:
                    self.get_logger().info(f"[ROSNode] 进/出仓流程暂停（从{self.bin_process_origin_mode}切换到{self.current_control_mode}）")
                    # 停止电机以便人工/其他模式接管
                    try:
                        self.set_motors_speed(0.0, 0.0)
                        self.set_brush_speed(0.0)
                    except Exception:
                        pass
                    self.bin_process_paused = True
                # 当暂停时，让其他模式的控制逻辑继续（例如 REMOTE、RTK_NAV），不返回
            else:
                # 回到来源模式 -> 恢复流程（如果之前暂停过）
                if self.bin_process_paused:
                    self.get_logger().info(f"[ROSNode] 进/出仓流程恢复（回到{self.bin_process_origin_mode}）")
                    self.bin_process_paused = False
                # 继续执行下面的控制分支

        # 2. 按控制模式执行不同逻辑（仅非进出仓状态生效）
        if self.current_control_mode == "REMOTE":
            try:
                ch2_norm = self.sbus_remote.get_channel_normalized(ch_idx=2)  # 前进后退
                ch0_norm = self.sbus_remote.get_channel_normalized(ch_idx=0)  # 左右旋转

                ch2_norm = 0.0 if abs(ch2_norm) < DEAD_ZONE else ch2_norm
                ch0_norm = 0.0 if abs(ch0_norm) < DEAD_ZONE else ch0_norm

                forward_backward_right = ch2_norm * MAX_SPEED * CH2_SENSITIVITY
                forward_backward_left = -forward_backward_right

                rotate_left_right = -ch0_norm * MAX_SPEED * CH3_SENSITIVITY

                left_speed_target = forward_backward_left + rotate_left_right
                right_speed_target = forward_backward_right + rotate_left_right

                left_speed = max(MIN_SPEED, min(MAX_SPEED, left_speed_target))
                right_speed = max(MIN_SPEED, min(MAX_SPEED, right_speed_target))

                self.set_motors_speed(left_speed, right_speed)
                ch6_norm = self.sbus_remote.get_channel_normalized(ch_idx=6)
                ch6_norm = 1.0 if ch6_norm == 1.0 else 0.0
                brush_speed = -ch6_norm * BRUSH_SPEED
                self.set_brush_speed(brush_speed)
            except Exception as e:
                self.get_logger().warn(f"[ROSNode] 获取遥控器速度失败：{e}")
                self.set_motors_speed(0.0, 0.0)
                self.set_brush_speed(0.0)

        elif self.current_control_mode == "NORMAL":
            state_msg = String()
            state_msg.data = str(self.current_status)
            self.state_pub.publish(state_msg)

            # 按当前状态赋值速度
            if self.current_status == "FORWARD":
                left_speed = -self.motor_ctrl.BASE_SPEED
                right_speed = self.motor_ctrl.BASE_SPEED
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status == "BACKWARD":
                left_speed = self.motor_ctrl.BASE_SPEED
                right_speed = -self.motor_ctrl.BASE_SPEED
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status == "TURN_LEFT":
                left_speed = self.motor_ctrl.BASE_SPEED
                right_speed = self.motor_ctrl.BASE_SPEED
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status == "TURN_RIGHT":
                left_speed = -self.motor_ctrl.BASE_SPEED
                right_speed = -self.motor_ctrl.BASE_SPEED
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status in ["STOP"]:
                left_speed = 0.0
                right_speed = 0.0
                self.set_motors_speed(left_speed, right_speed)
                self.set_brush_speed(0.0)
            # self.set_motors_speed(left_speed, right_speed)
            # stop brush

        elif self.current_control_mode == "RTK_NAV":
            left_speed = self.rtk_left_speed 
            right_speed = self.rtk_right_speed 
            self.set_motors_speed(left_speed, right_speed)
            self.get_logger().debug(f"[RTKControl] 左轮：{left_speed:.2f}，右轮：{right_speed:.2f}")
            # start brush
            self.set_brush_speed(BRUSH_SPEED)

    def keyboard_callback(self, msg: String) -> None:
        """键盘控制回调（新增RTK模式切换）"""
        key = msg.data.strip().lower()

        # 模式切换指令
        if key == 'r':
            # 切换到RTK导航模式（进出仓状态下禁止切换）
            if not self.is_in_bin_process:
                self.current_control_mode = "RTK_NAV"
                self.get_logger().info(f"[ROSNode] 控制模式切换：→ RTK_NAV")
            else:
                self.get_logger().warn("[ROSNode] 正在进出仓，禁止切换到RTK模式")
        elif key == 'n':
            # 切回普通模式
            self.current_control_mode = "NORMAL"
            self.get_logger().info(f"[ROSNode] 控制模式切换：→ NORMAL")
        elif key == 'm':
            # 切换到遥控器模式
            self.current_control_mode = "REMOTE"
            self.get_logger().info(f"[ROSNode] 控制模式切换：→ REMOTE")
        # 原有状态切换逻辑（仅非RTK模式生效）
        elif key in STATE_DICT:
            if self.current_control_mode != "RTK_NAV":
                self.switch_state(key)
            else:
                self.get_logger().warn("[ROSNode] 当前为RTK导航模式，忽略键盘状态指令")
        else:
            self.get_logger().warn(f"[ROSNode] 无效键盘指令：{key}，支持指令：{list(STATE_DICT.keys()) + ['r(RTK)', 'n(普通)', 'm(遥控)']}")

    def rtk_speed_callback(self, msg: Vector3):
        """订阅RTK节点的速度指令，更新本地速度变量"""
        self.rtk_left_speed = msg.x
        self.rtk_right_speed = msg.y
        self.get_logger().debug(f"[RTKSpeed] 左轮：{self.rtk_left_speed:.2f}，右轮：{self.rtk_right_speed:.2f}")
    
    def rtk_nav_status_callback(self, msg: String):
        """订阅RTK导航状态消息（备用）"""
        self.nav_status = msg.data.strip()
        if self.nav_status == "COMPLETED" and not self.is_in_bin_process:
            self.switch_state('l')

    def io_data_callback(self, msg: UInt8):
        """处理IO数据回调（可根据需要扩展功能）"""
        if self.current_control_mode != "RTK_NAV" and not self.is_in_bin_process:
            self.front_left = (msg.data & 0x01) == 0x01
            self.front_right = (msg.data & 0x02) == 0x02
            self.mid_left = (msg.data & 0x04) == 0x04
            self.mid_right = (msg.data & 0x08) == 0x08
            self.back_left = (msg.data & 0x10) == 0x10
            self.back_right = (msg.data & 0x20) == 0x20
            # 按位或结果存储传感器状态
            self.sensors_status = self.front_left | self.front_right<<1 | self.mid_left<<2 | self.mid_right<<3 | self.back_left<<4 | self.back_right<<5 
            self.sensors_status = ~self.sensors_status & 0x3F  # 取反并保留6位
            # self.get_logger().info(f"[IOData] 传感器状态：{self.sensors_status:06b}")


    def battery_callback(self, msg):
        """订阅电池数据的回调函数（修正版）"""
        if len(msg.data) < 3:
            self.get_logger().warn("警告：订阅到的电池数据不完整，跳过解析")
            return
        
        self.battery_remaining = msg.data[0]  # 电池百分比
        self.battery_current = round(msg.data[1], 2)  # 总电流（索引1）
        self.battery_total_voltage = round(msg.data[2], 2)  # 总电压（索引2）
    
    def get_heading_error(self, target_heading: float) -> float:
        """修正：计算当前航向与目标航向的误差（严格归一化到[-180, 180]，单位：deg）"""
        if self.imu_yaw_deg is None:
            return 0.0
        
        # 确保目标角度也归一化
        target_heading = (target_heading + 180) % 360 - 180
        current_heading = self.imu_yaw_deg
        
        # 计算最短角度差
        error = target_heading - current_heading
        error = (error + 180) % 360 - 180
        
        return error

    def get_speed_correction(self, target_heading: float) -> float:
        """修正：优化PID，修复角度差计算，避免反向修正"""
        # 前置校验：IMU数据是否有效且最新
        # if self.imu_yaw_deg is None or (time.time() - self.last_imu_update_time) > self.imu_update_interval:
        #     self.get_logger().warn("[Correction] IMU数据过期/无效，跳过修正")
        #     return 0.0

        yaw_error = self.get_heading_error(target_heading)
        yaw_error_abs = abs(yaw_error)

        # 误差死区：小于0.3度不修正
        if yaw_error_abs < 0.3:
            self.last_yaw_error = 0.0
            return 0.0

        # 分段KP参数（优化小误差修正，避免累积）
        if yaw_error_abs > 60:
            kp = 0.08  # 大误差：快速转向
        elif yaw_error_abs >40: #20:
            kp = 0.02  # 中误差：稳定修正
        else:
            kp = 0.005  # 小误差：精准修正

        # KD参数（阻尼，抑制波动）
        kd = 0.05
        yaw_error_diff = yaw_error - self.last_yaw_error
        d_term = kd * yaw_error_diff

        # 计算修正量（方向正确，无负号反转）
        correction = (kp * yaw_error) + d_term
        # 限制最大修正量
        correction_clamped = -max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        # 日志输出（便于调试）
        if abs(yaw_error - self.last_yaw_error) > 0.1:
            self.get_logger().info(f"进出仓对正yaw_error={yaw_error:.2f}，修正量={correction_clamped:.2f}")
        
        self.last_yaw_error = yaw_error
        return correction_clamped

    def switch_state(self, key: str) -> None:
        """状态机切换逻辑（新增进出仓状态标记）"""
        if key not in STATE_DICT:
            self.get_logger().warn(f"[ROSNode] 无效状态切换key：{key} (仅支持: {list(STATE_DICT.keys())})")
            return
        new_state = STATE_DICT[key]
        if new_state not in self.status_list:
            self.get_logger().warn(f"[ROSNode] 无效状态切换请求：{new_state}")
            return
        if new_state == self.current_status:
            self.get_logger().info(f"[ROSNode] 已处于{new_state}状态，无需切换")
            return

        self.get_logger().info(f"[ROSNode] 状态切换：{self.current_status} → {new_state}")
        self.current_status = new_state

        # 核心修复2：设置进出仓状态标记
        if new_state in ["UNLOADING", "LOADING"]:
            # 记录流程开始时的控制模式，作为暂停/恢复判断的来源模式
            self.bin_process_origin_mode = self.current_control_mode
            self.bin_process_paused = False
            self.is_in_bin_process = True
        elif new_state == "STOP" and self.is_in_bin_process:
            self.is_in_bin_process = False
            self.bin_process_origin_mode = None
            self.bin_process_paused = False

        # 状态执行逻辑
        if new_state == "STOP":
            # 停止：失能所有电机
            for motor in self.motor_ctrl.motors:
                self.motor_ctrl.motor_set_speed(motor["id"], 0.0)
                time.sleep(0.01)
                # 测试暂停失能
                # self.motor_ctrl.motor_disable(motor["id"])

        elif new_state == "START":
            # 启动：仅使能电机，不运动
            self.motor_ctrl.initialize_motors()
            time.sleep(0.001)

        elif new_state == "FORWARD":
            # 前进：双电机正转
            left_speed = -self.motor_ctrl.BASE_SPEED
            right_speed = self.motor_ctrl.BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)

        elif new_state == "BACKWARD":
            # 后退：双电机反转
            left_speed = self.motor_ctrl.BASE_SPEED
            right_speed = -self.motor_ctrl.BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
        elif new_state == "TURN_LEFT":
            # 左转
            left_speed = self.motor_ctrl.BASE_SPEED
            right_speed = self.motor_ctrl.BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
        elif new_state == "TURN_RIGHT":
            # 右转
            left_speed = -self.motor_ctrl.BASE_SPEED
            right_speed = -self.motor_ctrl.BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
        elif new_state == "UNLOADING":
            self.get_logger().info("[ROSNode] 进入出仓状态，启动出仓定时器")
            self.current_status = new_state
            # 初始化出仓阶段
            self.unloading_phase = "UNLOADING_FORWARD"  # 第一阶段：前进
            self.unloading_start_time = time.time()
            # 修正：降低定时器频率到100ms，匹配IMU更新频率
            self.unloading_timer = self.create_timer(0.1, self.handle_unloading_step)
        elif new_state == "LOADING":
            # self.current_control_mode = "NORMAL"
            self.get_logger().info("[ROSNode] 进入进仓状态，启动进仓定时器")
            self.current_status = new_state
            self.loading_phase = "LOADING_TURN"  # 第一阶段：调整角度
            self.loading_start_time = time.time()
            
            # 修正：目标角度归一化
            self.loading_turn_target_deg = (self.loading_turn_target_deg + 180) % 360 - 180
            # 修正：降低定时器频率到100ms，匹配IMU更新频率
            self.loading_timer = self.create_timer(0.1, self.handle_loading_step)
        elif new_state == "RTK_NAV":
            self.current_control_mode = "RTK_NAV"
            self.get_logger().info("[ROSNode] 切换到RTK导航模式，等待RTK速度指令")
            # 速度由RTK回调处理

        # 发布当前状态
        state_msg = String()
        state_msg.data = self.current_status
        self.state_pub.publish(state_msg)

    def publish_state(self):
        """发布机器人状态消息（MQTT）"""
        try:
            state_msg = {
                "status": self.current_status,
                "battery": self.battery_remaining,
                "battery_total_voltage": self.battery_total_voltage,
                "battery_current": self.battery_current,
                "imu_yaw": self.imu_yaw_deg if self.imu_yaw_deg is not None else 0.00,
                "rtk_status": self.rtk_status,
                "current_lon": self.current_lon,
                "current_lat": self.current_lat,
                "sensors_status":self.sensors_status,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            }
            ros_string_msg = String()
            ros_string_msg.data = json.dumps(state_msg, ensure_ascii=False)
            self.robot_state_pub.publish(ros_string_msg)
            self.get_logger().info(f"[ROSNode] 成功发布状态: {ros_string_msg.data}")
        except Exception as e:
            error_msg = {
                "status": "ERROR",
                "error_detail": str(e),
                "traceback": traceback.format_exc(),
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            }
            error_ros_msg = String()
            error_ros_msg.data = json.dumps(error_msg, ensure_ascii=False)
            self.robot_state_pub.publish(error_ros_msg)
            self.get_logger().error(f"[ROSNode] 发布状态失败: {str(e)}\n{traceback.format_exc()}")

    def status_callback(self, msg):
        """处理状态消息（完整修复：逻辑纠正+异常防护+类型校验）"""
        # 修复问题3：正确的ROS2消息类型校验
        if not isinstance(msg, String):
            self.get_logger().error(f"[ROSNode] 无效消息类型：{type(msg)}，仅支持std_msgs/String")
            return
        
        # 确保msg.data是字符串类型，避免后续处理异常
        msg_data = msg.data.strip() if isinstance(msg.data, str) else ""
        if not msg_data:
            self.get_logger().warn("[ROSNode] 空消息，跳过处理")
            return

        try:
            cmd_obj = json.loads(msg_data)
            
            # 修复问题1：先校验cmd_obj是否是字典类型，再查找command字段
            if not isinstance(cmd_obj, dict):
                self.get_logger().warn(f"[ROSNode] 消息不是JSON字典格式：{msg_data}，跳过字典解析")
                raise json.JSONDecodeError("Not a JSON dict", msg_data, 0)  # 主动抛错，进入下方字符串处理逻辑
            
            if "command" not in cmd_obj:
                self.get_logger().warn(f"[ROSNode] 未找到command字段: {msg_data}")
                return
            
            command = cmd_obj.get("command", "").strip()
            self.get_logger().info(f"[ROSNode] 解析到command：{command}")

            if command == "GET_STATUS":
                self.publish_state()
            elif command in STATE_DICT:
                self.switch_state(command)
            else:
                self.get_logger().warn(f"[ROSNode] 不支持的command：{command} (支持: GET_STATUS/{list(STATE_DICT.keys())})")
                return
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"[ROSNode] 消息不是有效JSON字典，尝试按字符串处理: {msg_data}, 错误: {e}")
            raw_command = msg_data.strip()
            if raw_command in STATE_DICT:
                self.switch_state(raw_command)
            else:
                self.get_logger().error(f"[ROSNode] 无效的字符串指令：{raw_command} (仅支持: {list(STATE_DICT.keys())})")
            self.publish_state()
        except Exception as e:
            # 修复问题2：移除ROS2不支持的exc_info参数，直接拼接异常信息和堆栈（可选：手动打印堆栈）
            self.get_logger().error(f"[ROSNode] 处理消息时发生未知错误: {msg_data}, 错误: {str(e)}, 堆栈信息: {traceback.format_exc()}")
            # 可选：如果需要打印完整堆栈，需先导入traceback模块（import traceback）
            self.publish_state()
    def get_adaptive_turn_speed(self, yaw_error_abs: float) -> float:
        """
        分级自适应转向基准速度（核心：大误差快，小误差慢）
        无需减小PID参数，通过基准速度分级实现快慢切换
        """
        if yaw_error_abs > 30:
            return 0.8 * self.motor_ctrl.BASE_SPEED # type: ignore # 大误差（>30°）：快速转向
        elif yaw_error_abs > 10:
            return 0.5 * self.motor_ctrl.BASE_SPEED   # 中误差（10°~30°）：中等速度
        else:
            return 0.2 * self.motor_ctrl.BASE_SPEED  # 小误差（<10°）：慢速转向，防止超调
        

    def handle_unloading_step(self):
        """出仓分步处理（修正：适配IMU更新频率，修复角度计算）"""
        # 如果流程被暂停，短路返回，等待恢复
        if self.bin_process_paused:
            return

        if self.unloading_phase is None:
            self.get_logger().warn("[ROSNode] 出仓阶段未初始化，停止定时器")
            self.unloading_timer.cancel()
            self.unloading_timer = None
            self.is_in_bin_process = False  # 重置进出仓标记
            return
        
        # 前置校验：IMU数据是否有效且最新
        # if self.imu_yaw_deg is None or (time.time() - self.last_imu_update_time) > self.imu_update_interval:
        #     self.get_logger().warn("[UNLOADING] IMU数据过期/无效，跳过本次处理")
        #     return
        
        current_time = time.time()
        
        # ========== 阶段1：前进 ==========
        if self.unloading_phase == "UNLOADING_FORWARD":
            if current_time - self.unloading_start_time < self.unloading_forword_threshold:
                correction = 0  # 直线纠偏待添加
                left_speed = -self.motor_ctrl.BASE_SPEED + correction
                right_speed = self.motor_ctrl.BASE_SPEED + correction
                self.set_motors_speed(left_speed, right_speed)
            else:
                self.get_logger().info("[UNLOADING] 前进阶段完成，进入转向阶段")
                self.unloading_phase = "UNLOADING_TURN"
                self.unloading_turn_start_time = current_time
                
                # 修正：目标角度归一化
                self.unloading_turn_target_deg = self.imu_yaw_deg + 90
                self.unloading_turn_target_deg = (self.unloading_turn_target_deg + 180) % 360 - 180
                self.get_logger().info(f"[UNLOADING] 转向目标角设定为{self.unloading_turn_target_deg:.2f}deg")

        # ========== 阶段2：转向 ==========
        elif self.unloading_phase == "UNLOADING_TURN":            
            
            correction = self.get_speed_correction(self.unloading_turn_target_deg)  # 目标航向180度（假设出仓方向为正后方）
            # 修正：角度差计算（使用归一化后的误差）
            yaw_diff = self.get_heading_error(self.unloading_turn_target_deg)
            turn_speed = self.get_adaptive_turn_speed(yaw_diff)
            # self.motor_ctrl.BASE_SPEED
            self.get_logger().info(f"[UNLOADING] 转向阶段 - 当前航向{self.imu_yaw_deg:.2f}deg，目标{self.unloading_turn_target_deg:.2f}deg，差值{yaw_diff:.2f}deg")
            
            left_speed = -1.0 * turn_speed + correction
            right_speed = -1.0 * turn_speed + correction
            self.set_motors_speed(left_speed, right_speed)
            if abs(yaw_diff) < self.yaw_diff_min:
                self.get_logger().info("[UNLOADING] 转向阶段完成，出仓结束")
                self.unloading_phase = "COMPLETE"
                self.set_motors_speed(0.0, 0.0)
                self.unloading_timer.cancel()
                self.unloading_timer = None
                self.is_in_bin_process = False  # 重置进出仓标记
            elif current_time - self.unloading_turn_start_time > self.unloading_turn_time_max:
                self.get_logger().warn(f"[UNLOADING] Timeout: 转向阶段超时，强制完成出仓")
                self.unloading_phase = "COMPLETE"
                self.is_in_bin_process = False  # 重置进出仓标记
        
        # ========== 阶段3：完成 ==========
        elif self.unloading_phase == "COMPLETE":
            # 新增：初始化超时计时（仅首次进入该分支时初始化）
            if not hasattr(self, 'unloading_gps_wait_start'):
                self.unloading_gps_wait_start = self.get_clock().now()
                self.get_logger().info("[UNLOADING] 开始等待GPS固定解，超时时间30s")
            
            elapsed_time = (self.get_clock().now() - self.unloading_gps_wait_start).nanoseconds / 1e9

            # 1. 优先：获取RTK Fixed固定解，正常收尾
            if self.rtk_status == 4:  # RTK固定解，GPS数据可靠
                self.get_logger().info(f"[UNLOADING] 出仓完成，当前GPS坐标：经度{self.current_lon:.6f}，纬度{self.current_lat:.6f}")
                self.unloading_lon = self.current_lon
                self.unloading_lat = self.current_lat
                self.get_logger().info("[UNLOADING] 出仓流程完成")
                # 清理超时标记
                if hasattr(self, 'unloading_gps_wait_start'):
                    delattr(self, 'unloading_gps_wait_start')
                # 终止定时器
                self.unloading_timer.cancel()
                self.unloading_timer = None
                self.is_in_bin_process = False  # 重置进出仓标记
            # 2. 新增：超时兜底（GPS长期无固定解，强制退出）
            elif elapsed_time > MAX_GPS_WAIT_TIME:
                self.get_logger().error(f"[UNLOADING] 等待GPS固定解超时（{MAX_GPS_WAIT_TIME}s），状态码始终为{self.rtk_status}，强制完成出仓流程")
                # 可选：记录当前非固定解的坐标（或置空）
                self.unloading_lon = self.current_lon
                self.unloading_lat = self.current_lat
                # 清理超时标记+终止定时器
                delattr(self, 'unloading_gps_wait_start')
                self.unloading_timer.cancel()
                self.unloading_timer = None
                self.is_in_bin_process = False
            # 3. 未超时+非固定解：继续等待，可选降频打印日志（避免刷屏）
            else:
                # 每3s打印一次警告（替代0.1Hz高频打印）
                if int(elapsed_time) % 3 == 0 and abs(elapsed_time - int(elapsed_time)) < 0.1:
                    self.get_logger().warn(f"[UNLOADING] 出仓完成，GPS状态不佳（状态码{self.rtk_status}），已等待{elapsed_time:.1f}s，继续等待...")
                elif self.unloading_phase == "COMPLETE":
                    # get_gps
                    if self.rtk_status == 4:  # RTK固定解，GPS数据可靠
                        self.get_logger().info(f"[UNLOADING] 出仓完成，当前GPS坐标：经度{self.current_lon:.6f}，纬度{self.current_lat:.6f}")
                        self.unloading_lon = self.current_lon
                        self.unloading_lat = self.current_lat
                        heading = self.imu_yaw_deg + 90.0 if self.imu_yaw_deg is not None else 0.00
                        heading = (heading + 360) % 360  # 归一化到0-360度
                        # pub unloading result
                        unloading_gps_msg = Vector3()
                        unloading_gps_msg.x = self.unloading_lon
                        unloading_gps_msg.y = self.unloading_lat
                        unloading_gps_msg.z = heading
                        self.unloading_gps_pub.publish(unloading_gps_msg)
                        self.get_logger().info("[UNLOADING] 出仓流程完成")
                        self.unloading_timer.cancel()
                        self.unloading_timer = None
                        self.is_in_bin_process = False  # 重置进出仓标记
                    else:
                        self.get_logger().warn(f"[UNLOADING] 出仓完成，但GPS状态不佳（状态码{self.rtk_status}），无法获取可靠坐标,继续等待GPS修正")

            
    # def handle_loading_step(self):
    #     """进仓分步处理（核心修复：角度计算、IMU校验、频率适配）"""
    #     # 如果流程被暂停，短路返回，等待恢复
    #     if self.bin_process_paused:
    #         return

    #     if self.loading_phase is None:
    #         self.get_logger().warn("[ROSNode] 进仓阶段未初始化，停止定时器")
    #         if self.loading_timer is not None:
    #             self.loading_timer.cancel()
    #             self.loading_timer = None
    #         self.is_in_bin_process = False  # 重置进出仓标记
    #         self.bin_process_origin_mode = None
    #         self.bin_process_paused = False
    #         return
        
    #     # 前置校验：IMU数据是否有效且最新
    #     # if self.imu_yaw_deg is None or (time.time() - self.last_imu_update_time) > self.imu_update_interval:
    #     #     self.get_logger().warn("[LOADING] IMU数据过期/无效，跳过本次处理")
    #     #     return
        
    #     try:
    #         current_time = time.time()

    #         # ========== 阶段1：调整进仓角度 ==========
    #         if self.loading_phase == "LOADING_TURN":                
                
    #             # 修正：使用归一化的角度差计算
    #             correction = self.get_speed_correction(self.loading_turn_target_deg)  # 目标航向为180度（假设进仓方向为正后方）
    #             yaw_diff = self.get_heading_error(self.loading_turn_target_deg)
    #             turn_speed = self.get_adaptive_turn_speed(yaw_diff)
    #             left_speed = turn_speed + correction
    #             right_speed = turn_speed + correction
    #             self.set_motors_speed(left_speed, right_speed)
    #             # self.get_logger().info(
    #             #     f"[LOADING] 角度调整阶段 - 当前航向{self.imu_yaw_deg:.2f}deg，"
    #             #     f"目标{self.loading_turn_target_deg:.2f}deg，差值{yaw_diff:.2f}deg"
    #             # )
                
    #             if abs(yaw_diff) < self.yaw_diff_min:
    #                 self.get_logger().info("[LOADING] 角度调整完成，进入后退进仓阶段")
    #                 self.loading_phase = "LOADING_BACKWARD"
    #                 self.loading_backward_start_time = current_time
    #             elif current_time - self.loading_start_time > self.loading_turn_time:
    #                 self.get_logger().warn("[LOADING] 角度调整超时，强制进入后退阶段")
    #                 self.loading_phase = "LOADING_BACKWARD"
    #                 self.loading_backward_start_time = current_time
            
    #         # ========== 阶段2：后退进仓 ==========
    #         elif self.loading_phase == "LOADING_BACKWARD":
    #             if current_time - self.loading_backward_start_time < self.loading_backward_threshold:
    #                 correction = 0
    #                 left_speed = self.motor_ctrl.BASE_SPEED + correction
    #                 right_speed = -(self.motor_ctrl.BASE_SPEED + correction)
    #                 self.set_motors_speed(left_speed, right_speed)
    #                 self.get_logger().info(f"[LOADING] 后退进仓阶段 - 已持续{current_time - self.loading_backward_start_time:.1f}秒")
    #             else:
    #                 self.get_logger().info("[LOADING] 后退进仓完成，进入完成阶段")
    #                 self.loading_phase = "COMPLETE"
            
    #         # ========== 阶段3：进仓完成 ==========
    #         elif self.loading_phase == "COMPLETE":
    #             self.get_logger().info("[LOADING] 进仓流程完成")
    #             self.set_motors_speed(0, 0)
    #             self.loading_timer.cancel()
    #             self.loading_timer = None
    #             self.nav_status = "IDLE"
    #             self.is_in_bin_process = False  # 重置进出仓标记
    #     except Exception as e:
    #         self.get_logger().error(f"[LOADING] 执行异常：{str(e)}")
    #         self.is_in_bin_process = False  # 异常时也重置标记
    def handle_loading_step(self):
        """进仓分步处理（核心修复：目标一致+稳定判定+后退停修正+频率正常）"""
        if self.bin_process_paused:
            return
        if self.loading_phase is None:
            self.get_logger().warn("[ROSNode] 进仓阶段未初始化，停止定时器")
            if self.loading_timer is not None:
                self.loading_timer.cancel()
                self.loading_timer = None
            self.is_in_bin_process = False
            self.bin_process_origin_mode = None
            self.bin_process_paused = False
            return
        # IMU数据校验
        # if self.imu_yaw_deg is None or (time.time() - self.last_imu_update_time) > self.imu_update_interval:
        #     self.get_logger().warn("[LOADING] IMU数据过期/无效，跳过本次处理")
        #     return
        
        try:
            current_time = time.time()
            # 新增：稳定达标计数器（类内变量，初始化在__init__里，下面会说）
            if not hasattr(self, 'yaw_stable_count'):
                self.yaw_stable_count = 0
            # 新增：后退阶段日志频率控制（1秒1次）
            if not hasattr(self, 'last_backward_log_time'):
                self.last_backward_log_time = 0.0

            # ========== 阶段1：调整进仓角度（核心：目标航向一致+稳定判定）==========
            if self.loading_phase == "LOADING_TURN":                
                # 修正1：传实际进仓目标航向，和判定目标一致，误差才能归0
                correction = self.get_speed_correction(self.loading_turn_target_deg)
                yaw_diff = self.get_heading_error(self.loading_turn_target_deg)
                turn_speed = self.get_adaptive_turn_speed(yaw_diff)
                left_speed = turn_speed + correction
                right_speed = turn_speed + correction
                self.set_motors_speed(left_speed, right_speed)
                
                # 计算归一化后的角度差
                self.get_logger().info(
                    f"[LOADING] 角度调整阶段 - 当前航向{self.imu_yaw_deg:.2f}deg，"
                    f"目标{self.loading_turn_target_deg:.2f}deg，差值{yaw_diff:.2f}deg"
                )
                
                # 修正2：稳定判定：连续3次误差<阈值，才判定完成（避免IMU抖动）
                if abs(yaw_diff) < self.yaw_diff_min:
                    self.yaw_stable_count += 1
                    self.get_logger().debug(f"[LOADING] 角度误差达标，稳定计数={self.yaw_stable_count}/3")
                    if self.yaw_stable_count >= 3:
                        self.get_logger().info("[LOADING] 角度调整稳定完成（连续3次达标），进入后退进仓阶段")
                        self.loading_phase = "LOADING_BACKWARD"
                        self.loading_backward_start_time = current_time
                        self.yaw_stable_count = 0  # 重置计数器
                else:
                    self.yaw_stable_count = 0  # 误差不达标，计数器清零
                
                # 超时逻辑保留
                if current_time - self.loading_start_time > self.loading_turn_time:
                    self.get_logger().warn("[LOADING] 角度调整超时，强制进入后退阶段")
                    self.loading_phase = "LOADING_BACKWARD"
                    self.loading_backward_start_time = current_time
                    self.yaw_stable_count = 0
            
            # ========== 阶段2：后退进仓（核心：停止修正+低频日志）==========
            elif self.loading_phase == "LOADING_BACKWARD":
                correction = 0.0  # 后退阶段彻底停止航向修正，解决频率混乱
                if current_time - self.loading_backward_start_time < self.loading_backward_threshold:
                    left_speed = self.motor_ctrl.BASE_SPEED + correction
                    right_speed = -(self.motor_ctrl.BASE_SPEED + correction)
                    self.set_motors_speed(left_speed, right_speed)
                    # 修正3：后退日志1秒1次，避免高频输出
                    if current_time - self.last_backward_log_time >= 1.0:
                        self.get_logger().info(f"[LOADING] 后退进仓阶段 - 已持续{current_time - self.loading_backward_start_time:.1f}秒")
                        self.last_backward_log_time = current_time
                else:
                    self.get_logger().info("[LOADING] 后退进仓完成，进入完成阶段")
                    self.loading_phase = "COMPLETE"
                    self.last_backward_log_time = 0.0  # 重置日志时间
            
            # ========== 阶段3：进仓完成 ==========
            elif self.loading_phase == "COMPLETE":
                self.get_logger().info("[LOADING] 进仓流程完成，电机停止")
                self.set_motors_speed(0, 0)
                if self.loading_timer is not None:
                    self.loading_timer.cancel()
                    self.loading_timer = None
                self.nav_status = "IDLE"
                self.is_in_bin_process = False
                self.yaw_stable_count = 0  # 重置计数器
                self.last_backward_log_time = 0.0  # 重置日志时间
        except Exception as e:
            self.get_logger().error(f"[LOADING] 执行异常：{str(e)}")
            self.is_in_bin_process = False
            self.yaw_stable_count = 0

    def set_motors_speed(self, left_speed: float, right_speed: float) -> None:
        """设置双电机速度（完全保留原有功能）"""
        # 左电机（ID=1）
        self.motor_ctrl.motor_set_speed(self.motor_ctrl.motors[0]["id"], left_speed)
        # 右电机（ID=2）
        self.motor_ctrl.motor_set_speed(self.motor_ctrl.motors[1]["id"], right_speed)

        # 构造并发布速度消息
        wheel_speed_msg = Vector3()
        wheel_speed_msg.x = float(left_speed)    # 左轮角速度
        wheel_speed_msg.y = float(right_speed)   # 右轮角速度
        wheel_speed_msg.z = 0.0           # brush speed
        self.speed_pub.publish(wheel_speed_msg)
        
    def set_brush_speed(self, brush_speed: float) -> None:
        """设置 刷 电机速度"""
        # 刷盘电机（ID=3）
        self.motor_ctrl.motor_set_speed(self.motor_ctrl.motors[2]["id"], brush_speed)

# -------------------------- 主函数入口 --------------------------
def main(args=None):
    rclpy.init(args=args)

    # 创建电机控制节点
    motor_node = MotorControlNode()

    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        motor_node.get_logger().info("[ROSNode] 收到中断信号，即将退出")
    except Exception as e:
        motor_node.get_logger().fatal(
            f"[ROSNode] 节点运行致命错误：{str(e)}\n{traceback.format_exc()}"
        )
    finally:
        # 退出时停止所有电机（原有清理逻辑）
        if motor_node:
            motor_node.get_logger().info("[ROSNode] 退出时停止所有电机...")
            for motor in motor_node.motor_ctrl.motors:
                try:
                    motor_node.motor_ctrl.motor_set_speed(motor["id"], 0.0)
                    time.sleep(0.1)
                    motor_node.motor_ctrl.motor_disable(motor["id"])
                except Exception as e:
                    motor_node.get_logger().warn(f"[ROSNode] 停止电机{motor['id']}失败：{str(e)}")
                time.sleep(0.001)
            if motor_node.motor_ctrl.bus and motor_node.motor_ctrl.bus is not None:
                motor_node.get_logger().info("[ROSNode] 关闭CAN串口连接...")
                try:
                    motor_node.motor_ctrl.bus.shutdown()
                except Exception as e:
                    motor_node.get_logger().warn(f"[ROSNode] 关闭CAN串口失败：{str(e)}")
            try:
                motor_node.sbus_remote.stop()
            except Exception as e:
                motor_node.get_logger().warn(f"[ROSNode] 关闭SBUS遥控器失败：{str(e)}")
        motor_node.destroy_node()
        rclpy.shutdown()
        print("[ROSNode] 电机控制节点退出完成")

if __name__ == "__main__":
    main()