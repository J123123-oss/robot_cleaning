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
from std_msgs.msg import String, UInt8, Float32, Float32MultiArray, UInt16MultiArray, Int16
from geometry_msgs.msg import Vector3
import traceback
import json
import subprocess
import threading
from enum import Enum
from sensor_msgs.msg import NavSatFix
import collections  # 用于创建固定长度的双端队列
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from custom_msgs.srv import ChargeControl  # 导入自定义充电控制服务类型  

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 注：保留你的原有导入，此处省略（实际使用时直接替换原有代码即可）
from motor_control.motor_driver import CanMotorDriver
from motor_control.remote_control import SBUSRemoteController
from motor_control.charging import Charging485Node

# -------------------------- 全局配置与枚举 --------------------------
class RobotStateKey(Enum):
    HOLD = "h"
    START = "x"
    FORWARD = "w"
    BACKWARD = "s"
    LEFT = "a"
    RIGHT = "d"
    LOADING = "l"
    UNLOADING = "u"
    AUTO_CLEANING = "r"
    DISABLE = "z"  # 新增：完全失能状态（区别于HOLD的停止但保持使能）


STATE_DICT = {e.value: e.name for e in RobotStateKey}  # {'h':'HOLD', 'x':'START'...}

MAX_SPEED = 12.0   # 遥控器最大速度
MIN_SPEED = -12.0  # 遥控器最小速度
BRUSH_SPEED = -18.0
CH2_SENSITIVITY = 1.0  # 前进后退灵敏度
CH3_SENSITIVITY = 0.5  # 左右旋转灵敏度
DEAD_ZONE = 0.08       # 控制死区
RC_CH_MAX_VALUE = 1722

# 新增：定义RTK Fixed最大等待时间（可根据实际需求调整，如30s）
MAX_GPS_WAIT_TIME = 30.0

#PID参数
MAX_CORRECTION = 0.8

# -------------------------- 电机控制节点（独立ROS2节点） --------------------------
class MotorControlNode(Node):
    def __init__(self, node_name='motor_control_node'):
        super().__init__(node_name)

        # 声明RTK路径参数，用于获取route_id
        self.declare_parameter("rtk_path_file", "/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/test_bridge6-11.txt")
        rtk_path_file = self.get_parameter("rtk_path_file").value
        self.get_logger().info(f"[ROSNode] 获取到rtk_path_file参数: {rtk_path_file}")
        # 从路径中提取文件名作为route_id
        if rtk_path_file:
            self.route_id = os.path.splitext(os.path.basename(rtk_path_file))[0]
        else:
            self.route_id = "default"
        self.get_logger().info(f"[ROSNode] 初始化route_id: {self.route_id}")

        # 循环频率：10Hz（兼容原有逻辑，可调整）
        self.rate = self.create_rate(20)
        self.last_yaw_error = 0.0  # 上一次的航向误差
        # MQTT点按方向定时器
        self.direction_timer = None
        self.brush_speed = 0.0  # 滚刷速度
        self.current_left_speed = 0.0  # 当前左轮速度
        self.current_right_speed = 0.0  # 当前右轮速度
        self.mqtt_control_speed = 10.0  # MQTT控制速度
        # UNLOADING parameters
        self.unloading_forword_threshold = 22.0 # seconds
        self.unloading_turn_start_time = None
        self.unloading_turn_time_max = 30.0
        self.unloading_phase = None  # None/"FORWARD"/"UNLOADING_TURN"/"COMPLETE"
        self.unloading_start_time = 0.0  # 出仓开始时间
        self.unloading_turn_target_deg = 0.0  # 出仓转向目标角
        self.unloading_timer: Optional[Timer] = None  # 出仓专用定时器

        self.loading_turn_target_deg = 89.04    # 进仓转向目标角
        self.loading_backward_threshold = 10.0  # 进仓后退时长（秒）
        self.loading_turn_time = 30.0
        self.loading_phase = None
        self.loading_start_time = 0.0
        self.loading_timer: Optional[Timer] = None
        self.loading_backward_start_time = 0.0
        
        self.state_publish_timer: Optional[Timer] = None  # 定时发布状态

        self.correction_state = "IDLE"  # 状态：IDLE/DEFLECT/RETRACT/CHECK
        self.correction_count = 0
        self.in_full_correction = False       # 流程锁：是否正在执行完整调整流程

        self.correction_start_time = 0.0  # 单次调整开始时间
        self.correction_duration = 1.0  # 单次小幅旋转调整时长（秒，可根据实际调）
        self.retract_duration = 2.0     # 后退固定时长（2秒）
        self.last_state = None  # 上一次的状态（用于偏转回正逻辑）
        self.check_max_timeout = 3.0          # 反向检查最大超时（秒，避免无限循环）

        # self.loading_adjust_phase = ""  # 大幅差值调整阶段：DEFLECT/RETREAT/CHECK
        # self.last_deflect_phase = ""  # 新增：记录上次的偏转方向（LEFT_DEFLECT/RIGHT_DEFLECT）
        # self.loading_adjust_start = 0.0  # 调整阶段开始时间
        # self.loading_stable_count = 0    # 对正稳定计数（需连续3次达标）
        # self.loading_max_stable = 3      # 稳定达标次数

        self.yaw_diff_min = 0.3 # 0.1 degree
        # 新增：IMU角度更新时间戳，用于过滤旧数据
        self.last_imu_update_time = 0.0
        self.imu_update_interval = 0.2  # 要求IMU至少100ms更新一次（适配常见IMU发布频率）

        self.nav_status = None
        self.complete_state = False
        self.is_charging = None # 新增：电池是否正在充电
        self.rc_control = False # 新增：是否遥控器控制（优先级最高）
        self.rc_previous_status = None  # 新增：遥控器接管时上一次显示状态

        self.battery_total_voltage = None  # 电池总电压
        self.battery_current = None  # 电池电流
        self.battery_remaining = None # 电池百分比
        self.battery_temperatures = [] # 电池温度，共3个
        self.sensors_status = 0b000000  # 6个传感器状态位（初始全无障碍）

        self.dock_sensors = 0  #停机仓传感器状态
        self.dock_last_sensors = 0
        self.charging_v = 0.0 # 停机仓充电电压
        self.charging_i = 0.0 # 停机仓充电电流
        self.charging_fault = 0 # 停机仓充电故障代码
        self.motor_fault_codes = [0, 0, 0]  # 三路电机故障码
        self.last_charging_fault = 0  # 上一次的充电故障代码（用于检测故障码变化）
        # self.battery_full_charge = False
        self.charge_resume_threshold = 90   #old:98 # 恢复充电的电量阈值（百分比）
        self.is_charge_paused = True  # 充电是否已暂停（充满后暂停）
        self.last_charge_stop_time = 0.0  # 上次停止充电的时间戳
        self.charge_resume_count = 0  # 恢复充电的次数
        self.last_charge_resume_time = 0.0  # 上次尝试恢复充电的时间戳

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

        # laser distance
        # 激光数据滤波配置
        self.laser_filter_window = 5  # 滤波窗口大小（取最近5次数据平均，可调整）
        self.laser_left_buffer = collections.deque(maxlen=self.laser_filter_window)  # 左激光缓存
        self.laser_right_buffer = collections.deque(maxlen=self.laser_filter_window)  # 右激光缓存
        self.laser_valid_min = 0  # 激光数据最小值（根据硬件调整）
        self.laser_valid_max = 5000  # 激光数据最大值（根据硬件调整）
        self.laser_distance = [0, 0]  # 滤波后的最终激光距离

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
        self.laser_subscription = self.create_subscription(
            UInt16MultiArray,
            "laser_distance",
            self.laser_callback,
            10
        )
        self.charging_subscription = self.create_subscription(
            Float32MultiArray,
            "charging_volt_curr",
            self.charging_volt_curr_cb,
            10
        )
        self.charging_fault_subscription = self.create_subscription(
            Int16,
            "charging_fault_code",
            self.charging_fault_code_cb,
            10
        )
        self.motor_fault_subscription = self.create_subscription(
            Float32MultiArray,
            "motor_fault_codes",
            self.motor_fault_callback,
            10
        )
        # 4. ROS2 发布器
        self.state_pub = self.create_publisher(String, "/motor/state", 10)  # 电机状态
        self.speed_pub = self.create_publisher(Vector3, "/motor/current_speed", 10)  # 电机当前速度
        self.unloading_gps_pub = self.create_publisher(Vector3, "/unloading_gps", 10)  # 出仓完成时GPS坐标
        self.mode_pub = self.create_publisher(String, "/control/mode", 10)  # 当前控制模式
        self.route_change_pub = self.create_publisher(String, "/rtk/route_change", 10)  # 路径切换话题
        self.robot_state_pub = self.create_publisher(String, "/robot_state", 10)  # robot mqtt msg
        self.dock_state_pub = self.create_publisher(String, "/dock_state", 10)  # dock mqtt msg
        self.rc_channels_pub = self.create_publisher(Float32MultiArray, "/rc_channels", 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/car_center_gps', self.gps_callback, 10)

        # 全局变量
        self.current_control_mode = "NORMAL"  # 默认普通模式
        self.rtk_left_speed = 0.0  # 存储RTK订阅的左轮速度
        self.rtk_right_speed = 0.0 # 存储RTK订阅的右轮速度

        self.status_list = [
            "HOLD", "START", "FORWARD", "BACKWARD", "LOADING", "UNLOADING",
            "LEFT", "RIGHT", "PAUSE", "AUTO_CLEANING","DISABLE","RC_ENABLE"
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
        self.charge_resume_timer = self.create_timer(30.0, self.charge_resume_callback)  # 30秒检查一次恢复充电
        self.state_publish_timer = self.create_timer(5.0, self.publish_state)  # 默认5秒发布状态

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
        # 由于我们现在让 MQTT-ROS 桥把 MQTT 的 robot/ID/status 主题直接发布为 ROS 话题
        # /mqtt_status，dock_state_callback 应订阅那个新话题而不是 robot_cmd 或 robot_state。
        # 这样就可以避免收到节点自身发布的 robot_state，并能正确解析外部MQTT状态中的
        # sensors 字段。
        self.mqtt_status_subscription = self.create_subscription(
            String,
            "mqtt_status",
            self.dock_state_callback,
            10
        )

        # 1. 创建服务客户端（与charging.py中的服务名对应）
        # 充电控制服务客户端（自定义服务类型）
        self.cli_start_charge = self.create_client(ChargeControl, 'start_charging')
        self.cli_stop_charge = self.create_client(ChargeControl, 'stop_charging')
        
        # 数据查询服务客户端（标准Trigger服务类型）
        self.cli_query_volt_curr = self.create_client(Trigger, 'query_volt_curr')
        self.cli_query_fault = self.create_client(Trigger, 'query_fault_code')
        
        # 等待服务端上线（超时5秒）
        # self.wait_for_services()
        
        self.get_logger().info("电机控制节点启动成功，已连接充电服务")

    def wait_for_services(self):
        """等待所有充电服务上线"""
        services = [
            ('start_charging', self.cli_start_charge),
            ('stop_charging', self.cli_stop_charge),
            ('query_volt_curr', self.cli_query_volt_curr),
            ('query_fault_code', self.cli_query_fault)
        ]
        
        for srv_name, cli in services:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"等待服务 {srv_name} 超时，继续运行（服务可能未启动）")
                # 移除 rclpy.shutdown() 以避免上下文无效
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
    def charge_resume_callback(self):
        # 充电停止条件判断（使用存储的传感器状态）
        if self.is_charging and ((self.dock_sensors & 0x08) or (self.dock_sensors & 0x04)):
            # 条件1：电压电流条件
            if self.charging_v >= 54.0 and 1.0 <= self.charging_i <= 1.6:
                self.get_logger().info("[ROSNode] 充电完成（电压电流条件）")
                self.stop_charge_async()
                self.is_charge_paused = True
                self.last_charge_stop_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f"[ROSNode] 充电已暂停，等待电量低于{self.charge_resume_threshold}%时恢复充电")
            # 条件2：故障码从非9变为9（满充保护）
            elif self.charging_fault == 9 and self.last_charging_fault != 9:
                self.get_logger().info("[ROSNode] 充电完成（故障码9-满充保护）")
                # self.stop_charge_async()
                self.is_charging = False  # 更新充电状态
                self.is_charge_paused = True
                self.last_charge_stop_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f"[ROSNode] 充电已暂停，等待电量低于{self.charge_resume_threshold}%时恢复充电")
        
        if self.is_charge_paused and self.battery_remaining is not None:
            if self.battery_remaining <= self.charge_resume_threshold:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self.last_charge_stop_time >= 60.0:
                    if (self.dock_sensors & 0x08) or (self.dock_sensors & 0x04):
                        if not self.is_charging:
                            if current_time - self.last_charge_resume_time >= 360.0:
                                self.get_logger().info(f"[ROSNode] 电量{self.battery_remaining}%低于阈值{self.charge_resume_threshold}%，车体到位，恢复充电（第{self.charge_resume_count + 1}次）")
                                self.start_charge_async()
                                self.last_charge_resume_time = current_time
                        else:
                            self.get_logger().info("[ROSNode] 充电已在进行中，清除充电暂停标志")
                            self.is_charge_paused = False
                    else:
                        self.get_logger().info(f"[ROSNode] 车体已离开停机仓，清除充电暂停标志，累计恢复充电{self.charge_resume_count}次")
                        self.is_charge_paused = False
                        self.charge_resume_count = 0
    def timer_callback(self):
        # 检查CAN串口是否正常打开
        if not self.motor_ctrl.bus:
            self.get_logger().warn("[ROSNode] CAN串口连接断开，尝试重连...")
            self.motor_ctrl.reconnect_can_bus()
        self.current_control_mode = self.sbus_remote.control_mode
        if self.rc_control:
            self.current_control_mode = "REMOTE"
        elif self.rc_control == False and self.current_control_mode not in ["AUTO_CLEANING", "REMOTE"]:
            self.current_control_mode = "NORMAL"
            # self.get_logger().info(f"{self.current_control_mode}")
        
        # 1. 发布当前控制模式（给RTK节点）
        mode_msg = String()
        mode_msg.data = self.current_control_mode if isinstance(self.current_control_mode, str) else "NORMAL"
        self.mode_pub.publish(mode_msg)
        self.publish_rc_channels()

        # 进出仓流程的暂停/恢复逻辑：
        # - 当正在进/出仓（is_in_bin_process=True）且当前控制模式不等于流程发起时，暂停流程；
        # - 当控制模式恢复到流程发起模式时，恢复流程；
        # 暂停时不取消定时器，仅停止电机并在 handler 中短路，从而可在回到原模式后继续。
        if self.is_in_bin_process:
            # 如果尚未记录来源模式，则以当前模式作为来源
            if self.bin_process_origin_mode is None and self.current_control_mode != "REMOTE":
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
                ch6_norm =1.0 if ch6_norm > 0.5 else 0.0  # 阈值判断
                brush_speed = ch6_norm * BRUSH_SPEED
                # self.get_logger().info(f"[ROSNode] 刷: {brush_speed:.2f}")
                self.set_brush_speed(brush_speed)
            except Exception as e:
                self.get_logger().warn(f"[ROSNode] 获取遥控器速度失败：{e}")
                self.set_motors_speed(0.0, 0.0)
                self.set_brush_speed(0.0)

        elif self.current_control_mode == "NORMAL":
            # 按当前状态赋值速度
            if self.current_status == "FORWARD":
                left_speed = -self.mqtt_control_speed
                right_speed = self.mqtt_control_speed
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status == "BACKWARD":
                left_speed = self.mqtt_control_speed
                right_speed = -self.mqtt_control_speed
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status == "LEFT":
                left_speed = self.mqtt_control_speed
                right_speed = self.mqtt_control_speed
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status == "RIGHT":
                left_speed = -self.mqtt_control_speed
                right_speed = -self.mqtt_control_speed
                self.set_motors_speed(left_speed, right_speed)
            elif self.current_status in ["HOLD"]:
                left_speed = 0.0
                right_speed = 0.0
                self.set_motors_speed(left_speed, right_speed)
            self.set_brush_speed(0.0)
            # self.set_motors_speed(left_speed, right_speed)
            # stop brush

        elif self.current_control_mode == "AUTO_CLEANING":
            left_speed = self.rtk_left_speed 
            right_speed = self.rtk_right_speed 
            self.set_motors_speed(left_speed, right_speed)
            self.get_logger().debug(f"[RTKControl] 左轮：{left_speed:.2f}，右轮：{right_speed:.2f}")
            # start brush
            # self.set_brush_speed(BRUSH_SPEED)
        state_msg = String()
        state_msg.data = str(self.current_status)
        self.state_pub.publish(state_msg)

    def keyboard_callback(self, msg: String) -> None:
        """键盘控制回调（新增RTK模式切换）"""
        key = msg.data.strip().lower()

        # 模式切换指令
        if key == 'r':
            # 切换到RTK导航模式（进出仓状态下禁止切换）
            if not self.is_in_bin_process:
                self.current_control_mode = "AUTO_CLEANING"
                self.get_logger().info(f"[ROSNode] 控制模式切换：→ AUTO_CLEANING")
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
            if self.current_control_mode != "AUTO_CLEANING":
                self.switch_state(key)
            else:
                self.get_logger().warn("[ROSNode] 当前为RTK导航模式，忽略键盘状态指令")
        else:
            self.get_logger().warn(f"[ROSNode] 无效键盘指令：{key}，支持指令：{list(STATE_DICT.keys()) + ['r(RTK)', 'n(普通)', 'm(遥控)']}")

    def rtk_speed_callback(self, msg: Vector3):
        """订阅RTK节点的速度指令，更新本地速度变量"""
        self.rtk_left_speed = msg.x
        self.rtk_right_speed = msg.y
        if self.current_control_mode == "AUTO_CLEANING":
            self.set_brush_speed(float(msg.z))
        self.get_logger().debug(f"[RTKSpeed] 左轮：{self.rtk_left_speed:.2f}，右轮：{self.rtk_right_speed:.2f}，刷：{msg.z:.2f}")
    
    def rtk_nav_status_callback(self, msg: String):
        """订阅RTK导航状态消息（备用）"""
        self.nav_status = msg.data.strip()
        # self.get_logger().info(f"[RTKNavStatus] 当前导航状态：{self.nav_status}")
        if self.nav_status == "COMPLETED" and not self.is_in_bin_process:
            self.get_logger().info(f"[RTKNavStatus] RTK导航完成，切换到LOADING状态")
            self.current_control_mode = "NORMAL"  # 切换回普通模式
            self.switch_state('l')

    def handle_route_change(self, route_id: str):
        """处理路径切换指令"""
        route_file = f"/home/ztl/robot_cleaning/src/rtk_nav/rtk_nav/cleaning_path/{route_id}.txt"
        
        if not os.path.exists(route_file):
            self.get_logger().error(f"[ROSNode] 路径文件不存在: {route_file}")
            return
        if self.current_status != "DISABLE":
            self.get_logger().warn(f"[ROSNode] 当前状态不是{self.current_status}，忽略路径切换指令")
            return
        route_msg = String()
        route_msg.data = route_file
        self.route_id = route_id
        self.route_change_pub.publish(route_msg)
        self.get_logger().info(f"[ROSNode] 已发布路径切换指令: {route_id} -> {route_file}")

    def io_data_callback(self, msg: UInt8):
        """处理IO数据回调（可根据需要扩展功能）"""
        if self.current_control_mode != "AUTO_CLEANING" and not self.is_in_bin_process:
            self.front_left = (msg.data & 0x01) == 0x01
            self.front_right = (msg.data & 0x02) == 0x02
            self.mid_left = (msg.data & 0x04) == 0x04
            self.mid_right = (msg.data & 0x08) == 0x08
            self.back_left = (msg.data & 0x10) == 0x10
            self.back_right = (msg.data & 0x20) == 0x20
            # 按位或结果存储传感器状态
            self.sensors_status = self.front_left | self.front_right<<1 | self.mid_left<<2 | self.mid_right<<3 | self.back_left<<4 | self.back_right<<5 
            self.sensors_status = ~self.sensors_status & 0x3F  # 取反并保留6位

            # self.front_left = (msg.data & 0x01) == 0x00 
            # self.front_right = (msg.data & 0x02) == 0x00
            # self.mid_left = (msg.data & 0x04) == 0x00
            # self.mid_right = (msg.data & 0x08) == 0x00
            # self.back_left = (msg.data & 0x10) == 0x00
            # self.back_right = (msg.data & 0x20) == 0x00
            # self.sensors_status = self.front_left | self.front_right<<1 | self.mid_left<<2 | self.mid_right<<3 | self.back_left<<4 | self.back_right<<5 
            # self.sensors_status = ~self.sensors_status & 0x3F  # 取反并保留6位

            # self.get_logger().info(f"[IOData] 传感器状态：{self.sensors_status:06b}")

    def voltage_to_soc(self, voltage):
        """
        13串三元锂电池 电压 → 电量百分比（非线性插值）
        :param voltage: 电池总电压 V
        :return: soc 电量百分比 0~100
        """
        # 电压-SOC 对应表（升序排列）
        volt_table = [37.7, 41.6, 43.5, 45.3, 47.1, 48.3, 49.2, 50.1, 50.9, 51.7, 52.5, 53.5, 54.6]
        soc_table =  [  0,    0,    5,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100]

        # 越界处理
        if voltage >= 54.6:
            return 100
        if voltage <= 37.7:
            return 0

        # 非线性插值（找到区间，线性估算）
        for i in range(len(volt_table)-1):
            v_low = volt_table[i]
            v_high = volt_table[i+1]
            s_low = soc_table[i]
            s_high = soc_table[i+1]

            if v_low <= voltage <= v_high:
                # 区间内插值计算
                soc = s_low + (voltage - v_low) * (s_high - s_low) / (v_high - v_low)
                return round(soc, 1)

        return 0
    def battery_callback(self, msg):
        """订阅电池数据的回调函数（修正版）"""
        if len(msg.data) < 3:
            self.get_logger().warn("警告：订阅到的电池数据不完整，跳过解析")
            return
        
        self.battery_remaining = msg.data[0]  # 电池百分比
        self.battery_current = round(msg.data[1], 2)  # 总电流（索引1）
        self.battery_total_voltage = round(msg.data[2], 2)  # 总电压（索引2）
        self.battery_temperatures = round(msg.data[3], 1) # 温度（索引3）
    def charging_volt_curr_cb(self, msg: Float32MultiArray):
        """订阅充电电压电流数据的回调函数"""
        if len(msg.data) < 2:
            self.get_logger().warn("警告：订阅到的充电数据不完整，跳过解析")
            return
        
        self.charging_v = round(msg.data[0], 2) if msg.data[0] else 0.0  # 充电电压（索引0）
        self.charging_i = round(msg.data[1], 2) if msg.data[1] else 0.0  # 充电电流（索引1）
        # self.battery_remaining = self.voltage_to_soc(self.charging_v) if self.charging_v > 0 else 0.0  # 电池百分比估计

    def charging_fault_code_cb(self, msg: Int16):
        """订阅充电故障代码的回调函数"""
        self.charging_fault = msg.data  # 故障代码（整数）
        # 补充中途故障后切换状态：
        if self.charging_fault != 0x00 and self.charging_fault != self.last_charging_fault:
            self.get_logger().info(f"故障码更新为{self.charging_fault}，设置充电暂停")
            self.is_charging = False
            self.is_charge_paused = True
            self.dock_last_sensors = 0
        self.last_charging_fault = self.charging_fault  # 保存上一次的故障码

    def motor_fault_callback(self, msg: Float32MultiArray):
        """订阅电机故障码数组的回调函数"""
        if len(msg.data) < 3:
            self.get_logger().warn("电机故障码数据不完整!")
            return

        self.motor_fault_codes = [int(msg.data[0]), int(msg.data[1]), int(msg.data[2])]
        
        fault_codes = self.motor_fault_codes
        has_fault = False
        fault_str = ""
        for i, code in enumerate(fault_codes):
            if code != 0:
                has_fault = True
                motor_name = ["左轮", "右轮", "前毛刷"][i]
                fault_str += f"{motor_name}:0x{code:02X}; "
        
        if has_fault:
            self.get_logger().warn(f"[MotorControl] 电机故障: {fault_str}")
            self.motor_ctrl.motor_clear_fault(1)
            self.motor_ctrl.motor_clear_fault(2)
            self.motor_ctrl.motor_clear_fault(3)
            self.get_logger().warn(f"发送清除故障指令！")

        # else:
        #     self.get_logger().debug("[MotorControl] 电机状态正常")
    
    # def laser_callback(self, msg: UInt16MultiArray):
    #     """订阅激光距离数据的回调函数"""
    #     if len(msg.data) < 2:
    #         self.get_logger().warn("激光距离数据不完整!")
    #         return
        
    #     self.laser_distance = [msg.data[0], msg.data[1]]  # 两路激光距离
    def laser_callback(self, msg: UInt16MultiArray):
        """订阅激光距离数据的回调函数，添加滑动平均滤波避免数据突变"""
        # 1. 基础数据校验
        if len(msg.data) < 2:
            self.get_logger().warn("激光距离数据不完整!")
            return
        
        # 2. 原始数据提取与异常值过滤
        raw_left = msg.data[0]
        raw_right = msg.data[1]
        
        # 过滤明显异常的突变值（超出合理范围则丢弃）
        if not (self.laser_valid_min <= raw_left <= self.laser_valid_max):
            # self.get_logger().warn(f"左激光数据异常：{raw_left}mm，丢弃该值")
            raw_left = self.laser_distance[0]  # 用上次滤波后的值替代
        if not (self.laser_valid_min <= raw_right <= self.laser_valid_max):
            # self.get_logger().warn(f"右激光数据异常：{raw_right}mm，丢弃该值")
            raw_right = self.laser_distance[1]  # 用上次滤波后的值替代
        
        # 3. 将有效数据加入滤波缓存
        self.laser_left_buffer.append(raw_left)
        self.laser_right_buffer.append(raw_right)
        
        # 4. 计算滑动平均值（缓存未满时取现有数据的平均）
        filtered_left = sum(self.laser_left_buffer) / len(self.laser_left_buffer)
        filtered_right = sum(self.laser_right_buffer) / len(self.laser_right_buffer)
        
        # 5. 保留整数（激光数据为整数，可选）
        # self.laser_distance = [int(filtered_left), int(filtered_right)]
        self.laser_distance = [int(raw_left), int(raw_right)]
        
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
        if yaw_error_abs > 30:
            kp = 0.05  # 大误差：快速转向
        elif yaw_error_abs > 10: #20:
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
        # if self.current_control_mode == "REMOTE":
        #     self.get_logger().warn("[ROSNode] 当前为遥控器模式，禁止设置状态")
        #     return
        # AUTO_CLEANING模式下允许重复设置HOLD状态，确保能正确切换控制模式
        if new_state == self.current_status:
            if new_state == "HOLD" and self.current_control_mode == "AUTO_CLEANING":
                self.get_logger().info(f"[ROSNode] AUTO_CLEANING模式下重复设置HOLD，允许执行")
            else:
                self.get_logger().info(f"[ROSNode] 已处于{new_state}状态，无需切换")
                return
        
        # 核心修改：正在进出仓时，仅响应HOLD指令，其余指令直接忽略并打印日志
        if self.is_in_bin_process:
            if new_state not in ["HOLD", "DISABLE"]:
                self.get_logger().warn(f"[ROSNode] 正在执行进\出仓流程，仅支持HOLD\DISABLE指令，忽略状态切换（{self.current_status}→{new_state}）")
                return
            # 若是HOLD指令，正常执行，后续会重置进出仓标记

        self.get_logger().info(f"[ROSNode] 状态切换：{self.current_status} → {new_state}")
        self.current_status = new_state

        # 核心修复2：设置进出仓状态标记
        if new_state in ["UNLOADING", "LOADING"]:
            # 记录流程开始时的控制模式，作为暂停/恢复判断的来源模式
            self.bin_process_origin_mode = self.current_control_mode
            self.bin_process_paused = False
            self.is_in_bin_process = True
        elif new_state == "HOLD" and self.is_in_bin_process:
            self.is_in_bin_process = False
            self.bin_process_origin_mode = None
            self.bin_process_paused = False
            # 额外：终止进出仓定时器，防止定时器继续执行逻辑
            if self.loading_timer is not None:
                self.loading_timer.cancel()
                # self.loading_timer = None
            if self.unloading_timer is not None:
                self.unloading_timer.cancel()
                # self.unloading_timer = None
            self.get_logger().info("[ROSNode] 进/出仓流程被HOLD指令强制终止，定时器已关闭")

        # 状态执行逻辑
        if new_state == "DISABLE":
            # 停止：失能所有电机
            for motor in self.motor_ctrl.motors:
                self.motor_ctrl.motor_set_speed(motor["id"], 0.0)
                self.motor_ctrl.motor_disable(motor["id"])
                time.sleep(0.01)
            # 降低状态发布频率至600秒
            if self.state_publish_timer is not None:
                self.state_publish_timer.cancel()
                self.state_publish_timer = self.create_timer(3000.0, self.publish_state)
                self.get_logger().info("[ROSNode] 进入DISABLE状态，状态发布频率改为3000秒")
                # 发布最后一条消息
                self.publish_state()
        elif new_state == "HOLD":
            # HOLD状态时切换到NORMAL模式，以便RTK节点能正确响应暂停
            if self.current_control_mode == "AUTO_CLEANING":
                self.current_control_mode = "NORMAL"
                self.get_logger().info("[ROSNode] HOLD状态：切换控制模式 AUTO_CLEANING -> NORMAL")
            
            # 停止：失能所有电机
            for motor in self.motor_ctrl.motors:
                self.motor_ctrl.motor_set_speed(motor["id"], 0.0)
                self.set_brush_speed(0.0)
                # self.motor_ctrl.motor_disable(motor["id"])
                time.sleep(0.01)
            # # 降低状态发布频率至600秒
            # if self.state_publish_timer is not None:
            #     self.state_publish_timer.cancel()
            #     self.state_publish_timer = self.create_timer(5.0, self.publish_state)
            #     self.get_logger().info("[ROSNode] 进入HOLD状态，状态发布频率改为60秒")

        elif new_state == "START":
            
            self.motor_ctrl.initialize_motors()
            time.sleep(0.001)
            self.complete_state = False
            # 恢复状态发布频率至1秒
            if self.state_publish_timer is not None:
                self.state_publish_timer.cancel()
                self.state_publish_timer = self.create_timer(5.0, self.publish_state)
                self.get_logger().info("[ROSNode] 进入START状态，状态发布频率恢复为5秒")

        elif new_state == "FORWARD":
            # 前进：双电机正转
            left_speed = -self.mqtt_control_speed
            right_speed = self.mqtt_control_speed
            self.set_motors_speed(left_speed, right_speed)
            # 点按前进：1秒后自动停止
            if self.direction_timer:
                self.direction_timer.cancel()
            self.direction_timer = self.create_timer(10.0, lambda: self.auto_stop("w"))

        elif new_state == "BACKWARD":
            # 后退：双电机反转
            left_speed = self.mqtt_control_speed
            right_speed = -self.mqtt_control_speed
            self.set_motors_speed(left_speed, right_speed)
            # 点按前进：1秒后自动停止
            if self.direction_timer:
                self.direction_timer.cancel()
            self.direction_timer = self.create_timer(10.0, lambda: self.auto_stop("s"))
        elif new_state == "LEFT":
            # 左转
            left_speed = self.mqtt_control_speed
            right_speed = self.mqtt_control_speed
            self.set_motors_speed(left_speed, right_speed)
            # 点按前进：1秒后自动停止
            if self.direction_timer:
                self.direction_timer.cancel()
            self.direction_timer = self.create_timer(10.0, lambda: self.auto_stop("a"))
        elif new_state == "RIGHT":
            # 右转
            left_speed = -self.mqtt_control_speed
            right_speed = -self.mqtt_control_speed
            self.set_motors_speed(left_speed, right_speed)
            # 点按前进：1秒后自动停止
            if self.direction_timer:
                self.direction_timer.cancel()
            self.direction_timer = self.create_timer(10.0, lambda: self.auto_stop("d"))
        elif new_state == "UNLOADING":
            # 使能电机
            self.motor_ctrl.initialize_motors()
            time.sleep(0.001)
            self.complete_state = False
            # 恢复状态发布频率至1秒
            if self.state_publish_timer is not None:
                self.state_publish_timer.cancel()
                self.state_publish_timer = self.create_timer(5.0, self.publish_state)
                self.get_logger().info("[ROSNode] 进入START状态，状态发布频率恢复为5秒")

            self.complete_state = False
            self.current_status = new_state
            # 初始化出仓阶段（仅初始化，不启动定时器）
            self.unloading_phase = None  # 先置空，等待归位后再初始化
            self.unloading_start_time = None  # 暂不记录启动时间
            self.unloading_timer = None  # 定时器先置空
            self.is_in_bin_process = True  # 标记进入进出仓流程（防止其他操作）
            
            # self.get_logger().info("[ROSNode] 进入出仓状态，启动出仓定时器")
            # self.complete_state = False
            # self.current_status = new_state
            # # 初始化出仓阶段
            # self.unloading_phase = "UNLOADING_FORWARD"  # 第一阶段：前进
            # self.unloading_start_time = time.time()
            # # 修正：降低定时器频率到100ms，匹配IMU更新频率
            self.unloading_timer = self.create_timer(0.05, self.handle_unloading_step)
        elif new_state == "LOADING":
            self.current_status = new_state
            # 初始化进仓阶段（仅初始化，不启动定时器）
            self.loading_phase = None  # 先置空，等待归位后再初始化
            self.loading_start_time = None  # 暂不记录启动时间
            self.loading_timer = None  # 定时器先置空
            self.is_in_bin_process = True  # 标记进入进出仓流程（防止其他操作）
            
            # self.get_logger().info("[ROSNode] 进入进仓状态，启动进仓定时器")
            # self.current_status = new_state
            # self.loading_phase = "LOADING_TURN"  # 第一阶段：调整角度
            # self.loading_start_time = time.time()
            
            # 修正：目标角度归一化
            self.loading_turn_target_deg = (self.loading_turn_target_deg + 180) % 360 - 180
            # 修正：降低定时器频率到100ms，匹配IMU更新频率
            self.loading_timer = self.create_timer(0.05, self.handle_loading_step)
        elif new_state == "AUTO_CLEANING":
            self.current_control_mode = "AUTO_CLEANING"
            self.get_logger().info(f"{self.current_control_mode}")
            self.get_logger().info(f"[ROSNode] 切换到RTK导航模式，等待RTK速度指令")
            # 速度由RTK回调处理

        # 发布当前状态
        state_msg = String()
        state_msg.data = self.current_status
        self.state_pub.publish(state_msg)
    def auto_stop(self, key):
        """方向键点按后自动停止，并重置对应标记"""
        self.switch_state("h")
        # self.mqtt_click_trigger[key] = False
        self.get_logger().info(f"[MQTT_CLICK] {key}动作执行完成，自动停止并重置标记")
        if self.direction_timer:
            self.direction_timer.cancel()
            self.direction_timer = None

    def publish_state(self):
        """发布机器人状态消息（MQTT）"""
        try:
            state_msg = {
                "status": self.current_status,
                "nav_status": self.nav_status,# rtk导航状态
                "battery": self.battery_remaining,
                "battery_total_voltage": self.battery_total_voltage,
                "battery_current": self.battery_current,
                "battery_temperatures": self.battery_temperatures,
                "imu_yaw": self.imu_yaw_deg if self.imu_yaw_deg is not None else 0.00,
                "rtk_status": self.rtk_status,
                "current_lon": self.current_lon if self.current_lon > 0.1 else None,
                "current_lat": self.current_lat if self.current_lat > 0.1 else None,
                "route_id": self.route_id if self.route_id is not None else "default",
                # "uuid": "163e4ac9-18a9-4e08-9301-36ca08e07581",
                "acceleration": {
                    "x": self.current_left_speed,
                    "y": self.current_right_speed,
                    "z": self.brush_speed
                },
                "sensors_status":self.sensors_status,
                "laser_left": self.laser_distance[0],
                "laser_right": self.laser_distance[1],
                "complete_state": self.complete_state,
                "charging_v": self.charging_v,
                "charging_i": self.charging_i,
                "charging_fault": self.charging_fault,
                "motor_fault": self.motor_fault_codes,
                "dock_sensors": self.dock_sensors,
                "resume_charge": self.charge_resume_count,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            }
            dock_state_msg = {
                "status": self.current_status,
                "complete_state": self.complete_state,
                # "full_charge": self.battery_full_charge ,
                "t": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            }
            ros_string_msg = String()
            ros_string_msg.data = json.dumps(state_msg, ensure_ascii=False)
            self.robot_state_pub.publish(ros_string_msg)
            dock_ros_msg = String()
            dock_ros_msg.data = json.dumps(dock_state_msg, ensure_ascii=False)
            self.dock_state_pub.publish(dock_ros_msg)
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
            # self.dock_state_pub.publish(error_ros_msg)
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

        # AUTO_CLEANING模式下只能接收RC_ENABLE和HOLD指令
        is_rtk_nav_mode = self.current_control_mode == "AUTO_CLEANING"
        
        try:
            cmd_obj = json.loads(msg_data)
            
            # 修复问题1：先校验cmd_obj是否是字典类型，再查找command字段
            if not isinstance(cmd_obj, dict):
                self.get_logger().warn(f"[ROSNode] 消息不是JSON字典格式：{msg_data}，跳过字典解析")
                raise json.JSONDecodeError("Not a JSON dict", msg_data, 0)  # 主动抛错，进入下方字符串处理逻辑
            
            if "command" not in cmd_obj:
                # 检查是否是路径切换消息 {"route_id": "xxx"}
                if "route_id" in cmd_obj:
                    route_id = cmd_obj.get("route_id", "")
                    if route_id:
                        self.handle_route_change(route_id)
                    else:
                        self.get_logger().warn("[ROSNode] route_id字段为空")
                else:
                    self.get_logger().warn(f"[ROSNode] 未找到command字段: {msg_data}")
                return
            
            command = cmd_obj.get("command", "").strip()
            self.get_logger().info(f"[ROSNode] 解析到command：{command}")

            if command == "GET_STATUS":
                self.publish_state()
            elif command == "RC_ENABLE":
                # AUTO_CLEANING模式下允许RC_ENABLE
                if is_rtk_nav_mode:
                    self.get_logger().info("[ROSNode] AUTO_CLEANING模式下允许RC_ENABLE")
                # 保存当前控制模式，以便禁用时恢复
                if not self.rc_control:
                    self.previous_control_mode = self.current_control_mode
                    self.rc_previous_status = self.current_status
                    self.rc_control = True
                    self.current_status = command
                self.get_logger().info(f"[ROSNode] RC_ENABLE 已启用遥控器控制，保存之前模式: {self.rc_previous_status}")
                self.publish_state()
            elif command == "RC_DISABLE":
                # 取消手动控制计时器
                if self.direction_timer:
                    self.direction_timer.cancel()
                    self.direction_timer = None
                # AUTO_CLEANING模式下允许RC_DISABLE
                if is_rtk_nav_mode:
                    self.get_logger().info("[ROSNode] AUTO_CLEANING模式下允许RC_DISABLE")
                self.rc_control = False
                # 恢复之前的控制模式
                if hasattr(self, 'previous_control_mode') and self.previous_control_mode:
                    restore_mode = self.previous_control_mode
                    self.current_control_mode = restore_mode
                    self.current_status = self.rc_previous_status
                    self.get_logger().info(f"[ROSNode] RC_DISABLE 已禁用遥控器控制，恢复之前模式: {self.rc_previous_status}")
                else:
                    self.get_logger().info(f"[ROSNode] RC_DISABLE 已禁用遥控器控制")
                self.publish_state()
            elif command in STATE_DICT:
                # AUTO_CLEANING模式下只允许HOLD指令
                if is_rtk_nav_mode and command != "HOLD":
                    self.get_logger().warn(f"[ROSNode] AUTO_CLEANING模式下拒绝指令: {command}，仅允许HOLD")
                    return
                if not self.rc_control and self.current_control_mode == "REMOTE":
                    self.current_control_mode = "NORMAL"
                    self.get_logger().info(f"[ROSNode] 状态切换时更新控制模式: REMOTE -> NORMAL")
                if self.rc_control and command in ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "HOLD"]:
                    self.get_logger().info(f"[ROSNode] 收到MQTT运动命令 {command}，暂时禁用RC控制")
                    self.rc_control = False
                    self.current_control_mode = "NORMAL"
                self.switch_state(command)
            elif command in STATE_DICT.values():
                # AUTO_CLEANING模式下只允许HOLD指令
                if is_rtk_nav_mode and command != "HOLD":
                    self.get_logger().warn(f"[ROSNode] AUTO_CLEANING模式下拒绝指令: {command}，仅允许HOLD")
                    return
                if not self.rc_control and self.current_control_mode == "REMOTE":
                    self.current_control_mode = "NORMAL"
                    self.get_logger().info(f"[ROSNode] 状态切换时更新控制模式: REMOTE -> NORMAL")
                if self.rc_control and command in ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "HOLD"]:
                    self.get_logger().info(f"[ROSNode] 收到MQTT运动命令 {command}，暂时禁用RC控制")
                    self.rc_control = False
                    self.current_control_mode = "NORMAL"
                key = [k for k, v in STATE_DICT.items() if v == command][0]
                self.switch_state(key)
            elif command == "CHANGE_ROUTE":
                route_id = cmd_obj.get("route_id", "")
                if route_id:
                    self.handle_route_change(route_id)
                else:
                    self.get_logger().warn("[ROSNode] CHANGE_ROUTE指令缺少route_id字段")
            else:
                self.get_logger().warn(f"[ROSNode] 不支持的command：{command} (支持: GET_STATUS/键{list(STATE_DICT.keys())}或状态名{list(STATE_DICT.values())})")
                return
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"[ROSNode] 消息不是有效JSON字典，尝试按字符串处理: {msg_data}, 错误: {e}")
            raw_command = msg_data.strip()
            # AUTO_CLEANING模式下只允许HOLD和RC_ENABLE/RC_DISABLE指令
            if is_rtk_nav_mode and raw_command not in ["HOLD", "RC_ENABLE", "RC_DISABLE"]:
                self.get_logger().warn(f"[ROSNode] AUTO_CLEANING模式下拒绝指令: {raw_command}，仅允许HOLD/RC_ENABLE/RC_DISABLE")
                return
            if raw_command in STATE_DICT:
                self.switch_state(raw_command)
            elif raw_command in STATE_DICT.values():
                key = [k for k, v in STATE_DICT.items() if v == raw_command][0]
                self.switch_state(key)
            else:
                self.get_logger().error(f"[ROSNode] 无效的字符串指令：{raw_command} (仅支持: {list(STATE_DICT.keys())})")
            self.publish_state()
        except Exception as e:
            # 修复问题2：移除ROS2不支持的exc_info参数，直接拼接异常信息和堆栈（可选：手动打印堆栈）
            self.get_logger().error(f"[ROSNode] 处理消息时发生未知错误: {msg_data}, 错误: {str(e)}, 堆栈信息: {traceback.format_exc()}")
            # 可选：如果需要打印完整堆栈，需先导入traceback模块（import traceback）
            self.publish_state()
    def dock_state_callback(self, msg):
        """处理来自MQTT的停靠传感器状态消息

        原来这个回调订阅了 /robot_state（本节点自身状态），导致收到自己的发布消息
        并提示 "未找到sensors字段"。现在我们通过桥接节点把 MQTT topic
        robot/<ID>/status 转发到 ROS topic /mqtt_status 并在此处订阅，因此 msg.data
        应该是外部平台下发的 JSON 字符串，其中包含 sensors 字段。
        """
        # self.get_logger().info(f"[ROSNode] 收到机器人状态消息: {msg.data}")
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
            
            # 修复问题1：先校验cmd_obj是否是字典类型，再查找sensors字段
            if not isinstance(cmd_obj, dict):
                self.get_logger().warn(f"[ROSNode] 消息不是JSON字典格式：{msg_data}，跳过字典解析")
                raise json.JSONDecodeError("Not a JSON dict", msg_data, 0)  # 主动抛错，进入下方字符串处理逻辑
            
            if "sensors" not in cmd_obj:
                # self.get_logger().warn(f"[ROSNode] 未找到sensors字段: {msg_data}")
                return
            
            self.dock_sensors = cmd_obj.get("sensors", [])
            self.get_logger().info(f"[ROSNode] 解析到sensors：{self.dock_sensors}")
            if self.dock_sensors == None:
                self.get_logger().info(f"[ROSNode] 解析到sensors为空，检查DOCK！！！")
                return
            # 在这里处理传感器数据
            # 检测 0x08 或 0x04 脉冲上升沿（0→1）
            dock_ok = (self.dock_sensors & 0x08) and not (self.dock_last_sensors & 0x08) or \
                      (self.dock_sensors & 0x04) and not (self.dock_last_sensors & 0x04)

            if dock_ok and not self.is_charging:
            # if (self.dock_sensors & 0x08 == 8 and self.dock_last_sensors & 0x08 != 8 and self.dock_last_sensors & 0x04 != 5) and not self.is_charging: #车体到位未充电
                self.get_logger().info("[ROSNode] 车体到位")
                if self.current_status == "DISABLE":
                    self.get_logger().info("[ROSNode] 电机非使能状态，允许充电")
                    self.start_charge_async()
                # self.query_volt_curr_async()
                # self.start_charge_sync()
            # 充电停止逻辑已移至 timer_callback 中统一处理
            
            self.dock_last_sensors = self.dock_sensors
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"[ROSNode] 消息不是有效JSON字典，尝试按字符串处理: {msg_data}, 错误: {e}")
            # 处理字符串形式的传感器数据
        except Exception as e:
            self.get_logger().error(f"[ROSNode] 处理消息时发生未知错误: {msg_data}, 错误: {str(e)}, 堆栈信息: {traceback.format_exc()}")
            self.publish_state()

    def get_adaptive_turn_speed(self, yaw_error_abs: float) -> float:
        """
        分级自适应转向基准速度（核心：大误差快，小误差慢）
        无需减小PID参数，通过基准速度分级实现快慢切换
        """
        if yaw_error_abs > 30:
            return 1.0 * self.motor_ctrl.BASE_SPEED # type: ignore # 大误差（>30°）：快速转向
        elif yaw_error_abs > 10:
            return 0.8 * self.motor_ctrl.BASE_SPEED   # 中误差（10°~30°）：中等速度
        else:
            return 0.1 * self.motor_ctrl.BASE_SPEED  # 小误差（<10°）：慢速转向，防止超调
        

    def handle_unloading_step(self):
        """出仓分步处理（修正：适配IMU更新频率，修复角度计算）"""
        # 如果流程被暂停，短路返回，等待恢复
        if self.bin_process_paused:
            return
        # 补充dock中传感器复位后再响应
        if (self.dock_sensors & 0x08) or (self.dock_sensors & 0x04):  # dock中左侧传感器被触发（有物体）
            self.get_logger().warn("[ROSNode] 拒绝进入出仓状态，仓内限位传感器触发！！！")
            return
        elif (self.dock_sensors & 0x02):  # dock中归位
             # ========== 新增：归位后首次初始化出仓流程 ==========
            if self.unloading_phase is None:
                self.get_logger().info("[UNLOADING] 检测到dock归位，初始化出仓流程")
                self.unloading_phase = "UNLOADING_FORWARD"  # 第一阶段：前进
                self.unloading_start_time = time.time()  # 记录真正的启动时间
                self.get_logger().info(f"[UNLOADING] 初始化完成，当前阶段：{self.unloading_phase}")

            # if self.unloading_phase is None:
            #     self.get_logger().warn("[ROSNode] 出仓阶段未初始化，停止定时器")
            #     self.unloading_timer.cancel()
            #     self.unloading_timer = None
            #     self.is_in_bin_process = False  # 重置进出仓标记
            #     return
            
            # 前置校验：IMU数据是否有效且最新
            # if self.imu_yaw_deg is None or (time.time() - self.last_imu_update_time) > self.imu_update_interval:
            #     self.get_logger().warn("[UNLOADING] IMU数据过期/无效，跳过本次处理")
            #     return
            
            current_time = time.time()
            # 新增：稳定达标计数器（类内变量，初始化在__init__里，下面会说）
            if not hasattr(self, 'yaw_stable_count_unloading'):
                self.yaw_stable_count_unloading = 0
            
            # ========== 阶段1：前进/后退 ==========
            if self.unloading_phase == "UNLOADING_FORWARD":
                if current_time - self.unloading_start_time < self.unloading_forword_threshold:
                    correction = 0  # 直线纠偏待添加
                    left_speed = self.motor_ctrl.BASE_SPEED + correction
                    right_speed = -self.motor_ctrl.BASE_SPEED + correction
                    self.set_motors_speed(left_speed, right_speed)
                else:
                    # self.get_logger().info("[UNLOADING] 前进阶段完成，进入转向阶段")
                    self.set_motors_speed(0.0, 0.0)  # 停止运动
                    self.get_logger().info("[UNLOADING] 等待RTK")
                    # self.unloading_phase = "UNLOADING_TURN"
                    self.unloading_phase = "COMPLETE"  # 直接进入完成阶段，等待GPS固定解（如果需要转向，可以在后续版本添加）
                    self.unloading_turn_start_time = current_time
                    
                    # 修正：目标角度归一化
                    # self.unloading_turn_target_deg = self.imu_yaw_deg + 90
                    # self.unloading_turn_target_deg = (self.unloading_turn_target_deg + 180) % 360 - 180
                    # self.get_logger().info(f"[UNLOADING] 转向目标角设定为{self.unloading_turn_target_deg:.2f}deg")

            # ========== 阶段2：转向 ==========
            # elif self.unloading_phase == "UNLOADING_TURN":            
                
            #     correction = self.get_speed_correction(self.unloading_turn_target_deg)  # 目标航向180度（假设出仓方向为正后方）
            #     # 修正：角度差计算（使用归一化后的误差）
            #     yaw_diff = self.get_heading_error(self.unloading_turn_target_deg)
            #     # 差值小于0，左转；差值大于0，右转，保持方向正确
            #     turn_speed = self.get_adaptive_turn_speed(yaw_diff) if yaw_diff <= 0 else -self.get_adaptive_turn_speed(yaw_diff)
            #     # self.motor_ctrl.BASE_SPEED
            #     # self.get_logger().info(f"[UNLOADING] 转向阶段 - 当前航向{self.imu_yaw_deg:.2f}deg，目标{self.unloading_turn_target_deg:.2f}deg，差值{yaw_diff:.2f}deg")
                
            #     left_speed = turn_speed + correction
            #     right_speed = turn_speed + correction
            #     self.set_motors_speed(left_speed, right_speed)
            #     # 修正2：稳定判定：连续3次误差<阈值，才判定完成（避免IMU抖动）
            #     if abs(yaw_diff) < self.yaw_diff_min:
            #         self.yaw_stable_count_unloading += 1
            #         self.get_logger().info(f"[UNLOADING] 角度误差达标，稳定计数={self.yaw_stable_count_unloading}/3")
            #         if self.yaw_stable_count_unloading >= 3:
            #             self.get_logger().info("[UNLOADING] 转向阶段完成，出仓结束（连续3次达标）")
            #             self.unloading_phase = "COMPLETE"
            #             self.set_motors_speed(0.0, 0.0)  # 停止运动
            #             self.yaw_stable_count_unloading = 0  # 误差不达标，计数器清零
            #     else:
            #         self.yaw_stable_count_unloading = 0  # 误差不达标，计数器清零
            #     if current_time - self.unloading_turn_start_time > self.unloading_turn_time_max:
            #         self.get_logger().warn(f"[UNLOADING] Timeout: 转向阶段超时，强制完成出仓")
            #         self.unloading_phase = "COMPLETE"
            #         self.set_motors_speed(0.0, 0.0)  # 停止运动
            
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
                    heading = self.imu_yaw_deg if self.imu_yaw_deg is not None else 0.00
                    heading = (heading + 360) % 360  # 归一化到0-360度
                    self.get_logger().info(f"[UNLOADING] 准备发布出仓GPS坐标到RTK: 经度={self.unloading_lon:.6f}, 纬度={self.unloading_lat:.6f}, 航向={heading:.2f}°")
                    unloading_gps_msg = Vector3()
                    unloading_gps_msg.x = self.unloading_lon
                    unloading_gps_msg.y = self.unloading_lat
                    unloading_gps_msg.z = heading
                    self.unloading_gps_pub.publish(unloading_gps_msg)
                    self.get_logger().info(f"[UNLOADING] 已发布出仓GPS坐标到/unloading_gps话题")
                    self.get_logger().info("[UNLOADING] 出仓流程完成")
                    self.switch_state('h') # 切回HOLD状态，确保电机停止
                    time.sleep(2.0)  # 确保状态切换生效
                    self.current_control_mode = "AUTO_CLEANING"
                    self.switch_state('r') # RTK导航模式，准备接受RTK速度指令
                    # 清理超时标记
                    if hasattr(self, 'unloading_gps_wait_start'):
                        delattr(self, 'unloading_gps_wait_start')
                    # 终止定时器
                    self.unloading_timer.cancel()
                    self.unloading_timer = None
                    self.is_in_bin_process = False  # 重置进出仓标记
                # 2. 新增：超时兜底（GPS长期无固定解，强制退出）
                elif elapsed_time > MAX_GPS_WAIT_TIME:
                    self.get_logger().error(f"[UNLOADING] 等待GPS固定解超时（{MAX_GPS_WAIT_TIME}s），状态码始终为{self.rtk_status}，进入DISABLE")
                    # # 可选：记录当前非固定解的坐标（或置空）
                    # self.unloading_lon = self.current_lon
                    # self.unloading_lat = self.current_lat
                    # 清理超时标记+终止定时器
                    delattr(self, 'unloading_gps_wait_start')
                    self.is_in_bin_process = False
                    self.switch_state('z') # 切回HOLD状态，确保电机停止
                    time.sleep(2.0)  # 确保状态切换生效
                    # self.current_control_mode = "AUTO_CLEANING"
                    # self.switch_state('r') # RTK导航模式，准备接受RTK速度指令
                    self.unloading_timer.cancel()
                    self.unloading_timer = None
                # 3. 未超时+非固定解：继续等待，可选降频打印日志（避免刷屏）
                else:
                    # 每3s打印一次警告（替代0.1Hz高频打印）
                    if int(elapsed_time) % 3 == 0 and abs(elapsed_time - int(elapsed_time)) < 0.1:
                        self.get_logger().warn(f"[UNLOADING] 出仓完成，GPS状态不佳（状态码{self.rtk_status}），已等待{elapsed_time:.1f}s，继续等待...")
                    elif self.unloading_phase == "COMPLETE":
                        # get_gps
                        if self.rtk_status == 4:  # RTK固定解，GPS数据可靠
                            self.get_logger().info(f"[UNLOADING] 出仓完成，当前GPS坐标：经度{self.unloading_lon:.6f}，纬度{self.unloading_lat:.6f}")
                            heading = self.imu_yaw_deg if self.imu_yaw_deg is not None else 0.00
                            heading = (heading + 360) % 360  # 归一化到0-360度
                            self.get_logger().info(f"[UNLOADING] 准备发布出仓GPS坐标到RTK: 经度={self.unloading_lon:.6f}, 纬度={self.unloading_lat:.6f}, 航向={heading:.2f}°")
                            # pub unloading result
                            unloading_gps_msg = Vector3()
                            unloading_gps_msg.x = self.unloading_lon
                            unloading_gps_msg.y = self.unloading_lat
                            unloading_gps_msg.z = heading
                            self.unloading_gps_pub.publish(unloading_gps_msg)
                            self.get_logger().info(f"[UNLOADING] 已发布出仓GPS坐标到/unloading_gps话题")
                            self.get_logger().info("[UNLOADING] 出仓流程完成")
                            self.is_in_bin_process = False  # 重置进出仓标记
                            self.switch_state('h') # 切回HOLD状态，确保电机停止
                            time.sleep(2.0)  # 确保状态切换生效
                            self.current_control_mode = "AUTO_CLEANING"
                            self.switch_state('r') # RTK导航模式，准备接受RTK速度指令
                            self.unloading_timer.cancel()
                            self.unloading_timer = None
                        elif int(elapsed_time) % 3 == 0 and abs(elapsed_time - int(elapsed_time)) < 0.1:
                            self.get_logger().warn(f"[UNLOADING] 出仓完成，但GPS状态不佳（状态码{self.rtk_status}），无法获取可靠坐标,继续等待GPS修正")
                            self.switch_state('h') # 切回HOLD状态，确保电机停止


            
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
        if (self.dock_sensors & 0x08) or (self.dock_sensors & 0x04):  # dock中左侧传感器被触发（有物体）
                self.get_logger().warn("[ROSNode] 拒绝进入进仓状态，仓内限位传感器触发！！！")
                return
        elif self.dock_sensors & 0x02:  # dock中归位
            # 归位后首次初始化进仓流程
            if self.loading_phase is None:
                self.get_logger().info("[LOADING] 检测到dock归位，初始化进仓流程")
                self.loading_phase = "LOADING_TURN"  # 第一阶段：调整角度
                self.loading_start_time = time.time()
                self.get_logger().info(f"[LOADING] 初始化完成，当前阶段：{self.loading_phase}")
        
            # if self.loading_phase is None:
            #     self.get_logger().warn("[ROSNode] 进仓阶段未初始化，停止定时器")
            #     if self.loading_timer is not None:
            #         self.loading_timer.cancel()
            #         self.loading_timer = None
            #     self.is_in_bin_process = False
            #     self.bin_process_origin_mode = None
            #     self.bin_process_paused = False
            #     return
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
                    # 差值小于0，左转；差值大于0，右转，保持方向正确
                    turn_speed = self.get_adaptive_turn_speed(yaw_diff) if yaw_diff <= 0 else -self.get_adaptive_turn_speed(yaw_diff)
                    left_speed = turn_speed + correction
                    right_speed = turn_speed + correction
                    self.set_motors_speed(left_speed, right_speed)
                    
                    # 计算归一化后的角度差
                    # self.get_logger().info(
                    #     f"[LOADING] 角度调整阶段 - 当前航向{self.imu_yaw_deg:.2f}deg，"
                    #     f"目标{self.loading_turn_target_deg:.2f}deg，差值{yaw_diff:.2f}deg"
                    # )
                    
                    # 修正2：稳定判定：连续3次误差<阈值，才判定完成（避免IMU抖动）
                    if abs(yaw_diff) < self.yaw_diff_min:
                        self.yaw_stable_count += 1
                        self.get_logger().info(f"[LOADING] 角度误差达标，稳定计数={self.yaw_stable_count}/3")
                        if self.yaw_stable_count >= 3:
                            self.get_logger().info("[LOADING] 角度调整稳定完成（连续3次达标），进入后退进仓阶段")
                            self.loading_phase = "LOADING_BACKWARD"
                            self.loading_backward_start_time = current_time
                            self.yaw_stable_count = 0  # 重置计数器
                    else:
                        self.yaw_stable_count = 0  # 误差不达标，计数器清零
                    
                    # 超时逻辑保留
                    # if current_time - self.loading_start_time > self.loading_turn_time:
                    #     self.get_logger().warn("[LOADING] 角度调整超时，进入DISABLE")
                    #     self.switch_state('z')  # 进入DISABLE状态，人工干预
                    #     # self.loading_phase = "LOADING_BACKWARD"
                    #     # self.loading_backward_start_time = current_time
                    #     self.yaw_stable_count = 0
                
                # ========== +低频日志）==========
                elif self.loading_phase == "LOADING_BACKWARD":
                    # 初始化变量
                    left = self.laser_distance[0]
                    right = self.laser_distance[1]
                    diff_dis = abs(left - right)  # 取绝对值，只关注差值大小
                    base_speed = self.motor_ctrl.BASE_SPEED
                    left_speed = 0.0
                    right_speed = 0.0
                    correction = 0.0  # 后退阶段彻底停止航向修正，解决频率混乱
                    if self.laser_distance is None:
                        self.get_logger().info("[LOADING] 激光测距数据不可用")
                        return
                    
                    # 步骤1：激光距离<3000mm（核心条件）
                    if left < 3000 and right < 3000:
                        self.get_logger().info(f"[LOADING] 激光距离有效 - 左：{left}mm, 右：{right}mm, 差值：{diff_dis}mm")
                        
                        # 步骤2：判断激光差值是否>1000mm（大幅偏离）执行「偏转→后退2s→反向偏转检查」流程
                        # if diff_dis > 1000:
                        if diff_dis > 1000 and self.correction_count < 5 or self.in_full_correction:  # 最多2次调整

                            self.get_logger().info("[LOADING] 大幅偏离（差值>1000mm），强力旋转对准（中速）")
                            # # 强力纠偏：左转/右转 速度为 BASE_SPEED/2.0
                            # if (left - right) < 0:  # 左转
                            #     left_speed = base_speed / 2.0
                            #     right_speed = base_speed / 2.0
                            # else:  # 右转
                            #     left_speed = -base_speed / 2.0
                            #     right_speed = -base_speed / 2.0
                            self.in_full_correction = True

                            if self.correction_state == "IDLE":
                                self.correction_count += 1
                                self.correction_start_time = current_time
                                self.get_logger().info(f"[LOADING] 大幅偏离，第{self.correction_count}次调整-开始偏转")

                                if (left - right) < 0:  # 左转
                                    left_speed = base_speed / 4.0  # 左转：双正（低速）
                                    right_speed = base_speed / 4.0
                                    self.correction_state = "LEFT_DEFLECT"

                                else:  # 右转
                                    left_speed = -base_speed / 4.0  # 右转：双负（低速）
                                    right_speed = -base_speed / 4.0
                                    self.correction_state = "RIGHT_DEFLECT"
                                self.last_state = self.correction_state
                            # 阶段1续：保持偏转直到时长结束
                            elif self.correction_state in ["LEFT_DEFLECT", "RIGHT_DEFLECT"]:
                                if current_time - self.correction_start_time < self.correction_duration:
                                    # 保持偏转速度
                                    if self.correction_state == "LEFT_DEFLECT":
                                        left_speed = base_speed / 4.0
                                        right_speed = base_speed / 4.0
                                    else:
                                        left_speed = -base_speed / 4.0
                                        right_speed = -base_speed / 4.0
                                else:
                                    # 偏转结束，切换到后退阶段（固定2秒）
                                    self.correction_start_time = current_time
                                    self.correction_state = "RETRACT"
                                    self.get_logger().info("[LOADING] 偏转结束，开始后退2秒")
                            
                            # 阶段2：固定后退2秒（左负右正）
                            elif self.correction_state == "RETRACT":
                                if current_time - self.correction_start_time < self.retract_duration:
                                    # 后退速度：左负右正（严格匹配你的定义）
                                    left_speed = base_speed / 4.0
                                    right_speed = -base_speed / 4.0
                                else:
                                    # 后退结束，切换到反向偏转检查阶段
                                    self.correction_start_time = current_time
                                    self.correction_state = "CHECK"
                                    self.get_logger().info("[LOADING] 后退2秒完成，开始反向偏转检查")
                            
                            # 阶段3：反向偏转检查（修正差值）
                            elif self.correction_state == "CHECK":
                                # 先检查是否达标：diff_dis < 1000 则立即退出
                                if diff_dis < 10:
                                    self.get_logger().info(f"[LOADING] 反向检查达标（差值{diff_dis}mm<10），退出检查阶段")
                                    # 重置状态+解锁+计数+1
                                    self.in_full_correction = False
                                    self.correction_state = "IDLE"
                                    self.last_state = None
                                    self.correction_count += 1
                                    left_speed = 0.0  # 停止反向偏转
                                    right_speed = 0.0
                                else:
                                    # 未达标则继续反向偏转，同时增加超时保护
                                    check_elapsed = current_time - self.correction_start_time
                                    if check_elapsed < self.check_max_timeout:
                                        self.get_logger().warning(f"[LOADING] 反向检查中（已持续{check_elapsed:.1f}s），当前差值{diff_dis}")
                                        # 反向偏转：原左转则右转，原右转则左转
                                        if self.last_state == "LEFT_DEFLECT":  # 原偏左 → 反向右转（双负）
                                            left_speed = -base_speed / 6.0
                                            right_speed = -base_speed / 6.0
                                        elif self.last_state == "RIGHT_DEFLECT":  # 原偏右 → 反向左转（双正）
                                            left_speed = base_speed / 6.0
                                            right_speed = base_speed / 6.0
                                    else:
                                        # 超时未达标，强制退出（避免无限检查）
                                        self.get_logger().warning(f"[LOADING] 反向检查超时（{self.check_max_timeout}s），强制退出")
                                        self.in_full_correction = False
                                        self.correction_state = "IDLE"
                                        self.last_state = None
                                        self.correction_count += 1
                                        left_speed = 0.0
                                        right_speed = 0.0

                            #     if current_time - self.correction_start_time < self.correction_duration:
                            #         # 反向偏转：原左转则右转，原右转则左转
                            #         if self.last_state == "LEFT_DEFLECT":  # 原偏左 → 反向右转（双负）
                            #             left_speed = -base_speed / 6.0
                            #             right_speed = -base_speed / 6.0
                            #         elif self.last_state == "RIGHT_DEFLECT":  # 原偏右 → 反向左转（双正）
                            #             left_speed = base_speed / 6.0
                            #             right_speed = base_speed / 6.0
                            #     else:
                            #         # 检查结束，重置状态，完成一次完整调整
                            #         self.correction_state = "IDLE"
                            #         self.last_state = None
                            #         self.in_full_correction = False
                            #         self.get_logger().info(f"[LOADING] 第{self.correction_count}次调整完成，检查差值：{diff_dis}mm")
                            # self.get_logger().info(f"correction_state={self.correction_state}, correction_count={self.correction_count}")
                        # 步骤3：差值≤1000mm → 中等速度纠偏直行
                        else:
                            # 步骤7：激光距离<230mm → 最终对位判断
                            # if left < 230 and right < 230:
                            if left < 375 and right < 375:
                                if not hasattr(self, 'straight_loading_time'):
                                    self.straight_loading_time = current_time
                                self.get_logger().info("[LOADING] 极近距离（<375mm），判断差值是否<2mm 进行最终对位")
                                # 定时器：5秒后切换到完成阶段（确保电机停止）
                                if self.straight_loading_time is not None:
                                    elapsed_time = current_time - self.straight_loading_time
                                    if elapsed_time >= 5.0 and self.complete_state == False:
                                        # self.get_logger().info("[LOADING] 进仓完成超时，强制进入完成阶段")
                                        # self.in_full_correction = False
                                        # self.loading_phase = "COMPLETE"
                                        # self.complete_state = True
                                        self.last_backward_log_time = 0.0  # 重置日志时间
                                        self.correction_count = 0  # 重置次数
                                        self.straight_loading_time = None
                                        self.get_logger().info("[LOADING] 进仓未完成，但已持续极近距离超过5秒，进入DISABLE状态，等待人工干预")
                                        self.switch_state('z') # 切回DISABLE状态，确保电机停止
                                # 步骤8：差值<2mm → 停止
                                if diff_dis < 2:
                                    self.get_logger().info("[LOADING] 对位完成，停止")
                                    left_speed = 0.0
                                    right_speed = 0.0
                                    self.get_logger().info("[LOADING] 后退进仓完成，进入完成阶段")
                                    self.in_full_correction = False
                                    self.loading_phase = "COMPLETE"
                                    self.complete_state = True
                                    self.is_charging = False  #清除充电状态，衔接后续充电
                                    # self.battery_full_charge = False #清除充电状态，衔接后续充电
                                    self.last_backward_log_time = 0.0  # 重置日志时间
                                    self.correction_count = 0  # 重置次数
                                    self.straight_loading_time = None
                                    self.switch_state('z') # 切回DISABLE状态，确保电机停止
                                    
                                # 步骤9：差值≥2mm → 最终对位（低速旋转）
                                else:
                                    self.get_logger().warning("[LOADING] 极近距离但差值≥2mm, 最终对位")
                                    if (left - right) < 0:
                                        left_speed = base_speed / 10.0
                                        right_speed = base_speed / 10.0
                                    else:
                                        left_speed = -base_speed / 10.0
                                        right_speed = -base_speed / 10.0
                                    
                            # 步骤4：激光距离<1000mm → 进入低速纠偏阶段
                            elif left < 1000 and right < 1000:
                                # self.get_logger().info("[LOADING] 近距离（<1000mm），判断差值是否<5mm")
                                # 步骤5：差值≥10mm → 低速旋转对准
                                if diff_dis >= 10:
                                    self.get_logger().info("[LOADING] 中等偏差（>10mm），低速旋转对准")
                                    if (left - right) < 0:
                                        left_speed = base_speed / 10.0
                                        right_speed = base_speed / 10.0
                                    else:
                                        left_speed = -base_speed / 10.0
                                        right_speed = -base_speed / 10.0
                                # 步骤6：差值<10mm → 低速纠偏直行
                                else:
                                    self.get_logger().info("[LOADING] 差值<10mm，直行")
                                    left_speed = -base_speed / 1.5
                                    right_speed = base_speed / 1.5
                            else:
                                # self.get_logger().info(f"[LOADING] 中距离（≥1000mm），判断差值是否<10mm")
                                # 差值≥40mm → 中速旋转对准
                                if diff_dis >= 20:
                                    self.get_logger().info("[LOADING] 大偏差（>10mm），中速旋转对准")
                                    if (left - right) < 0:
                                        left_speed = base_speed / 4.0
                                        right_speed = base_speed / 4.0
                                    else:
                                        left_speed = -base_speed / 4.0
                                        right_speed = -base_speed / 4.0
                                else:
                                    left_speed = -base_speed
                                    right_speed = base_speed
                                
                        
                        # 设置最终电机速度
                        self.set_motors_speed(left_speed, right_speed)
                    
                        # 修正3：后退日志5秒1次，避免高频输出
                        if current_time - self.last_backward_log_time >= 5.0:
                            self.get_logger().info(f"[LOADING] 后退进仓阶段 - 已持续{current_time - self.loading_backward_start_time:.1f}秒")
                            self.last_backward_log_time = current_time
                    else:
                        self.get_logger().info("[LOADING] 寻找目标")
                        if left < 3000 and right > 3000:
                            # 左转寻找目标
                            left_speed = base_speed
                            right_speed = base_speed
                        elif left > 3000 and right < 3000:
                            # 右转寻找目标
                            left_speed = -base_speed
                            right_speed = -base_speed
                        else:
                            # 直行寻找dock
                            left_speed = -base_speed 
                            right_speed = base_speed  
                        self.set_motors_speed(left_speed, right_speed)
                
                # ========== 阶段3：进仓完成 ==========
                elif self.loading_phase == "COMPLETE":
                    self.get_logger().info("[LOADING] 进仓流程完成，电机停止")
                    if self.loading_timer is not None:
                        self.loading_timer.cancel()
                        self.loading_timer = None
                    self.nav_status = "IDLE"
                    self.is_in_bin_process = False
                    self.switch_state('z') # DISABLE状态，确保电机停止
                    self.yaw_stable_count = 0  # 重置计数器
                    self.last_backward_log_time = 0.0  # 重置日志时间
            except Exception as e:
                self.get_logger().error(f"[LOADING] 执行异常：{str(e)}")
                self.is_in_bin_process = False
                self.yaw_stable_count = 0

    def set_motors_speed(self, left_speed: float, right_speed: float) -> None:
        """设置双电机速度（完全保留原有功能）"""
        # 保存当前速度值
        self.current_left_speed = float(left_speed)
        self.current_right_speed = float(right_speed)
        
        # 左电机（ID=1）
        self.motor_ctrl.motor_set_speed(self.motor_ctrl.motors[0]["id"], left_speed)
        # 右电机（ID=2）
        self.motor_ctrl.motor_set_speed(self.motor_ctrl.motors[1]["id"], right_speed)

        # 构造并发布速度消息
        wheel_speed_msg = Vector3()
        wheel_speed_msg.x = float(left_speed)    # 左轮角速度
        wheel_speed_msg.y = float(right_speed)   # 右轮角速度
        wheel_speed_msg.z = float(self.brush_speed)
        self.speed_pub.publish(wheel_speed_msg)

    def publish_rc_channels(self) -> None:
        remote_state = self.sbus_remote.get_remote_state()
        raw_channels = remote_state.get("channel_raw", [])
        norm_channels = remote_state.get("channel_normalized", [])
        msg = Float32MultiArray()
        msg.data = [1.0 if remote_state.get("is_connected", False) else 0.0]
        msg.data.extend([float(v) for v in raw_channels[:16]])
        msg.data.extend([float(v) for v in norm_channels[:16]])
        self.rc_channels_pub.publish(msg)
        
    def set_brush_speed(self, brush_speed: float) -> None:
        """设置 刷 电机速度"""
        # 刷盘电机（ID=3）
        result = self.motor_ctrl.motor_set_speed(self.motor_ctrl.motors[2]["id"], brush_speed)
        if result:
            self.brush_speed = float(brush_speed)
        else:
            self.get_logger().warn(f"[ROSNode] 刷盘电机速度下发失败：{brush_speed}")
        
    # -------------------------- 异步调用方式（推荐，非阻塞） --------------------------
    def start_charge_async(self):
        """异步调用开始充电服务"""
        req = ChargeControl.Request()
        
        # 发送异步请求并设置回调
        future = self.cli_start_charge.call_async(req)
        future.add_done_callback(self._start_charge_callback)
        return future

    def _start_charge_callback(self, future):
        """异步启动充电的回调函数"""
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"异步启动充电成功: {resp.message}")
                self.charge_resume_count += 1
                self.is_charging = True  # 更新充电状态
                self.is_charge_paused = False  # 充电成功启动，清除暂停标志
                self.last_charging_fault = 0  # 重置故障码状态，避免假充判断错误
            else:
                self.get_logger().error(f"异步启动充电失败: {resp.message}")
                self.is_charging = False  # 确保状态一致
        except Exception as e:
            self.get_logger().error(f"异步启动充电异常: {str(e)}")
            self.is_charging = False

    def stop_charge_async(self):
        """异步调用停止充电服务"""
        req = ChargeControl.Request()
        future = self.cli_stop_charge.call_async(req)
        future.add_done_callback(self._stop_charge_callback)
        return future

    def _stop_charge_callback(self, future):
        """异步停止充电的回调函数"""
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"异步停止充电成功: {resp.message}")
                self.is_charging = False  # 更新充电状态
            else:
                self.get_logger().error(f"异步停止充电失败: {resp.message}")
                self.is_charging = True  # 更新充电状态

        except Exception as e:
            self.get_logger().error(f"异步停止充电异常: {str(e)}")
            self.is_charging = True  # 更新充电状态
            
# -------------------------- 主函数入口 --------------------------
def main(args=None):
    rclpy.init(args=args)

    # 创建电机控制节点
    motor_node = MotorControlNode()
    # 使用多线程执行器（支持异步调用）
    executor = MultiThreadedExecutor()

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
