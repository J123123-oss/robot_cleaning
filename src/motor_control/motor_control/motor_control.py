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
from std_msgs.msg import String, UInt8
from geometry_msgs.msg import Vector3
import subprocess
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_control.motor_driver import CanMotorDriver
from motor_control.remote_control import SBUSRemoteController
from rtk_nav.rtk_nav import RTKNavControlNode

# -------------------------- 全局配置与枚举 --------------------------
STATE_DICT = {
    'z': "STOP",
    'x': "START",
    'w': "FORWARD",
    's': "BACKWARD",
    'a': "TURN_LEFT",
    'd': "TURN_RIGHT",
    'l': "LOADING",
    'u': "UNLOADING"
}

MAX_SPEED = 260.0   # 遥控器最大速度
MIN_SPEED = -160.0  # 遥控器最小速度
# 通道灵敏度系数（可微调，0~1之间，用于控制通道对速度的影响程度）
CH2_SENSITIVITY = 1.0  # 前进后退灵敏度
CH3_SENSITIVITY = 1.0  # 左右旋转灵敏度
DEAD_ZONE = 0.05       # 控制死区

# 控制模式枚举（新增RTK导航模式）
class ControlMode:
    REMOTE = "REMOTE"
    NORMAL = "NORMAL"
    RTK_NAV = "RTK_NAV"

# -------------------------- 电机控制节点（独立ROS2节点） --------------------------
class MotorControlNode(Node):
    def __init__(self, node_name='motor_control_node'):
        super().__init__(node_name)

        # 循环频率：10Hz（兼容原有逻辑，可调整）
        self.rate = self.create_rate(10)
        # UNLOADING parameters
        # 出仓阶段标记（关键：拆分出仓为多个阶段）
        self.unloading_forword_threshold = 10.0 # seconds
        self.unloading_phase = None  # None/"FORWARD"/"UNLOADING_TURN"/"COMPLETE"
        self.unloading_start_time = 0.0  # 出仓开始时间
        self.unloading_turn_target_rad = 0.0  # 出仓转向目标角
        self.unloading_timer: Optional[Timer] = None  # 出仓专用定时器

        self.loading_turn_target_rad = 0.0  # 进仓转向目标角
        self.loading_backward_threshold = 10.0  # 进仓后退时长（秒）
        self.loading_phase = None
        self.loading_start_time = 0.0
        self.loading_timer: Optional[Timer] = None
        self.loading_backward_start_time = 0.0

        self.nav_status = None

        # 1. 初始化电机控制模块
        self.motor_ctrl = CanMotorDriver(node_name='can_motor_driver', channel='vcan0', interface='socketcan', baudrate=1000000)
        self.get_logger().info("[ROSNode] 开始初始化CAN串口...")
        if not self.motor_ctrl.create_can_bus():
            self.get_logger().warn("[ROSNode] CAN串口首次初始化失败，进入重连模式")
            if not self.motor_ctrl.reconnect_can_bus():
                self.get_logger().fatal("[ROSNode] CAN串口重连失败，无法继续运行，退出节点")
                rclpy.signal_shutdown("CAN串口重连失败")
                return

        # 2. 初始化遥控器模块
        self.sbus_remote = SBUSRemoteController()
        if not self.sbus_remote._init_serial():
            self.get_logger().warn("[ROSNode] 遥控器串口初始化失败，仅支持RTK和键盘控制")

        self.rtk_nav = RTKNavControlNode()
        self.imu_yaw_rad = self.rtk_nav.imu_yaw

        # 3. ROS2 订阅器
        self.keyboard_sub = self.create_subscription(
            String,
            "/keyboard/control",
            self.keyboard_callback,
            10  # QoS深度
        )
        # 新增：订阅RTK节点发布的电机速度指令
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
        # 4. ROS2 发布器
        self.state_pub = self.create_publisher(String, "/motor/state", 10)  # 电机状态
        self.speed_pub = self.create_publisher(Vector3, "/motor/current_speed", 10)  # 电机当前速度
        self.mode_pub = self.create_publisher(String, "/control/mode", 10)  # 当前控制模式

        # 全局变量
        self.current_control_mode = ControlMode.NORMAL  # 默认普通模式
        # self.current_control_mode = self.sbus_remote.control_mode  # 默认普通模式
        self.rtk_left_speed = 0.0  # 存储RTK订阅的左轮速度
        self.rtk_right_speed = 0.0 # 存储RTK订阅的右轮速度

        self.status_list = [
            "STOP", "START", "FORWARD", "BACKWARD", "LOADING", "UNLOADING",
            "TURN_LEFT", "TURN_RIGHT", "PAUSE"
        ]
        self.current_status = self.status_list[0]

        # 初始化电机（进入START状态）
        self.switch_state('x')

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1秒 = 10Hz

    def timer_callback(self):
        # 检查CAN串口是否正常打开
        if not self.motor_ctrl.bus:
            self.get_logger().warn("[ROSNode] CAN串口连接断开，尝试重连...")
            self.motor_ctrl.reconnect_can_bus()

        # 1. 发布当前控制模式（给RTK节点）
        mode_msg = String()
        mode_msg.data = self.current_control_mode
        self.mode_pub.publish(mode_msg)

        # 2. 按控制模式执行不同逻辑
        if self.current_control_mode == ControlMode.REMOTE:
            # 遥控器模式（原有逻辑）
            if self.current_status != "START":
                self.switch_state('x')
            try:

                # 步骤1：获取通道2（前进后退）和通道3（左右旋转）的归一化值（-1.0 ~ 1.0）
                ch2_norm = self.sbus_remote.get_channel_normalized(ch_idx=2)  # 前进后退
                ch3_norm = self.sbus_remote.get_channel_normalized(ch_idx=3)  # 左右旋转

                ch2_norm = 0.0 if abs(ch2_norm) < DEAD_ZONE else ch2_norm
                ch3_norm = 0.0 if abs(ch3_norm) < DEAD_ZONE else ch3_norm

                # 步骤2：计算通道2的差速分量（前进后退，左右轮速度相反）
                # ch2_norm > 0：前进；ch2_norm < 0：后退；=0：静止
                forward_backward_left = ch2_norm * MAX_SPEED * CH2_SENSITIVITY
                forward_backward_right = -forward_backward_left  # 左右轮速度相反数，实现前进后退

                # 步骤3：计算通道3的同速分量（左右旋转，左右轮速度相同）
                # ch3_norm > 0：向右旋转；ch3_norm < 0：向左旋转；=0：不旋转
                rotate_left_right = ch3_norm * MAX_SPEED * CH3_SENSITIVITY  # 同速分量，左右轮共用

                # 步骤4：速度叠加（核心：两个通道的分量相加，实现同时控制）
                left_speed_target = forward_backward_left + rotate_left_right
                right_speed_target = forward_backward_right + rotate_left_right

                # 步骤5：上下限约束，确保速度在[MIN_SPEED, MAX_SPEED]范围内
                left_speed = max(MIN_SPEED, min(MAX_SPEED, left_speed_target))
                right_speed = max(MIN_SPEED, min(MAX_SPEED, right_speed_target))

                # 设置电机速度
                self.set_motors_speed(left_speed, right_speed)
                self.get_logger().debug(
                    f"[RemoteControl] 左轮：{left_speed:.2f}，右轮：{right_speed:.2f} "
                    f"通道2归一化值：{ch2_norm:.2f}，通道3归一化值：{ch3_norm:.2f}"
                )
            except Exception as e:
                self.get_logger().warn(f"[ROSNode] 获取遥控器速度失败：{e}")
                self.set_motors_speed(0.0, 0.0)

        elif self.current_control_mode == ControlMode.NORMAL:
            # 普通模式（键盘控制，原有逻辑）
            state_msg = String()
            state_msg.data = str(self.current_status)
            self.state_pub.publish(state_msg)

            # 按当前状态赋值速度
            if self.current_status == "FORWARD":
                left_speed = -self.motor_ctrl.BASE_SPEED
                right_speed = self.motor_ctrl.BASE_SPEED
            elif self.current_status == "BACKWARD":
                left_speed = self.motor_ctrl.BASE_SPEED
                right_speed = -self.motor_ctrl.BASE_SPEED
            elif self.current_status == "TURN_LEFT":
                left_speed = -self.motor_ctrl.BASE_SPEED
                right_speed = -self.motor_ctrl.BASE_SPEED
            elif self.current_status == "TURN_RIGHT":
                left_speed = self.motor_ctrl.BASE_SPEED
                right_speed = self.motor_ctrl.BASE_SPEED
            else:
                left_speed = 0.0
                right_speed = 0.0
            self.set_motors_speed(left_speed, right_speed)

        elif self.current_control_mode == ControlMode.RTK_NAV:
            # RTK导航模式（新增逻辑：使用RTK订阅的速度）
            if self.current_status != "START":
                self.switch_state('x')
            # 将RTK订阅的速度转换为电机可识别的量级
            left_speed = self.rtk_left_speed 
            right_speed = self.rtk_right_speed 
            self.set_motors_speed(left_speed, right_speed)
            self.get_logger().debug(f"[RTKControl] 左轮：{left_speed:.2f}，右轮：{right_speed:.2f}")

        # 处理回调并延时
        # rclpy.spin_once(self, timeout_sec=0.01)
        # self.rate.sleep()
    def keyboard_callback(self, msg: String) -> None:
        """键盘控制回调（新增RTK模式切换）"""
        key = msg.data.strip().lower()

        # 模式切换指令
        if key == 'r':
            # 切换到RTK导航模式
            self.current_control_mode = ControlMode.RTK_NAV
            self.get_logger().info(f"[ROSNode] 控制模式切换：→ {ControlMode.RTK_NAV}")
            # 切换时自动使能电机
            if self.current_status != "START":
                self.switch_state('x')
        elif key == 'n':
            # 切回普通模式
            self.current_control_mode = ControlMode.NORMAL
            self.get_logger().info(f"[ROSNode] 控制模式切换：→ {ControlMode.NORMAL}")
        elif key == 'm':
            # 切换到遥控器模式
            self.current_control_mode = ControlMode.REMOTE
            self.get_logger().info(f"[ROSNode] 控制模式切换：→ {ControlMode.REMOTE}")
        # 原有状态切换逻辑（仅非RTK模式生效）
        elif key in STATE_DICT:
            if self.current_control_mode != ControlMode.RTK_NAV :
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
        # self.get_logger().info(f"[RTKNavStatus] 当前RTK导航状态：{self.nav_status}")
        if self.nav_status == "COMPLETED":
            self.switch_state('l')

    def io_data_callback(self, msg: UInt8):
        """处理IO数据回调（可根据需要扩展功能）"""
        if self.current_control_mode == ControlMode.REMOTE:
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
            if msg.data is not 0:
                self.get_logger().info(f"--------------Boundary Detected--------------")
                self.motor_set_speed(1, 0)
                self.motor_set_speed(2, 0) 
            # else:
                # recover speed
                # self.motor_set_speed(1, 0)
                # self.motor_set_speed(2, 0) 
    def switch_state(self, key: str) -> None:
        """状态机切换逻辑（完全保留原有功能）"""
        new_state = STATE_DICT[key]
        if new_state not in self.status_list:
            self.get_logger().warn(f"[ROSNode] 无效状态切换请求：{new_state}")
            return
        if new_state == self.current_status:
            self.get_logger().info(f"[ROSNode] 已处于{new_state}状态，无需切换")
            return

        self.get_logger().info(f"[ROSNode] 状态切换：{self.current_status} → {new_state}")
        self.current_status = new_state

        if self.unloading_timer is not None:
            self.unloading_timer.cancel()
            self.unloading_timer = None

        # 状态执行逻辑
        if new_state == "STOP":
            # 停止：失能所有电机
            for motor in self.motor_ctrl.motors:
                self.motor_ctrl.motor_set_speed(motor["id"], 0)  # 初始速度0
                time.sleep(0.01)
                self.motor_ctrl.motor_disable(motor["id"])

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
            # 创建10Hz定时器处理出仓分步逻辑
            self.unloading_timer = self.create_timer(0.1, self.handle_unloading_step)
        elif new_state == "LOADING":
            self.current_control_mode = ControlMode.NORMAL
            self.get_logger().info("[ROSNode] 进入进仓状态，启动进仓定时器")
            self.current_status = new_state
            self.loading_phase = "LOADING_TURN"  # 第一阶段：调整角度
            self.loading_start_time = time.time()
            # # 初始化进仓转向目标（=出仓,90度）
            if self.rtk_nav.imu_yaw is None:
                self.get_logger().warn("[LOADING] IMU航向角未获取，使用默认0度作为基准")
                self.loading_turn_target_rad = math.pi / 2  # 转90度（进仓角度）
            else:
                self.loading_turn_target_rad = self.rtk_nav.imu_yaw + math.pi / 2  #   TEST!!!!!
                # 角度归一化到[-π, π]
                self.loading_turn_target_rad = math.atan2(
                    math.sin(self.loading_turn_target_rad),
                    math.cos(self.loading_turn_target_rad)
                )
            self.loading_timer = self.create_timer(0.1, self.handle_loading_step)

            # current_time = time.time()
            # if self.unloading_flag == False:
            #     self.unloading_time = current_time
            #     # 前进：双电机正转
            #     left_speed = -self.motor_ctrl.BASE_SPEED
            #     right_speed = self.motor_ctrl.BASE_SPEED
            #     self.set_motors_speed(left_speed, right_speed)
            #     target = self.imu_yaw if self.imu_yaw is not None else 0 # rad
            #     self.unloading_flag = True

            # if current_time - self.unloading_time  > self.unloading_forword_threshold:
            #     # 右转
            #     left_speed = -self.motor_ctrl.BASE_SPEED
            #     right_speed = -self.motor_ctrl.BASE_SPEED
            #     self.set_motors_speed(left_speed, right_speed)
            #     if self.imu_yaw > target + math.pi / 2:
            #         self.set_motors_speed(0.0, 0.0)
            #         # switch to RTK mode
            #         self.switch_state("r")
            #         self.get_logger.info("UNLOADING complete, switch to RTK mode")
            #         return
        # 发布当前状态
        state_msg = String()
        state_msg.data = self.current_status
        self.state_pub.publish(state_msg)

    def handle_unloading_step(self):
        """出仓分步处理（定时器回调，每100ms执行一次）"""
        if self.unloading_phase is None:
            self.get_logger().warn("[ROSNode] 出仓阶段未初始化，停止定时器")
            self.unloading_timer.cancel()
            self.unloading_timer = None
            return
        
        current_time = time.time()
        correction = self.rtk_nav.get_speed_correction(self.imu_yaw_rad)
        
        
        # ========== 阶段1：前进 ==========
        if self.unloading_phase == "UNLOADING_FORWARD":
            # 持续前进直到达到时间阈值
            if current_time - self.unloading_start_time < self.unloading_forword_threshold:
                # 前进：双电机正转（保持速度输出）
                left_speed = -(self.motor_ctrl.BASE_SPEED + correction)
                right_speed = self.motor_ctrl.BASE_SPEED + correction
                self.set_motors_speed(left_speed, right_speed)
                self.get_logger().info(f"[UNLOADING] 前进阶段 - 已持续{current_time - self.unloading_start_time:.1f}秒")
            else:
                # 前进时间到，切换到转向阶段
                self.get_logger().info("[UNLOADING] 前进阶段完成，进入转向阶段")
                self.unloading_phase = "UNLOADING_TURN"
                # 初始化转向目标（基于当前IMU航向）
                if self.rtk_nav.imu_yaw is None:
                    self.get_logger().warn("[UNLOADING] IMU航向角未获取，使用默认0度作为基准")
                    self.unloading_turn_target_rad = math.pi / 2  # 90度
                else:
                    self.unloading_turn_target_rad = self.rtk_nav.imu_yaw + math.pi / 2
                    # 角度归一化到[-π, π]
                    if self.unloading_turn_target_rad > math.pi:
                        self.unloading_turn_target_rad -= 2 * math.pi
        
        # ========== 阶段2：转向 ==========
        elif self.unloading_phase == "UNLOADING_TURN":
            if self.rtk_nav.imu_yaw is None:
                self.get_logger().warn("[UNLOADING] IMU航向角未获取，暂不转向")
                return
            
            # 右转：
            left_speed = -0.3 * self.motor_ctrl.BASE_SPEED
            right_speed = -0.3 * self.motor_ctrl.BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
            
            # 检查是否达到转向目标
            yaw_diff = abs(self.rtk_nav.imu_yaw - self.unloading_turn_target_rad)
            self.get_logger().debug(f"[UNLOADING] 转向阶段 - 当前航向{self.rtk_nav.imu_yaw:.2f}rad，目标{self.unloading_turn_target_rad:.2f}rad，差值{yaw_diff:.2f}rad")
            
            # 角度差小于0.05rad（约2.86度）视为转向完成
            
            if yaw_diff < 0.05:
                self.get_logger().info("[UNLOADING] 转向阶段完成，出仓结束")
                self.unloading_phase = "COMPLETE"
                # 停止电机
                self.set_motors_speed(0.0, 0.0)
                # 自动切换到RTK模式
                self.current_control_mode = ControlMode.RTK_NAV
                # 停止定时器
                self.unloading_timer.cancel()
                self.unloading_timer = None
            elif current_time - self.unloading_start_time > 2 * self.unloading_forword_threshold:
                self.get_logger().warn(f"[UNLOADING] Timeout: 转向阶段超时，强制完成出仓")
                self.current_control_mode = ControlMode.RTK_NAV
                self.unloading_phase = "COMPLETE"

        
        # ========== 阶段3：完成 ==========
        elif self.unloading_phase == "COMPLETE":
            self.get_logger().info("[UNLOADING] 出仓流程完成")
            self.unloading_timer.cancel()
            self.unloading_timer = None
            
    def handle_loading_step(self):
        if self.loading_phase is None:
            self.get_logger().warn("[ROSNode] 进仓阶段未初始化，停止定时器")
            if self.loading_timer is not None:  # 修复BUG2：判空再取消
                self.loading_timer.cancel()
                self.loading_timer = None
            return
        try:
            current_time = time.time()
            correction = self.rtk_nav.get_speed_correction(self.imu_yaw_rad)
            
            # ========== 阶段1：调整进仓角度（左转90度） ==========
            if self.loading_phase == "LOADING_TURN":
                if self.rtk_nav.imu_yaw is None:
                    self.get_logger().warn("[LOADING] IMU航向角未获取，暂不转向")
                    return
                
                # 双电机转（=出仓转向）
                left_speed = -0.3 * self.motor_ctrl.BASE_SPEED
                right_speed = -0.3 * self.motor_ctrl.BASE_SPEED
                self.set_motors_speed(left_speed, right_speed)
                # 初始化转向目标（基于当前IMU航向）
                # if self.rtk_nav.imu_yaw is None:
                #     self.get_logger().warn("[UNLOADING] IMU航向角未获取，使用默认0度作为基准")
                #     self.unloading_turn_target_rad = math.pi / 2  # 90度
                # else:
                #     self.unloading_turn_target_rad = self.rtk_nav.imu_yaw + math.pi / 2
                #     # 角度归一化到[-π, π]
                #     if self.unloading_turn_target_rad > math.pi:
                #         self.unloading_turn_target_rad -= 2 * math.pi
                
                # 计算角度差
                yaw_diff = abs(self.rtk_nav.imu_yaw - self.loading_turn_target_rad)
                
                self.get_logger().info(
                    f"[LOADING] 角度调整阶段 - 当前航向{self.rtk_nav.imu_yaw:.2f}rad，"
                    f"目标{self.loading_turn_target_rad:.2f}rad，差值{yaw_diff:.2f}rad"
                )
                
                # 角度差小于0.05rad视为调整完成
                if yaw_diff < 0.05:
                    self.get_logger().info("[LOADING] 角度调整完成，进入后退进仓阶段")
                    self.loading_phase = "LOADING_BACKWARD"
                    self.loading_backward_start_time = current_time  # 记录后退开始时间
                    self.set_motors_speed(0.0, 0.0)  # 短暂停止，准备后退
                # 角度调整超时（5秒）
                elif current_time - self.loading_start_time > 5.0:
                    self.get_logger().warn("[LOADING] 角度调整超时，强制进入后退阶段")
                    self.loading_phase = "LOADING_BACKWARD"
                    self.loading_backward_start_time = current_time
            
            # ========== 阶段2：后退进仓 ==========
            elif self.loading_phase == "LOADING_BACKWARD":
                # 持续后退直到达到时间阈值
                if current_time - self.loading_backward_start_time < self.loading_backward_threshold:
                    # 后退：双电机反转（与前进相反）
                    left_speed = self.motor_ctrl.BASE_SPEED + correction
                    right_speed = -(self.motor_ctrl.BASE_SPEED + correction)
                    self.set_motors_speed(left_speed, right_speed)
                    self.get_logger().info(f"[LOADING] 后退进仓阶段 - 已持续{current_time - self.loading_backward_start_time:.1f}秒")
                else:
                    self.get_logger().info("[LOADING] 后退进仓完成，进入完成阶段")
                    self.loading_phase = "COMPLETE"
                    # self.set_motors_speed(0, 0)  # 停止电机
                    # 切换到待机模式 # STOP
                    # self.switch_state("z")
                    # self.current_control_mode = ControlMode.NORMAL
            
            # ========== 阶段3：进仓完成 ==========
            elif self.loading_phase == "COMPLETE":
                self.get_logger().info("[LOADING] 进仓流程完成")
                self.set_motors_speed(0, 0)  # 停止电机
                # 切换到待机模式 # STOP
                self.switch_state("z")
                self.loading_timer.cancel()
                self.loading_timer = None
        except Exception as e:
            self.get_logger().error(f"[LOADING] 执行异常：{str(e)}")

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
        motor_node.get_logger().fatal(f"[ROSNode] 节点运行异常：{str(e)}")
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