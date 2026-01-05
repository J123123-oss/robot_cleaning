#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import struct
import time
import os
import sys
import math
from typing import Optional, List, Dict, Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_control.motor_driver import CanMotorDriver
from motor_control.remote_control import SBUSRemoteController

# -------------------------- 全局配置与枚举 --------------------------
STATE_DICT = {
    'z': "STOP",
    'x': "START",
    'w': "FORWARD",
    's': "BACKWARD",
    'a': "TURN_LEFT",
    'd': "TURN_RIGHT",
}
CURRENT_STATE = "STOP"

BASE_SPEED = 100.0  # 导航目标速度（dps）= BASE_SPEED*100
MAX_SPEED = 260.0   # 遥控器最大速度
MIN_SPEED = -160.0  # 遥控器最小速度
# 通道灵敏度系数（可微调，0~1之间，用于控制通道对速度的影响程度）
CH2_SENSITIVITY = 1.0  # 前进后退灵敏度
CH3_SENSITIVITY = 1.0  # 左右旋转灵敏度
DEAD_ZONE = 0.05       # 控制死区

GLOBAL_MOTOR_CONFIG = [
    {"id": 1},
    {"id": 2}
]

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

        # 4. ROS2 发布器
        self.state_pub = self.create_publisher(String, "/motor/state", 10)  # 电机状态
        self.speed_pub = self.create_publisher(Vector3, "/motor/current_speed", 10)  # 电机当前速度
        self.mode_pub = self.create_publisher(String, "/control/mode", 10)  # 当前控制模式

        # 全局变量
        self.current_control_mode = ControlMode.NORMAL  # 默认普通模式
        # self.current_control_mode = self.sbus_remote.control_mode  # 默认普通模式
        self.rtk_left_speed = 0.0  # 存储RTK订阅的左轮速度
        self.rtk_right_speed = 0.0 # 存储RTK订阅的右轮速度

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
            if CURRENT_STATE != "START":
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
            state_msg.data = str(CURRENT_STATE)
            self.state_pub.publish(state_msg)

            # 按当前状态赋值速度
            if CURRENT_STATE == "FORWARD":
                left_speed = -BASE_SPEED
                right_speed = BASE_SPEED
            elif CURRENT_STATE == "BACKWARD":
                left_speed = BASE_SPEED
                right_speed = -BASE_SPEED
            elif CURRENT_STATE == "TURN_LEFT":
                left_speed = -BASE_SPEED
                right_speed = -BASE_SPEED
            elif CURRENT_STATE == "TURN_RIGHT":
                left_speed = BASE_SPEED
                right_speed = BASE_SPEED
            else:
                left_speed = 0.0
                right_speed = 0.0
            self.set_motors_speed(left_speed, right_speed)

        elif self.current_control_mode == ControlMode.RTK_NAV:
            # RTK导航模式（新增逻辑：使用RTK订阅的速度）
            if CURRENT_STATE != "START":
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
            if CURRENT_STATE != "START":
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
            if self.current_control_mode != ControlMode.RTK_NAV:
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

    def switch_state(self, key: str) -> None:
        """状态机切换逻辑（完全保留原有功能）"""
        global CURRENT_STATE
        new_state = STATE_DICT[key]
        if new_state == CURRENT_STATE:
            self.get_logger().info(f"[ROSNode] 已处于{new_state}状态，无需切换")
            return

        self.get_logger().info(f"[ROSNode] 状态切换：{CURRENT_STATE} → {new_state}")
        CURRENT_STATE = new_state

        # 状态执行逻辑
        if new_state == "STOP":
            # 停止：失能所有电机
            for motor in GLOBAL_MOTOR_CONFIG:
                self.motor_ctrl.motor_set_speed(motor["id"], 0)  # 初始速度0
                time.sleep(0.01)
                self.motor_ctrl.motor_disable(motor["id"])

        elif new_state == "START":
            # 启动：仅使能电机，不运动
            self.motor_ctrl.initialize_motors()
            time.sleep(0.001)

        elif new_state == "FORWARD":
            # 前进：双电机正转
            left_speed = -BASE_SPEED
            right_speed = BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)

        elif new_state == "BACKWARD":
            # 后退：双电机反转
            left_speed = BASE_SPEED
            right_speed = -BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
        elif new_state == "TURN_LEFT":
            # 左转
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
        elif new_state == "TURN_RIGHT":
            # 右转
            left_speed = -BASE_SPEED
            right_speed = -BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)

        # 发布当前状态
        state_msg = String()
        state_msg.data = CURRENT_STATE
        self.state_pub.publish(state_msg)

    def set_motors_speed(self, left_speed: float, right_speed: float) -> None:
        """设置双电机速度（完全保留原有功能）"""
        # 左电机（ID=1）
        self.motor_ctrl.motor_set_speed(GLOBAL_MOTOR_CONFIG[0]["id"], left_speed)
        # 右电机（ID=2）
        self.motor_ctrl.motor_set_speed(GLOBAL_MOTOR_CONFIG[1]["id"], right_speed)

        # 构造并发布速度消息
        wheel_speed_msg = Vector3()
        wheel_speed_msg.x = left_speed    # 左轮角速度
        wheel_speed_msg.y = right_speed   # 右轮角速度
        wheel_speed_msg.z = 0.0           # 预留字段
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
            for motor in GLOBAL_MOTOR_CONFIG:
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