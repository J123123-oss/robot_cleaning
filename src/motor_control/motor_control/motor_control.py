#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import struct
import time
import os
import sys
import math
from typing import Optional, List, Dict, Tuple, Generator, Union
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motor_control.motor_driver import CanMotorDriver
from motor_control.remote_control import SBUSRemoteController

# 全局常量定义
STATE_DICT = {
    'z': "STOP",
    'x': "START",
    'w': "FORWARD",
    's': "BACKWARD",
    'a': "TURN_LEFT",
    'd': "TURN_RIGHT",
}
CURRENT_STATE = "STOP"
BASE_SPEED = 1000.0 # 目标速度（dps）= BASE_SPEED*100
GLOBAL_MOTOR_CONFIG = [
    {"id": 1},
    {"id": 2}
]

# 控制模式枚举（兼容遥控器类）
class ControlMode:
    REMOTE = "REMOTE"  # 字符串类型
    NORMAL = "NORMAL"  # 字符串类型

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')  # ROS2 节点初始化

        # 循环频率：5Hz
        self.rate = self.create_rate(10)

        # 初始化核心模块
        self.motor_ctrl = CanMotorDriver(node_name='can_motor_driver', channel='can0', interface='socketcan', baudrate=1000000)
        self.get_logger().info("[ROSNode] 开始初始化CAN串口...")
        if not self.motor_ctrl.create_can_bus():
            self.get_logger().warn("[ROSNode] CAN串口首次初始化失败，进入重连模式")
            if not self.motor_ctrl.reconnect_can_bus():
                self.get_logger().fatal("[ROSNode] CAN串口重连失败，无法继续运行，退出节点")
                rclpy.signal_shutdown("CAN串口重连失败")
                return
            
        # 初始化遥控器解析模块
        self.sbus_remote = SBUSRemoteController()
        if not self.sbus_remote._init_serial():
            self.get_logger().warn("[ROSNode] 遥控器串口初始化失败，仅支持原有控制逻辑")

        # ROS2 订阅器
        self.keyboard_sub = self.create_subscription(
            String,
            "/keyboard/control",
            self.keyboard_callback,
            10  # QoS深度
        )

        # ROS2 发布器
        self.state_pub = self.create_publisher(String, "/motor/state", 10)  # 电机状态
        self.speed_pub = self.create_publisher(Vector3, "/motor/current_speed", 10)  # 电机速度
        self.mode_pub = self.create_publisher(String, "/control/mode", 10)  # 控制模式发布

        # 初始化电机（进入START状态）
        self.switch_state('x')

    def keyboard_callback(self, msg: String) -> None:
        """键盘控制回调（仅在非遥控器模式下有效）"""
        # 尝试获取遥控器控制模式，兼容 SBUSRemoteController 类
        # is_remote_mode = False  # 默认为非遥控器模式
        # try:
        #     # 获取通道6归一化值（浮点数），判断是否为遥控器模式（接近-1.0）
        #     ch6_val = self.sbus_remote.get_channel_normalized(6)
        #     if abs(ch6_val - (-1.0)) < 0.1:
        #         is_remote_mode = True
        # except Exception as e:
        #     # 遥控器初始化失败或获取通道失败，均视为非遥控器模式
        #     self.get_logger().debug(f"遥控器通道获取失败：{e}，允许键盘控制")

        # # 遥控器模式下忽略键盘指令
        # if is_remote_mode:
        #     self.get_logger().info("[ROSNode] 当前为遥控器控制模式，忽略键盘指令")
        #     return
        
        key = msg.data.strip().lower()
        if key in STATE_DICT:
            self.switch_state(key)
        else:
            self.get_logger().warn(f"[ROSNode] 无效键盘指令：{key}，支持指令：{list(STATE_DICT.keys())}")

    def switch_state(self, key: str) -> None:
        """状态机切换逻辑"""
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
                self.motor_ctrl.set_velocity_closed_loop(motor["id"], 0)  # 初始速度0
                self.motor_ctrl.stop_motor(motor["id"])
                time.sleep(1)
                self.motor_ctrl.disable_drive(motor["id"])  # 非使能电机
                self.motor_ctrl.brake_lock(motor["id"])  # 抱闸锁死
                time.sleep(0.001)

        elif new_state == "START":
            # 启动：仅使能电机，不运动
            for motor in GLOBAL_MOTOR_CONFIG:
                self.motor_ctrl.set_acceleration(motor["id"], 5000)  # 设置加速度
                time.sleep(0.001)
                self.motor_ctrl.set_deceleration(motor["id"], 5000)  # 设置减速度
                # self.motor_ctrl.motor_set_mode(motor["id"], motor["run_mode"])
                # time.sleep(0.001)
                # self.motor_ctrl.motor_set_current_limit(motor["id"], motor["current_limit"])
                time.sleep(0.001)
                self.motor_ctrl.enable_drive(motor["id"])
                time.sleep(0.001)
                self.motor_ctrl.set_velocity_closed_loop(motor["id"], 0)  # 初始速度0
                time.sleep(0.001)

        elif new_state == "FORWARD":
            # 前进：双电机正转（需先确认电机转向，调整符号）
            left_speed = -BASE_SPEED
            right_speed = BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)

        elif new_state == "BACKWARD":
            # 后退：双电机反转
            left_speed = BASE_SPEED
            right_speed = -BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
        elif new_state == "TURN_LEFT":
            # 左转：左轮减速
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)
        elif new_state == "TURN_RIGHT":
            # 右转：右轮减速
            left_speed = -BASE_SPEED
            right_speed = -BASE_SPEED
            self.set_motors_speed(left_speed, right_speed)

        # 发布当前状态
        state_msg = String()
        state_msg.data = CURRENT_STATE
        self.state_pub.publish(state_msg)

    def rad_from_linear(self, linear_speed: float) -> float:
        """线速度转角速度（简化实现，可根据车轮半径调整）"""
        wheel_radius = 0.05  # 示例：车轮半径0.05m
        if wheel_radius <= 0:
            return 0.0
        return linear_speed / wheel_radius

    def set_motors_speed(self, left_speed: float, right_speed: float) -> None:
        """设置双电机速度（差速控制）"""
        # 左电机（ID=1）
        self.motor_ctrl.set_velocity_closed_loop(GLOBAL_MOTOR_CONFIG[0]["id"], left_speed)
        # 右电机（ID=2）
        self.motor_ctrl.set_velocity_closed_loop(GLOBAL_MOTOR_CONFIG[1]["id"], right_speed)
        
        # 构造并发布速度消息
        wheel_speed_msg = Vector3()
        wheel_speed_msg.x = left_speed    # 左轮角速度（rad/s）
        wheel_speed_msg.y = right_speed   # 右轮角速度（rad/s）
        wheel_speed_msg.z = 0.0           # 预留字段，无意义设为0
        self.speed_pub.publish(wheel_speed_msg)

    def run(self) -> None:
        """节点主循环（运行中检测串口状态和控制模式）"""
        while rclpy.ok():
            # 检查CAN串口是否正常打开
            if not self.motor_ctrl.bus:
                self.get_logger().warn("[ROSNode] CAN串口连接断开，尝试重连...")
                self.motor_ctrl.reconnect_can_bus()
            
            # 模式判断（保持原有逻辑）
            current_mode = ControlMode.NORMAL  # 遥控器初始化失败，直接用NORMAL模式
            
            # 发布控制模式
            mode_msg = String()
            mode_msg.data = str(current_mode)
            self.mode_pub.publish(mode_msg)
            
            # 后续原有逻辑（心跳、速度控制等）...
            if current_mode == ControlMode.REMOTE:
                if CURRENT_STATE != "START":
                    self.switch_state('x')
                    self.get_logger().info("使能~~~~~")
                try:
                    left_speed = right_speed = BASE_SPEED
                    self.set_motors_speed(left_speed, right_speed)
                    self.get_logger().info(f"[RemoteControl] 左轮：{left_speed:.2f}，右轮：{right_speed:.2f}")
                except Exception as e:
                    self.get_logger().warn(f"[ROSNode] 获取遥控器速度失败：{e}")
                    self.set_motors_speed(0.0, 0.0)
            else:
                # 正常模式：持续发送心跳
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
                    left_speed = 0.0
                    right_speed = BASE_SPEED
                elif CURRENT_STATE == "TURN_RIGHT":
                    left_speed = -BASE_SPEED
                    right_speed = 0.0
                else:
                    left_speed = 0.0
                    right_speed = 0.0
                self.set_motors_speed(left_speed, right_speed)
            
            # 关键：手动触发一次回调处理 + 保证延时生效
            rclpy.spin_once(self, timeout_sec=0.01)
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    # 创建电机控制节点
    motor_node = MotorControlNode()

    try:
        # 方案1：使用 spin 支撑节点运行（推荐，符合 ROS2 规范）
        rclpy.spin(motor_node)
        # 方案2：若需自定义循环，保留 run() 方法，同时增加 spin_once
        # motor_node.run()
    except KeyboardInterrupt:
        motor_node.get_logger().info("[ROSNode] 收到中断信号，即将退出")
    except Exception as e:
        motor_node.get_logger().fatal(f"[ROSNode] 节点运行异常：{str(e)}")
    finally:
        # 原有清理逻辑不变...
        if motor_node:
            motor_node.get_logger().info("[ROSNode] 退出时停止所有电机...")
            for motor in GLOBAL_MOTOR_CONFIG:
                try:
                    motor_node.motor_ctrl.set_velocity_closed_loop(motor["id"], 0.0)
                    motor_node.motor_ctrl.stop_motor(motor["id"])
                    time.sleep(0.1)
                    motor_node.motor_ctrl.disable_drive(motor["id"])
                    motor_node.motor_ctrl.brake_lock(motor["id"])
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
        print("[ROSNode] 节点退出完成")

if __name__ == "__main__":
    main()